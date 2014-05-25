// Radio-Bootloader for Attiny84A and nRF24L01+, developped using Atmel Studio.
// See http://s.wangnick.de/doku.php?id=betriebsstundenzaehler for target hardware.
// (C) Copyright 2014 Sebastian Wangnick.
// Usage under "CC Attribution-Noncommercial-Share Alike 3.0 Unported" as described in http://creativecommons.org/licenses/by-nc-sa/3.0/ is granted.
//
//                                                MISO    MOSI
//                                nRF24L01 MOSI - DO      DI - nRF24L01 MISO
//                                                PA5     PA6
//                                                2019181716
//                                              +-+-+-+-+-+-+
//                     nRF24L01 SCK - USCK PA4 1+ o         + 15 PA7 - nRF24L01 CSN
//                           nRF24L01 CE - PA3 2+           + 14 NRESET
//  VCC - 22kOhm, 27kOhm GND - TX 115200 - PA2 3+           + 13 PB2 - nRF24L01 VCC
//          VCC - 2.2kOhm - HMC5883L SCL - PA1 4+           + 12 XTAL2
//          VCC - 2.2kOhm - HMC5883L SDA - PA0 5+           + 11 XTAL1
//                                              +-+-+-+-+-+-+
//                                                6 7 8 9 10 
//                                                    V G
//                                                    C N
//                                                    C D
// Fuses: Low:0x7F High:0xD6 Extended:0xFE
//  External crystal/osc, 16k/14CK+65ms
//  Brown-out detection at 1.8V
//  Preserve EEPROM through chip erase cycle
//  SPI enabled
//  Self-programming enabled
//  Clock divided by 8 at startup
//  DebugWire not enabled
//  CKOUT not enabled
//  Reset not disabled
//  Watchdog timer not always on

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#define F_CPU 8000000 // 8 MHz is safe as from 2.475V, 16MHz is safe only as from 3.825V, We operate at 3.3V -> 8MHz
#include <util/delay.h>
#include "nRF24L01.h"

void init (void) __attribute__ ((naked)) __attribute__ ((section (".bootsect")));
void init (void) {asm volatile ( "rjmp start" );}

void __do_copy_data (void) __attribute__ ((naked)) __attribute__ ((section (".text9")));
void __do_copy_data (void) {}
void __do_clear_bss (void) __attribute__ ((naked)) __attribute__ ((section (".text9")));
void __do_clear_bss (void) {}

uint8_t mcusr;

#define DEBUG_UART

#ifdef DEBUG_UART
#ifdef DEBUG_SWID
const char SWID[] PROGMEM = __FILE__ " " __DATE__ " " __TIME__;
const char* swid;
#endif

#include "ds.h"

void uart_puts (char* s) {
    while (*s) {
        uart_putc(*s++);
    }
}
void uart_putp (PGM_P s) {
    char c;
    while ((c = pgm_read_byte(s++))) {
        uart_putc(c);
    }
}
#define hex(digit) ((digit)+((digit)>9?'A'-10:'0'))
void uart_puthex2 (uint8_t val) {
    uart_putc(hex(val>>4));
    uart_putc(hex(val&0xF));
}
#define uart_puthex4(val) do{uart_puthex2(((uint16_t)(val))>>8);uart_puthex2(((uint16_t)(val))&0xFF);}while(0)
#endif

uint8_t spi_transfer (uint8_t data) {
    USIDR = data;
    USISR = 1<<USIOIF; // Clear completion flag
    while (!(USISR & 1<<USIOIF)) {
        USICR = 1<<USIWM0 | 1<<USICS1 | 1<<USICLK | 1<<USITC;
    }
    return USIDR;
}

inline void spi_write (uint8_t* buffer, uint8_t len) {
    while (len--) {
        spi_transfer(*buffer++);
    }
}

inline void spi_write_P (PGM_P buffer, uint8_t len) {
    while (len--) {
        spi_transfer(pgm_read_byte(buffer++));
    }
}

inline void spi_read (uint8_t* buffer, uint8_t len) {
    while (len--) {
        *buffer++ = spi_transfer(0xFF);
    }
}

inline void spi_init(void) {
    DDRA |= (1<<PINA4); // SPI CLK
    DDRA |= (1<<PINA5); // SPI DO
    DDRA &= ~(1<<PINA6); // SPI DI
    PORTA |= (1<<PINA6); // SPI DI pullup
}

#define CSN                 (1<<PINA7)
#define CE                  (1<<PINA3)
#define CE_HI               PORTA |= CE
#define CE_LO               PORTA &= ~CE
#define CSN_HI              PORTA |= CSN
#define CSN_LO              PORTA &= ~CSN
#define MIRF_CONFIG         (1<<EN_CRC|1<<CRCO)
#define CHANNEL			    76
#define ADDRLEN             2
const char SADDR[ADDRLEN] PROGMEM = "S1";
const char CADDR[ADDRLEN] PROGMEM = "C1";
#define DATALEN             16
#define SEQNO_NUMMASK       0x7F
#define SEQNO_CLOSE         0xFE
#define CMD_BOOTSTART       0x10
#define CMD_BOOTDATA        0x11
#define CMD_BOOTDATAEND     0x12
#define CMD_STANDBY         0x20
typedef struct {
    uint8_t port;
    uint8_t seqno; // 0..SEQNO_NUMMASK, wrapping back to 0, or SEQNO_CLOSE to abort comms
    uint8_t cmd;
    uint8_t len;
    uint16_t addr;
    uint8_t data[DATALEN];
} Payload;
Payload payload;

uint8_t mirf_write_register (uint8_t reg, uint8_t data) {
    CSN_LO;
    uint8_t status = spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(data);
    CSN_HI;
    return status;
}

#ifdef DEBUG_UART
inline uint8_t mirf_read_register (uint8_t reg) {
    CSN_LO;
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    uint8_t value = spi_transfer(NOP);
    CSN_HI;
    return value;
}
#endif

void mirf_write_address (uint8_t reg, PGM_P addr) {
    CSN_LO;
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_write_P(addr, ADDRLEN);
    CSN_HI;
}

// Emits payload bytes. Returns true if receiver replied (data contains reply).
uint8_t mirf_send (void) {
    uint8_t port = payload.port, seqno = payload.seqno, status = 0, cycle, retry;
    for (retry = 0; retry<40 && !(status & (1<<RX_DR)); retry++) {

        // Start TX phase
        mirf_write_register(STATUS, 1<<TX_DS|1<<MAX_RT|1<<RX_DR);
        mirf_write_register(CONFIG, MIRF_CONFIG | 0<<PRIM_RX | 1<<PWR_UP); // Power up, TX mode
        _delay_us(150); // Tpd2stby with external clock. Time required before CE high is allowed.

        payload.port = port;
        payload.seqno = seqno;
#ifdef DEBUG_UART
        uart_putp(PSTR("TX rty="));
        uart_puthex2(retry);
        uart_putp(PSTR(" prt="));
        uart_puthex2(port);
        uart_putp(PSTR(" sno="));
        uart_puthex2(seqno);
        uart_putc('\n');
#endif
        CSN_LO;
        spi_transfer(W_TX_PAYLOAD);
        spi_write((uint8_t*)&payload,sizeof(Payload));
        CSN_HI;

        CE_HI; // Start transmission
        _delay_us(15);
        CE_LO;

        for (cycle=0; cycle<200; cycle++) {
            status = mirf_read_register(STATUS);
            if (status & (1<<TX_DS|1<<MAX_RT)) break;
            _delay_us(100);
        }
        mirf_write_register(STATUS, 1<<TX_DS|1<<MAX_RT|1<<RX_DR);
        
#ifdef DEBUG_UART
        uart_putp(PSTR("TX sts="));
        uart_puthex2(status);
        uart_putp(PSTR(" cyc="));
        uart_puthex2(cycle);
        uart_putp(PSTR(" FIS="));
        uart_puthex2(mirf_read_register(FIFO_STATUS));
        uart_putc('\n');
#endif
        
        if (status & (1<<RX_DR)) {
            // Ignore any ACK payload
            CSN_LO;
            spi_transfer(FLUSH_RX);
            CSN_HI;
            status &= ~(1<<RX_DR);
        }
        if (!(status & (1<<TX_DS))) {
            CSN_LO;
            spi_transfer(FLUSH_TX);
            CSN_HI;
        } else {
            // Start RX phase
            mirf_write_register(CONFIG, MIRF_CONFIG | 1<<PRIM_RX | 1<<PWR_UP); // Power is up, switch to RX mode
            CE_HI;
            for (cycle=0; cycle<200; cycle++) {
                status = mirf_write_register(STATUS,1<<RX_DR);
                if (status & (1<<RX_DR)) { // Read data in
                    CSN_LO;
                    spi_transfer(R_RX_PAYLOAD);
                    spi_read((uint8_t*)&payload,sizeof(Payload));
                    CSN_HI;
                    asm volatile ("NOP"); // 50ns minimum before CSN can go low again. Lets play it safe here.
                    CSN_LO;
                    spi_transfer(FLUSH_RX);
                    CSN_HI;

#ifdef DEBUG_UART
                    uart_putp(PSTR("RX prt="));
                    uart_puthex2(payload.port);
                    uart_putp(PSTR(" sno="));
                    uart_puthex2(payload.seqno);
                    uart_putp(PSTR(" cmd="));
                    uart_puthex2(payload.cmd);
                    uart_putp(PSTR(" len="));
                    uart_puthex2(payload.len);
                    uart_putp(PSTR(" adr="));
                    uart_puthex4(payload.addr);
                    uart_putc('\n');
#endif
                    if ((!port || payload.port==port) && (payload.seqno==SEQNO_CLOSE || payload.seqno==((seqno+1)&SEQNO_NUMMASK))) {
                        break;
                    } else {
                        status &= ~(1<<RX_DR);
                    }
                }            
                _delay_us(100);
            }
            CE_LO;
            
#ifdef DEBUG_UART
            uart_putp(PSTR("RX sts="));
            uart_puthex2(status);
            uart_putp(PSTR(" cyc="));
            uart_puthex2(cycle);
            uart_putc('\n');
#endif            
        }  
        mirf_write_register(CONFIG, MIRF_CONFIG); // Power down
    }
    return status & (1<<RX_DR);  
}

#define SPM_PAGEMASK ~(SPM_PAGESIZE-1)
#define SPM_NOPAGE ((Address)-1)

typedef uint16_t Address;
uint8_t eepromwait;
uint8_t pa; // 0..SPM_PAGESIZE
Address spmaddr, spmpage; // 0..FLASHEND
uint8_t spmdata[SPM_PAGESIZE];
uint8_t spmdirty;

inline void flashinit (void) {
    eepromwait = 0;
    spmpage = SPM_NOPAGE;
    spmaddr = 0;
    spmdirty = 0;
}

uint8_t flashflush (void) { // Returns 0 on success, 1 if validation is not successful
    if (spmdirty) {
        if (!eepromwait) {
            eeprom_busy_wait();
            eepromwait = 1;
        }
        for (pa=0; pa<SPM_PAGESIZE; pa+=2) {
            boot_page_fill(pa,spmdata[pa+1]<<8|spmdata[pa]); // low byte is written first, then high byte as second
        }
        boot_page_erase(spmpage);
        boot_spm_busy_wait();		// Wait until the memory is erased.
        boot_page_write(spmpage);   // Store buffer in flash page. This also erases the temporary buffer again.
        boot_spm_busy_wait();       // Wait until the memory is written.
        // boot_rww_enable();          // Allow read access again such that a) verification can be done and b) the next page can be read
        uint8_t val;
        for (pa=0; pa<SPM_PAGESIZE; pa++) {
            val = pgm_read_byte(spmpage+pa);
            if (val!=spmdata[pa]) {
#ifdef DEBUG_UART
                uart_putc('@');
                uart_puthex4(spmpage+pa);
                uart_putc(':');
                uart_puthex2(spmdata[pa]);
                uart_putc('>');
                uart_puthex2(val);
                uart_putc('\n');
#endif
                return 1;
            }
        }
        spmdirty = 0;
    }
    return 0;
}

uint8_t flashwrite (uint8_t data) { // Returns 0 on success, 1 if writing was not successful
#ifdef DEBUG_UART
    uart_puthex2(data);
#endif

    if ((spmaddr&SPM_PAGEMASK)!=spmpage) {
        if (flashflush()) return 1;
        spmpage = spmaddr&SPM_PAGEMASK;
        for (pa=0; pa<SPM_PAGESIZE; pa++) {
            spmdata[pa] = pgm_read_byte(spmpage+pa);
        }
    }
    pa = spmaddr&~SPM_PAGEMASK;
    if (spmdata[pa]!=data) {
        spmdata[pa] = data;
        spmdirty = 1;
    }
    spmaddr++;
    return 0;
}

void appjump (void) __attribute__ ((naked)) __attribute__ ((section (".appjump")));
void appjump (void) {asm volatile ( "rjmp init" );}
#define APPRJMPLOC (FLASHEND+1-SPM_PAGESIZE)
// or ((Address)appjump)
#define APPRJMPCORR (SPM_PAGESIZE/2)

void start (void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
void start (void) {
    if (MCUSR&(1<<WDRF)) {
        appjump();
    }
    asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
    asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );
    asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
    asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );
    asm volatile ( "clr __zero_reg__" );									// GCC depends on register r1 set to 0
    asm volatile ( "out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG)) );	// set SREG to 0
    
    mcusr = MCUSR;
    MCUSR =	0;
    wdt_reset();
    wdt_disable();
    
#ifdef DEBUG_UART
#ifdef DEBUG_SWID
    swid = SWID+sizeof(SWID)-1;                  // AVR Studio compiles using ./../ in front of filename, have to strip
    while (swid!=SWID && pgm_read_byte(swid-1)!='/') swid--;
#endif    
    uart_init();
    uart_putc('\n');
#ifdef DEBUG_SWID
    uart_putp(swid);
    uart_putp(PSTR(", "));
#endif
    uart_putp(PSTR("MCUSR="));
    uart_puthex2(mcusr);
    uart_putc('\n');
#endif
   
    // Attempt to download application
    spi_init();
    DDRA |= CSN|CE;
    CE_LO;
    CSN_HI;
    _delay_ms(100);
    // This is absolutely on the safe side. The radio needs 100ms as from 1.9V.
    // Our bodlevel is 2.7V, and we wait 65ms (fuse) for crystal stabilisation.
    mirf_write_register(RX_PW_P0, sizeof(payload));
    mirf_write_register(RX_PW_P1, sizeof(payload));
    mirf_write_register(RF_SETUP,1<<RF_PWR_LOW|1<<RF_PWR_HIGH); // 0db, 1Mbps (default is 2Mbps)
    mirf_write_register(RF_CH, CHANNEL);
    mirf_write_register(SETUP_AW, ADDRLEN-2);
    mirf_write_address(TX_ADDR, SADDR);
    mirf_write_address(RX_ADDR_P0, SADDR);
    mirf_write_address(RX_ADDR_P1, CADDR);
    mirf_write_register(SETUP_RETR,0<<ARD|15<<ARC); // 250us ARD, 15 retries. Setting this (to whatever) seems to be mandatory to be able to send at all.
    // Default values after reset:
    
#ifdef RESET_CLEANUP
    mirf_write_register(EN_RXADDR, 1<<ERX_P0|1<<ERX_P1);
    //mirf_write_register(DYNPD,0);
    // Cleanup. Sometimes some stuff gets stuck
    CSN_LO;
    spi_transfer(FLUSH_RX);
    CSN_HI;
    asm volatile ("NOP"); // 50ns minimum before CSN can go low again. Lets play it safe here.
    CSN_LO;
    spi_transfer(FLUSH_TX);
    CSN_HI;
    mirf_write_register(STATUS, 1<<TX_DS|1<<MAX_RT|1<<RX_DR);
#endif
    
    uint8_t success, i, val;
    uint16_t appresetvector = 0xFFFF;
    payload.port = 0;
    payload.seqno = 0;
    payload.cmd = CMD_BOOTSTART;
    success = mirf_send();
    if (success) {
#ifdef DEBUG_UART
        uart_putp(PSTR("flashinit()\n"));
#endif
        flashinit();
        while (success && payload.seqno!=SEQNO_CLOSE) {
#ifdef DEBUG_UART
            uart_putp(PSTR("flashwrite()\n"));
#endif
            spmaddr = payload.addr;
            i = 0;
            if (spmaddr==0) {
                val = payload.data[1];
                success = (val&0xF0)==0xC0;
                appresetvector = ((val&0x0F)<<8)|payload.data[0];
#ifdef DEBUG_UART
                uart_putp(PSTR("appresetvector "));
                uart_puthex4(appresetvector);
#endif
#ifdef DEBUG_UART
                uart_putc('\n');
#endif
                spmaddr = 2;
                i = 2;
            }
#ifdef DEBUG_UART
            uart_putp(PSTR("spmaddr "));
            uart_puthex4(spmaddr);
            uart_putp(PSTR(": "));
#endif
            for (; i<sizeof(payload.data); i++) {
                success = success || flashwrite(payload.data[i]);
            }
#ifdef DEBUG_UART
            uart_putc('\n');
#endif
            success = success || mirf_send();
        }
#ifdef DEBUG_UART
        uart_putp(PSTR("flashflush()\n"));
#endif
        success = success || flashflush();
        if (success) {
            appresetvector += APPRJMPCORR; // RJMP appstart
        } else {
            appresetvector = 0x001F; // RJMP init
        }
        spmaddr = APPRJMPLOC;
        success = success
            || flashwrite((uint8_t)appresetvector)
            || flashwrite(((appresetvector>>8)&0x0F)|0xC0);
        
        payload.seqno = SEQNO_CLOSE;
        payload.addr = success;
        mirf_send();
    }
#ifdef DEBUG_UART
    uart_putp(PSTR("success="));
    uart_puthex2(success);
    uart_putc('\n');
#endif
    if (!success) {
        _delay_ms(10000);
    }
#ifdef DEBUG_UART
    uart_putp(PSTR("rjmp appjump\n"));
#endif
    asm volatile ( "rjmp appjump" );
}
    
    

    



