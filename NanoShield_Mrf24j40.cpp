/*
 * File:   NanoShield_Mrf24j40.cpp
 * copyright Circuitar Eletr�nicos, 2014
 * modified BSD License / apache license
 */

#include "NanoShield_Mrf24j40.h"
//#include "arduPi.h"
// aMaxPHYPacketSize = 127, from the 802.15.4-2006 standard.
static uint8_t rx_buf[127];

// essential for obtaining the data frame only
// bytes_MHR = 2 Frame control + 1 sequence number + 2 panid + 2 shortAddr Destination + 2 shortAddr Source
static int bytes_MHR = 9;
static int bytes_FCS = 2; // FCS length = 2
static int bytes_nodata = bytes_MHR + bytes_FCS; // no_data bytes in PHY payload,  header length + FCS

static int ignoreBytes = 0; // bytes to ignore, some modules behaviour.

static boolean bufPHY = false; // flag to buffer all bytes in PHY Payload, or not

volatile uint8_t flag_got_rx;
volatile uint8_t flag_got_tx;

static rx_info_t rx_info;
static tx_info_t tx_info;


/**
 * Constructor MRF24J Object.
 * @param pin_reset, @param pin_chip_select, @param pin_interrupt
 */
#ifdef RASPBERRY
NanoShield_Mrf24j40::NanoShield_Mrf24j40(int cs_pin) {

    _pin_cs =  cs_pin;
}
#else
NanoShield_Mrf24j40::NanoShield_Mrf24j40(int pin_reset, int pin_chip_select, int pin_interrupt) {
    _pin_reset = pin_reset;
    _pin_cs = pin_chip_select;
    _pin_int = pin_interrupt;

    pinMode(_pin_reset, OUTPUT);
    pinMode(_pin_cs, OUTPUT);
    pinMode(_pin_int, INPUT);

    SPI.setBitOrder(MSBFIRST) ;
    SPI.setDataMode(SPI_MODE0);
    SPI.begin();
}
#endif
void NanoShield_Mrf24j40::begin(void){
    pinMode(_pin_cs,OUTPUT);
    SPI.setBitOrder(MSBFIRST) ;
    SPI.setDataMode(SPI_MODE0);
    SPI.begin();
}

void NanoShield_Mrf24j40::reset(void) {
    digitalWrite(_pin_reset, LOW);
    delay(10);  // just my gut
    digitalWrite(_pin_reset, HIGH);
    delay(20);  // from manual
}

byte NanoShield_Mrf24j40::read_short(byte address) {
    digitalWrite(_pin_cs, LOW);
    // 0 top for short addressing, 0 bottom for read
    SPI.transfer(address<<1 & 0b01111110);
    byte ret = SPI.transfer(0x00);
    digitalWrite(_pin_cs, HIGH);
    return ret;
}

byte NanoShield_Mrf24j40::read_long(word address) {
    digitalWrite(_pin_cs, LOW);
    byte ahigh = address >> 3;
    byte alow = address << 5;
    SPI.transfer(0x80 | ahigh);  // high bit for long
    SPI.transfer(alow);
    byte ret = SPI.transfer(0);
    digitalWrite(_pin_cs, HIGH);
    return ret;
}


void NanoShield_Mrf24j40::write_short(byte address, byte data) {
    digitalWrite(_pin_cs, LOW);
    // 0 for top short address, 1 bottom for write
    SPI.transfer((address<<1 & 0b01111110) | 0x01);
    SPI.transfer(data);
    digitalWrite(_pin_cs, HIGH);
}

void NanoShield_Mrf24j40::write_long(word address, byte data) {
    digitalWrite(_pin_cs, LOW);
    byte ahigh = address >> 3;
    byte alow = address << 5;
    SPI.transfer(0x80 | ahigh);  // high bit for long
    SPI.transfer(alow | 0x10);  // last bit for write
    SPI.transfer(data);
    digitalWrite(_pin_cs, HIGH);
}

word NanoShield_Mrf24j40::get_pan(void) {
    byte panh = read_short(MRF_PANIDH);
    return panh << 8 | read_short(MRF_PANIDL);
}

void NanoShield_Mrf24j40::set_pan(word panid) {
    write_short(MRF_PANIDH, panid >> 8);
    write_short(MRF_PANIDL, panid & 0xff);
}

void NanoShield_Mrf24j40::address16_write(word address16) {
    write_short(MRF_SADRH, address16 >> 8);
    write_short(MRF_SADRL, address16 & 0xff);
}

word NanoShield_Mrf24j40::address16_read(void) {
    byte a16h = read_short(MRF_SADRH);
    return a16h << 8 | read_short(MRF_SADRL);
}

/**
 * Simple send 16, with acks, not much of anything.. assumes src16 and local pan only.
 * @param data
 */
void NanoShield_Mrf24j40::send16(word dest16, char * data) {
    byte len = strlen(data); // get the length of the char* array
    int i = 0;
    write_long(i++, bytes_MHR); // header length
    // +ignoreBytes is because some module seems to ignore 2 bytes after the header?!.
    // default: ignoreBytes = 0;
    write_long(i++, bytes_MHR+ignoreBytes+len);

    // 0 | pan compression | ack | no security | no data pending | data frame[3 bits]
    //write_long(i++, 0b01100001); // first byte of Frame Control
    write_long(i++, 0b01000001); // first byte of Frame Control (no ACK)
    // 16 bit source, 802.15.4 (2003), 16 bit dest,
    write_long(i++, 0b10001000); // second byte of frame control
    write_long(i++, 1);  // sequence number 1

    word panid = get_pan();

    write_long(i++, panid & 0xff);  // dest panid
    write_long(i++, panid >> 8);
    write_long(i++, dest16 & 0xff);  // dest16 low
    write_long(i++, dest16 >> 8); // dest16 high

    word src16 = address16_read();
    write_long(i++, src16 & 0xff); // src16 low
    write_long(i++, src16 >> 8); // src16 high

    // All testing seems to indicate that the next two bytes are ignored.
    //2 bytes on FCS appended by TXMAC
    i+=ignoreBytes;
    for (int q = 0; q < len; q++) {
        write_long(i++, data[q]);
    }
    // ack on, and go!
    //write_short(MRF_TXNCON, (1<<MRF_TXNACKREQ | 1<<MRF_TXNTRIG));
    write_short(MRF_TXNCON, (1<<MRF_TXNTRIG)); // (no ACK)
}

void NanoShield_Mrf24j40::set_interrupts(void) {
    // interrupts for rx and tx normal complete
    write_short(MRF_INTCON, 0b11110110);
}

/** use the 802.15.4 channel numbers..
 */
void NanoShield_Mrf24j40::set_channel(byte channel) {
    write_long(MRF_RFCON0, (((channel - 11) << 4) | 0x03));
}

void NanoShield_Mrf24j40::init(void) {
    /*
    // Seems a bit ridiculous when I use reset pin anyway
    write_short(MRF_SOFTRST, 0x7); // from manual
    while (read_short(MRF_SOFTRST) & 0x7 != 0) {
        ; // wait for soft reset to finish
    }
    */
    write_short(MRF_PACON2, 0x98); // – Initialize FIFOEN = 1 and TXONTS = 0x6.
    write_short(MRF_TXSTBL, 0x95); // – Initialize RFSTBL = 0x9.

    write_long(MRF_RFCON0, 0x03); // – Initialize RFOPT = 0x03.
    write_long(MRF_RFCON1, 0x01); // – Initialize VCOOPT = 0x02.
    write_long(MRF_RFCON2, 0x80); // – Enable PLL (PLLEN = 1).
    write_long(MRF_RFCON6, 0x90); // – Initialize TXFIL = 1 and 20MRECVR = 1.
    write_long(MRF_RFCON7, 0x80); // – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
    write_long(MRF_RFCON8, 0x10); // – Initialize RFVCO = 1.
    write_long(MRF_SLPCON1, 0x21); // – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.
    write_short(MRF_TRISGPIO, 0x08); // – Set GPIO3 to high to enable PA regulator in MRF24J40MC
    write_short(MRF_GPIO, 0x08);
    write_long(MRF_TESTMODE, 0x0F); // – Enable PA/LNA control

    //  Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and
    //  Nonbeacon-Enabled Networks�?):
    write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
    write_short(MRF_CCAEDTH, 0x60); // – Set CCA ED threshold.
    write_short(MRF_BBREG6, 0x40); // – Set appended RSSI value to RXFIFO.
//    set_interrupts();
    set_channel(15);
    // max power is by default.. just leave it...
    // Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)�?.
    write_short(MRF_RFCTL, 0x04); //  – Reset RF state machine.
    write_short(MRF_RFCTL, 0x00); // part 2
    delay(1); // delay at least 192usec
}

/**
 * Call this from within an interrupt handler connected to the MRFs output
 * interrupt pin.  It handles reading in any data from the module, and letting it
 * continue working.
 * Only the most recent data is ever kept.
 */
void NanoShield_Mrf24j40::interrupt_handler(void) {
    uint8_t last_interrupt = read_short(MRF_INTSTAT);
    if (last_interrupt & MRF_I_RXIF) {
        flag_got_rx++;
        // read out the packet data...
        //noInterrupts();
        rx_disable();
        // read start of rxfifo for, has 2 bytes more added by FCS. frame_length = m + n + 2
        uint8_t frame_length = read_long(0x300);

        // buffer all bytes in PHY Payload
        if(bufPHY){
            int rb_ptr = 0;
            for (int i = 0; i < frame_length; i++) { // from 0x301 to (0x301 + frame_length -1)
                rx_buf[rb_ptr++] = read_long(0x301 + i);
            }
        }

        // buffer data bytes
        int rd_ptr = 0;
        // from (0x301 + bytes_MHR) to (0x301 + frame_length - bytes_nodata - 1)
        for (int i = 0; i < rx_datalength(); i++) {
            rx_info.rx_data[rd_ptr++] = read_long(0x301 + bytes_MHR + i);
        }

        rx_info.frame_length = frame_length;
        // same as datasheet 0x301 + (m + n + 2) <-- frame_length
        rx_info.lqi = read_long(0x301 + frame_length);
        // same as datasheet 0x301 + (m + n + 3) <-- frame_length + 1
        rx_info.rssi = read_long(0x301 + frame_length + 1);

        rx_enable();
        //interrupts();
    }
    if (last_interrupt & MRF_I_TXNIF) {
        flag_got_tx++;
        uint8_t tmp = read_short(MRF_TXSTAT);
        // 1 means it failed, we want 1 to mean it worked.
        tx_info.tx_ok = !(tmp & ~(1 << TXNSTAT));
        tx_info.retries = tmp >> 6;
        tx_info.channel_busy = (tmp & (1 << CCAFAIL));
    }
}


/**
 * Call this function periodically, it will invoke your nominated handlers
 */
void NanoShield_Mrf24j40::check_flags(void (*rx_handler)(void), void (*tx_handler)(void)){
    // TODO - we could check whether the flags are > 1 here, indicating data was lost?
    if (flag_got_rx) {
        flag_got_rx = 0;
        rx_handler();
    }
    if (flag_got_tx) {
        flag_got_tx = 0;
        tx_handler();
    }
}

/**
 * Set RX mode to promiscuous, or normal
 */
void NanoShield_Mrf24j40::set_promiscuous(boolean enabled) {
    if (enabled) {
        write_short(MRF_RXMCR, 0x01);
    } else {
        write_short(MRF_RXMCR, 0x00);
    }
}

rx_info_t * NanoShield_Mrf24j40::get_rxinfo(void) {
    return &rx_info;
}

tx_info_t * NanoShield_Mrf24j40::get_txinfo(void) {
    return &tx_info;
}

uint8_t * NanoShield_Mrf24j40::get_rxbuf(void) {
    return rx_buf;
}

int NanoShield_Mrf24j40::rx_datalength(void) {
    return rx_info.frame_length - bytes_nodata;
}

void NanoShield_Mrf24j40::set_ignoreBytes(int ib) {
    // some modules behaviour
    ignoreBytes = ib;
}

/**
 * Set bufPHY flag to buffer all bytes in PHY Payload, or not
 */
void NanoShield_Mrf24j40::set_bufferPHY(boolean bp) {
    bufPHY = bp;
}

boolean NanoShield_Mrf24j40::get_bufferPHY(void) {
    return bufPHY;
}

/**
 * Set PA/LNA external control
 */
void NanoShield_Mrf24j40::set_palna(boolean enabled) {
    if (enabled) {
        write_long(MRF_TESTMODE, 0x07); // Enable PA/LNA on MRF24J40MB module.
    }else{
        write_long(MRF_TESTMODE, 0x00); // Disable PA/LNA on MRF24J40MB module.
    }
}

void NanoShield_Mrf24j40::rx_flush(void) {
    write_short(MRF_RXFLUSH, 0x01);
}

void NanoShield_Mrf24j40::rx_disable(void) {
    write_short(MRF_BBREG1, 0x04);  // RXDECINV - disable receiver
}

void NanoShield_Mrf24j40::rx_enable(void) {
    write_short(MRF_BBREG1, 0x00);  // RXDECINV - enable receiver
}

void NanoShield_Mrf24j40::WakeUp(int wakePin) {
  unsigned int t0;
  byte wakeif;
  
  while (true) {
    if (wakePin) {
      // Wake up via WAKE pin
      digitalWrite(wakePin, HIGH);
      write_short(MRF_RFCTL, 0x04);
      write_short(MRF_RFCTL, 0x00);
      delay(3); // At least 2ms, according to datasheet
      digitalWrite(wakePin, LOW);
    } else {
      // Wake up via SPI
      write_short(MRF_WAKECON, 0x40);
      delayMicroseconds(50); // Delay at least 10us, according to forum post
      write_short(MRF_WAKECON, 0x00);
      write_short(MRF_RFCTL, 0x04);
      write_short(MRF_RFCTL, 0x00);
      delay(3); // At least 2ms, according to datasheet
    }
    
    // For one second, check that wake-up occured through WAKEIF bit
    t0 = millis();
    do {
      delay(1);
      wakeif = read_short(MRF_INTSTAT) & 0x40;
    } while (!wakeif && (millis() - t0 < 1000));
    
    // If the transceiver woke up, exit, otherwise try waking up again
    if (wakeif) break;
    
    // Wait for one second and try to wake up again
    delay(1000);
  }
}

void NanoShield_Mrf24j40::Sleep() {
  write_short(MRF_WAKECON, 0x80);
  write_short(MRF_SOFTRST, 0x04);
  write_short(MRF_SLPACK, 0x80);
}
