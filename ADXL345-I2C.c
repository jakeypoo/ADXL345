/********** INCOMPLETE*************/

#include <avr/io.h>

// TWSR values (not bits)
// (taken from avr-libc twi.h - thank you Marek Michalkiewicz)
// Master
#define TW_START				0x08
#define TW_REP_START				0x10
// Master Transmitter
#define TW_MT_SLA_ACK				0x18
#define TW_MT_SLA_NACK				0x20
#define TW_MT_DATA_ACK				0x28
#define TW_MT_DATA_NACK				0x30
#define TW_MT_ARB_LOST				0x38
// Master Receiver
#define TW_MR_ARB_LOST				0x38
#define TW_MR_SLA_ACK				0x40
#define TW_MR_SLA_NACK				0x48
#define TW_MR_DATA_ACK				0x50
#define TW_MR_DATA_NACK				0x58
// Slave Transmitter
#define TW_ST_SLA_ACK				0xA8
#define TW_ST_ARB_LOST_SLA_ACK			0xB0
#define TW_ST_DATA_ACK				0xB8
#define TW_ST_DATA_NACK				0xC0
#define TW_ST_LAST_DATA				0xC8
// Slave Receiver
#define TW_SR_SLA_ACK				0x60
#define TW_SR_ARB_LOST_SLA_ACK			0x68
#define TW_SR_GCALL_ACK				0x70
#define TW_SR_ARB_LOST_GCALL_ACK		0x78
#define TW_SR_DATA_ACK				0x80
#define TW_SR_DATA_NACK				0x88
#define TW_SR_GCALL_DATA_ACK			0x90
#define TW_SR_GCALL_DATA_NACK			0x98
#define TW_SR_STOP				0xA0
// Misc
#define TW_NO_INFO				0xF8
#define TW_BUS_ERROR				0x00

#define WRITE_sda() DDRC = DDRC | 0b00010000 //SDA must be output when writing
#define READ_sda()  DDRC = DDRC & 0b11101111 //SDA must be input when reading - don't forget the resistor on SDA!!

#define TWI_ENABLE()   (TWCR |= (uint8_t)(1<<TWEN)
#define TWI_DISABLE()  (TWCR &= (uint8_t)~(1<<TWEN)

#define EXTERNAL                                0
#define INTERNAL                                1

//#define TW_PULLUPS INTERNAL
//#define TW_PULLUPS EXTERNAL       // if defined, (en/dis)ables the internal pull-ups on SDA and SCL pins; default is (no external pullups) internal pullups enabled

#define TW_SET_PULLUPS(EXTERNAL)          PORTC &= (uint8_t)~( (1<<4) | (1<<5) )
#define TW_SET_PULLUPS(INTERNAL)          PORTC |= (uint8_t)( (1<<4) | (1<<5) )

#define TW_PS_1                              0x00
#define TW_PS_4                              0x01
#define TW_PS_16                             0x02
#define TW_PS_32                             0x03

//--- Init the TWI/I2C ---
void tw_init(void);

//--- Set the two-wire clock bits --- 
void tw_set_br(uint16_t bit_rate_kHz);

void tw_init(void)
{
    TWI_ENABLE();
    //make sure the PUD is not set
    //enable internal pullups and set port registers    

#ifdef TWI_PULLUPS
    TW_SET_PULLUPS(TW_PULLUPS);
#else
    TW_SET_PULLUPS(INTERNAL);
#endif


}

void tw_set_br(uint16_t bit_rate_kHz)
{
    uint8_t br_div =  
}
