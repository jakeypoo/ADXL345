/*---------------- AVR TWI (I2C) -----------------*/
/* j.telatnik                                2010 */
/* ---------------------------------------------- */
/* this library was written to take care of some  */
/* common two-wire functions and setup, it was    */
/* written to work with the ATMEGA48/88/128/328   */
/* family of devices, although it should work with*/
/* most devices.                                  */
/*                                                */
/*   NOTES:                                       */
/* -F_CLK must be defined in order to set the     */
/*    timer properly                              */
/* -TW_PULLUPS should be defined as INTERNAL or   */
/*     EXTERNAL in order to enable or disable     */
/*     internal interrupts, default is internal   */
/* -TW_DATA_TRANSFER_MODE_FAST should be defined  */
/*     to enable 400kHz bit rate, else TW_DATA_T- */
/*     RANSFER_MODE_STANDARD should be defined to */
/*     enable 100kHz bit rate, default is         */
/*     standard                                   */
/* -TW_DELAY is roughly the delay in us between a */
/*     master STOP transmission and the next      */
/*     START transmission (but not for a repeated */
/*     START). I found that some devices are not  */
/*     immediately addressable after a STOP and   */
/*     will NACK a SLA+W if there is not at least */
/*     a 10us delay between TX of a STOP - START  */
/*                                                */
/*   TO DOS:                                      */
/* -Currently does not work with TWI interrupt    */
/*     handling, needs to be implemented by       */
/*     altering the TWCR assignment to not alter  */
/*     the TWIE bit (ie. change the ='s to |='s   */
/*     and &='s)                                  */
/* ---------------------------------------------- */

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#ifndef TW_DELAY
#define TW_DELAY 10
#endif

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


#define TWI_ENABLE()   (TWCR |= (uint8_t)(1<<TWEN))
#define TWI_DISABLE()  (TWCR &= (uint8_t)~(1<<TWEN))

#define EXTERNAL_PULLUPS                                0
#define INTERNAL_PULLUPS                                ((uint8_t)(1<<4)|(1<<5))

//#define TW_PULLUPS INTERNAL_PULLUPS
//#define TW_PULLUPS EXTERNAL_PULLUPS       // if defined, (en/dis)ables the internal pull-ups on SDA and SCL pins; default is (no external pullups) internal pullups enabled

//#define TW_DATA_TRANSFER_MODE_FAST      //400kHz transfer speed
//#define TW_DATA_TRANSFER_MODE_STANDARD  //100kHz tranfer speed

#define TW_SET_PULLUPS(x)          PORTC =(PORTC & (uint8_t)~((1<<4)|(1<<5)))|(x)

#define TW_SET_PS(x)               TWSR = (uint8_t)(x)

#define TW_PS_1                              0x00
#define TW_PS_4                              0x01
#define TW_PS_16                             0x02
#define TW_PS_64                             0x03


#define TW_SEND_START()                   TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTA)
#define TW_SEND_STOP()                    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)
#define TW_SEND_STOP_START()              TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)|(1<<TWSTA)
#define TW_CLR_INT()                      TWCR = (1<<TWINT)|(1<<TWEN)
#define TW_SEND_SLAR(x)                   TWDR = (uint8_t)(x<<1) | (0x01); \
                                          TW_CLR_INT()
#define TW_SEND_SLAW(x)                   TWDR = (uint8_t)(x<<1) & 0xFE; \
                                          TW_CLR_INT()
#define TW_SEND_BYTE(x)                   TWDR = x; \
                                          TW_CLR_INT()
#define TW_REC_ACK()                      TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA)
#define TW_REC_NACK()                     TWCR = (1<<TWINT)|(1<<TWEN)


//--- Init the TWI/I2C ---
void tw_init(void);

//--- Set the two-wire clock bits --- 
void tw_set_br(int16_t bitrate_kHz);

//--- wait for TWINT to be set, return the status register ---
uint8_t tw_get_status();

//--- Returns a byte from slave:register ; returns 0 if there is an error ---
uint8_t tw_read_byte(uint8_t slave_address, uint8_t register_address);

//--- Returns n bytes from slave:[register:register+n] into array data; return 1 if successful---
uint8_t tw_read_block(uint8_t *data, uint8_t n, uint8_t slave_address, uint8_t register_address);

//--- writes data to slave:register, returns 1 if successful ---
uint8_t tw_write_byte(uint8_t data, uint8_t slave_address, uint8_t register_address);

//--- writes n bytes to slave:[register:register+n] from array data, returns 1 if successful ---
uint8_t tw_write_block(uint8_t *data, uint8_t n, uint8_t slave_address, uint8_t register_address);

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
void tw_init(void)
{
    //only supports one transfer speed, either 400kHz or 100kHz
#ifdef TW_DATA_TRANSFER_MODE_FAST
    tw_set_br(400);
#else
    tw_set_br(100);
#endif
#if !defined(TW_DATA_TRANSFER_MODE_STANDARD) && !defined(TW_DATA_TRANSFER_MODE_FAST)
#define TW_DATA_TRANSFER_MODE_STANDARD
#endif
    MCUCR &= (uint8_t)~(1<<PUD); //make sure global pull up disable is not set
#ifdef TW_PULLUPS
    TW_SET_PULLUPS(TW_PULLUPS);
#else
    TW_SET_PULLUPS(INTERNAL_PULLUPS);
#endif

    //remember to select data transfer speed on the device 
    TWI_ENABLE();

}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
void tw_set_br(int16_t bitrate_kHz)
{
    uint8_t br_div; 
    
    TW_SET_PS(TW_PS_1); //clears the prescaler

    //select the prescaler, calculate the bit rate divisor
    if( (int16_t)(F_CLK/16000)/bitrate_kHz - 16 > 255 ) 
    { 
        TW_SET_PS(TW_PS_64); 
        br_div = (uint8_t)((F_CLK/bitrate_kHz/1000)-16)/128;
  } 
    else if( (int16_t)(F_CLK/4000)/bitrate_kHz - 16 > 255 )
    {
         TW_SET_PS(TW_PS_16);
         br_div = (uint8_t)((F_CLK/bitrate_kHz/1000)-16)/32;
   }
    else if( (int16_t)(F_CLK/1000)/bitrate_kHz - 16 > 255 )
    {
         TW_SET_PS(TW_PS_4);
         br_div = (uint8_t)((F_CLK/bitrate_kHz/1000)-16)/8;
    }
    else  br_div = (uint8_t)((F_CLK/bitrate_kHz/1000)-16)/2;
#ifdef VERBOSE
    uart_print_int(br_div);
    uart_put('\n');
#endif   
    TWBR = br_div;  //set the bit rate divisor
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t tw_get_status()
{
    while( !(TWCR & (1<<TWINT)) ) ;
#ifdef VERBOSE
    uart_print_hex(TWSR & 0xF8);
    uart_put('\n');
#endif 
    return (TWSR & 0xF8);
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t tw_read_byte(uint8_t slave_address, uint8_t register_address)
{
    uint8_t data_in;  
    while(1){
#ifdef TW_DELAY
       _delay_us(TW_DELAY);
#endif
#ifdef VERBOSE
    uart_put('r');uart_put('b');
    uart_put('\n');
#endif 
        TW_SEND_START();                //grab the line
        if(tw_get_status() != TW_START) {TW_SEND_STOP(); continue;}
        TW_SEND_SLAW(slave_address);    //send a SLA+W 
        if(tw_get_status() != TW_MT_SLA_ACK) {TW_SEND_STOP(); continue;}
        TW_SEND_BYTE(register_address); //then register address
        if(tw_get_status() != TW_MT_DATA_ACK) {TW_SEND_STOP(); continue;}
        TW_SEND_START();                //repeated start
        if(tw_get_status() != TW_REP_START) {TW_SEND_STOP(); continue;}
        TW_SEND_SLAR(slave_address);    //send a SLA+R
        if(tw_get_status() != TW_MR_SLA_ACK) {TW_SEND_STOP(); continue;}
        TW_REC_NACK();                  //let the data come in, send a NACK
        if(tw_get_status() != TW_MR_DATA_NACK) {TW_SEND_STOP(); continue;}
        data_in = TWDR;                 //read the data register
        TW_SEND_STOP();
        return data_in;  
    }
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t tw_read_block(uint8_t *data, uint8_t n, uint8_t slave_address, uint8_t register_address)
{
    uint8_t i;
    while(1){
#ifdef TW_DELAY
        _delay_us(TW_DELAY);
#endif
#ifdef VERBOSE
    uart_put('r');uart_put('B');
    uart_put('\n');
#endif 
        TW_SEND_START();                //grab the line
        if(tw_get_status() != TW_START) {TW_SEND_STOP(); continue;}
        TW_SEND_SLAW(slave_address);    //send a SLA+W 
        if(tw_get_status() != TW_MT_SLA_ACK) {TW_SEND_STOP(); continue;}
        TW_SEND_BYTE(register_address); //then register address
        if(tw_get_status() != TW_MT_DATA_ACK) {TW_SEND_STOP(); continue;}
        TW_SEND_START();                //repeated start
        if(tw_get_status() != TW_REP_START) {TW_SEND_STOP(); continue;} 
        TW_SEND_SLAR(slave_address);    //send a SLA+R
        if(tw_get_status() != TW_MR_SLA_ACK) {TW_SEND_STOP(); continue;}
        for(i = 0; i < (n-1) ; i++)
        {
            TW_REC_ACK();               //let n-1 data bytes come in, respond with ACKs to keep them flowing
            if(tw_get_status() != TW_MR_DATA_ACK) {TW_SEND_STOP(); return 0;}
            *(data+i) = TWDR;           //array must be at least n bytes in size
        }
        TW_REC_NACK();                  //let last data byte come in, send a NACK
        if(tw_get_status() != TW_MR_DATA_NACK) {TW_SEND_STOP(); continue;}
        *(data+(n-1)) = TWDR;           
        TW_SEND_STOP();
        return 1;
    }
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t tw_write_byte(uint8_t data, uint8_t slave_address, uint8_t register_address)
{  
    while(1){
#ifdef TW_DELAY
       _delay_us(TW_DELAY);
#endif
#ifdef VERBOSE
    uart_put('w');uart_put('b');
    uart_put('\n');
#endif 
        TW_SEND_START();                //grab the line
        if(tw_get_status() != TW_START) {TW_SEND_STOP(); continue;}
        TW_SEND_SLAW(slave_address);    //send a SLA+W 
        if(tw_get_status() != TW_MT_SLA_ACK) {TW_SEND_STOP(); continue;}
        TW_SEND_BYTE(register_address); //then register address
        if(tw_get_status() != TW_MT_DATA_ACK) {TW_SEND_STOP(); continue;}
        TW_SEND_BYTE(data);             //send byte 
        if(tw_get_status() != TW_MT_DATA_ACK)  {TW_SEND_STOP(); continue;} 
        TW_SEND_STOP();                 //got the awknowledge, drop the line
        return 1;
    }
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t tw_write_block(uint8_t *data, uint8_t n, uint8_t slave_address, uint8_t register_address)
{
    uint8_t i;
    while(1){
#ifdef TW_DELAY
       _delay_us(TW_DELAY);
#endif
#ifdef VERBOSE
    uart_put('w');uart_put('B');
    uart_put('\n');
#endif 
        TW_SEND_START();                //grab the line
        if(tw_get_status() != TW_START) {TW_SEND_STOP(); continue;}
        TW_SEND_SLAW(slave_address);    //send a SLA+W 
        if(tw_get_status() != TW_MT_SLA_ACK) {TW_SEND_STOP(); continue;}
        TW_SEND_BYTE(register_address); //then register address
        if(tw_get_status() != TW_MT_DATA_ACK) {TW_SEND_STOP(); continue;}
        for(i = 0; i < (n-1) ; i++)
        {  
            TW_SEND_BYTE( *(data+i) );  //array containing data must be at least n bytes in size
            if(tw_get_status() != TW_MT_DATA_ACK)  {TW_SEND_STOP(); return 0;}   
        }
        TW_SEND_BYTE( *(data+(n-1)) );  //send the last byte in the array              
        if(tw_get_status() != TW_MT_DATA_ACK)  {TW_SEND_STOP(); continue;}   
        TW_SEND_STOP();
        return 1;
    } 
}
