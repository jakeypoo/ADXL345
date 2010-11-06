#define F_CPU 16000000UL
#define ADXL_ALT_ADDRESS_LOW
#define TW_PULLUPS INTERNAL
#define TW_DATA_TRANSFER_MODE_FAST    
//#define VERBOSE
#define TW_DELAY 10 
#define ADXL_FULL_RES  //define to get full resolution, else set to 10 bit resolution

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>  
#include <avr/pgmspace.h>
#include <util/delay.h>

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

#define EXTERNAL_PULLUPS                                0    //CHANGED#
#define INTERNAL_PULLUPS                                1

//#define TW_PULLUPS INTERNAL_PULLUPS
//#define TW_PULLUPS EXTERNAL_PULLUPS       // if defined, (en/dis)ables the internal pull-ups on SDA and SCL pins; default is (no external pullups) internal pullups enabled

//#define TW_DATA_TRANSFER_MODE_FAST      //400kHz transfer speed
//#define TW_DATA_TRANSFER_MODE_STANDARD  //100kHz tranfer speed

#define TW_SET_PULLUPS(EXTERNAL_PULLUPS)     PORTC &= (uint8_t)~( (1<<4) | (1<<5) )
#define TW_SET_PULLUPS(INTERNAL_PULLUPS)     PORTC |= (uint8_t)( (1<<4) | (1<<5) )

#define TW_SET_PS(x)                      TWSR   = (uint8_t)(x)

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

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

void uart_init(uint16_t brate)
{
//        UBRR0L = F_CPU / 16 / brate - 1;
//        UBRR0H = ((F_CPU / 16 / brate - 1)>>8);
        // enable receive and transmit
//        UCSR0B = _BV(RXEN0) | _BV(TXEN0);
        // asyncronous, no parity, 1 stop bit, 8 bit character size
//        UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);

    uint16_t ubrr = (F_CPU/(16UL*brate))-1;
    UBRR0L = (ubrr& 0xFF); 
    UBRR0H = (ubrr>>8);

//    UCSR0A = 0x00;
    UCSR0C = 0x06;
    UCSR0B = (uint8_t)((1<<TXEN0));
}

static void uart_put(const char data_char)
{
//                while ( (UCSR0A & (1<<UDRE0))==0 );
//                UDR0 = data_char;

    while( !(UCSR0A & (1<<UDRE0)) ) ;
    UDR0 = data_char;
}

void uart_print_hex(uint32_t data_uint)
{
    int8_t ms_nibble;
    uint8_t nibble;
    uart_put('0');
    uart_put('x');
    for(ms_nibble = 28 ; ms_nibble >= 0; ms_nibble -=4)
    {
        if( (data_uint>>ms_nibble) & 0x0F) break; 
    }
    for( ; ms_nibble >= 0; ms_nibble -=4)
    {
        nibble = (uint8_t)( (data_uint>>ms_nibble) & 0x0F);
        if(nibble>0x09) nibble += 0x37;
        else nibble += 0x30;
        uart_put(nibble); 
    }
}

void uart_print_uint(uint32_t data_uint)
{
    uint32_t ms_digit;
    uint8_t digit;
    if(data_uint==0) uart_put('0');
    else
    {
        for(ms_digit = 1000000000; ms_digit>=1; ms_digit /= 10)
        {
            if(data_uint >= ms_digit) break;
        }
        for( ; ms_digit>=1; ms_digit /= 10)
        {
            digit = (uint8_t)(data_uint/ms_digit);
            data_uint = data_uint % ms_digit;
            uart_put(digit + 0x30);  
        }
    }   
}

void uart_print_int(int32_t data_int)
{
    if(data_int < 0)
    {
        uart_put('-');
        data_int *= (-1);
    }
    uart_print_uint((uint32_t)(data_int));
}

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------


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
    if( (int32_t)(F_CPU/16000)/bitrate_kHz - 16 > 255 ) 
    { 
        TW_SET_PS(TW_PS_64); 
        br_div = ((F_CPU/(bitrate_kHz*1000))-16)/128;
    } 
    else if( (int32_t)(F_CPU/4000)/bitrate_kHz - 16 > 255 )
    {
         TW_SET_PS(TW_PS_16);
         br_div = ((F_CPU/(bitrate_kHz*1000))-16)/32;
    }
    else if( (int32_t)(F_CPU/1000)/bitrate_kHz - 16 > 255 )
    {
         TW_SET_PS(TW_PS_4);
         br_div = ((F_CPU/(bitrate_kHz*1000))-16)/8;
    }
    else  br_div = ((F_CPU/(bitrate_kHz*1000))-16)/2;

    TWBR = br_div;  //set the bit rate divisor
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t tw_get_status()
{ 
    while( !(TWCR & (1<<TWINT)) ) ;
    return (TWSR & 0xF8);
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t tw_read_byte(uint8_t slave_address, uint8_t register_address)
{    
#ifdef VERBOSE
    uart_put('r'); uart_put('b'); uart_put('\n');
#endif
    uint8_t data_in;  
    while(1){
#ifdef TW_DELAY
   _delay_us(TW_DELAY);
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
#ifdef VERBOSE
    uart_put('r'); uart_put('B'); uart_put('\n');
#endif
    while(1){
#ifdef TW_DELAY
   _delay_us(TW_DELAY);
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
    for( uint8_t i = 0; i < (n-1) ; i++)
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
#ifdef VERBOSE
    uart_put('w'); uart_put('b'); uart_put('\n');
#endif
    while(1){
#ifdef TW_DELAY
   _delay_us(TW_DELAY);
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
#ifdef VERBOSE 
    uart_put('w'); uart_put('B'); uart_put('\n'); 
#endif
    while(1){
#ifdef TW_DELAY
   _delay_us(TW_DELAY);
#endif
    TW_SEND_START();                //grab the line
    if(tw_get_status() != TW_START) {TW_SEND_STOP(); continue;}
    TW_SEND_SLAW(slave_address);    //send a SLA+W 
    if(tw_get_status() != TW_MT_SLA_ACK) {TW_SEND_STOP(); continue;}
    TW_SEND_BYTE(register_address); //then register address
    if(tw_get_status() != TW_MT_DATA_ACK) {TW_SEND_STOP(); continue;}
    for(uint8_t i = 0; i < (n-1) ; i++)
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

//Alternate address select, default is SDO/ALT ADDRESS pin grounded
 
//#define ADXL_ALT_ADDRESS_HIGH   //uncomment if SDO pin is tied high
//#define ADXL_ALT_ADDRESS_LOW    //uncomment if SDO pin is tied to ground

#ifdef ADXL_ALT_ADDRESS_HIGH
#define ADXL_SLA                0x1D
#else
#define ADXL_SLA                0x53
#endif
#if !defined(ADXL_ALT_ADDRESS_HIGH) && !defined(ADXL_ALT_ADDRESS_HIGH)
#define ADXL_ALT_ADDRESS_LOW 
#endif

/* **************** ADXL345 REGISTER MAP ****************** */
#define ADXL_DEVID            0x00     //Device ID  (0b11100101)
// 0x10 to 0x1C are reserved
#define ADXL_THRESH_TAP       0x1D     //Tap threshold (unsigned; 0x01 = 62.5mg)
#define ADXL_OFSX             0x1E     //X-axis offset (2's comp; 0x01 = 15.6mg)
#define ADXL_OFSY             0x1F     //Y-axis offset (2's comp; 0x01 = 15.6mg)
#define ADXL_OFSZ             0x20     //Z-axis offset (2's comp; 0x01 = 15.6mg)
#define ADXL_DUR              0x21     //Tap duration  (unsigned; 0x01 = 625 us) 
#define ADXL_LATENT           0x22     //Tap latency   (unsigned; 0x01 = 1.25ms)
#define ADXL_WINDOW           0x23     //Tap window    (unsigned; 0x01 = 1.25ms)
#define ADXL_THRESH_ACT       0x24     //Activity threshold    (unsigned; 0x01 = 62.5mg) 
#define ADXL_THRESH_INACT     0x25     //Inactivity threshold  (unsigned; 0x01 = 62.5mg)
#define ADXL_TIME_INACT       0x26     //Inactivity time       (unsigned; 0x01 = 1 sec)
#define ADXL_ACT_INACT_CTL    0x27     //Axis enable control for activity/inactivity detection
#define ADXL_THRESH_FF        0x28     //Free-fall threshold   (unsigned; 0x01 = 62.5mg) 
#define ADXL_TIME_FF          0x29     //Freefall time         (unsigned; 0x01 = 5ms)
#define ADXL_TAP_AXES         0x2A     //Axes control for tap/double tap
#define ADXL_ACT_TAP_STATUS   0x2B     //Source of tap/double tap
#define ADXL_BW_RATE          0x2C     //Data rate and power mode control
#define ADXL_POWER_CTL        0x2D     //Power saving features control
#define ADXL_INT_ENABLE       0x2E     //Inturrupt enable control
#define ADXL_INT_MAP          0x2F     //Inturrupt mapping control
#define ADXL_INT_SOURCE       0x30     //Source of interrupts
#define ADXL_DATA_FORMAT      0x31     //Data format control
#define ADXL_DATAX0           0x32     //X-Axis Data 0
#define ADXL_DATAX1           0x33     //X-Axis Data 1
#define ADXL_DATAY0           0x34     //Y-Axis Data 0
#define ADXL_DATAY1           0x35     //Y-Axis Data 1
#define ADXL_DATAZ0           0x36     //Z-Axis Data 0
#define ADXL_DATAZ1           0x37     //Z-Axis Data 1
#define ADXL_FIFO_CTL         0x38     //FIFO control 
#define ADXL_FIFO_STATUS      0x39     //FIFO status  

#define ADXL_RATE_CODE_I2C_FAST    0x0C

#define ADXL_RANGE_2G              0x00
#define ADXL_RANGE_4G              0x01
#define ADXL_RANGE_8G              0x02
#define ADXL_RANGE_16G             0x03
#define ADXL_RES_BIT               0x08

#define ADXL_X_AXIS                ADXL_DATAX0
#define ADXL_Y_AXIS                ADXL_DATAY0
#define ADXL_Z_AXIS                ADXL_DATAZ0


// --- set/clear the measure bit in the power control register to turn on/sleep the adxl ---
#define ADXL_ENABLE()             tw_write_byte((uint8_t)(1<<3), ADXL_SLA, ADXL_POWER_CTL)
#define ADXL_SLEEP()              tw_write_byte( 0x00, ADXL_SLA, ADXL_POWER_CTL)

#define ADXL_RANGE_SEL(x)         tw_write_byte( x, ADXL_SLA, ADXL_DATA_FORMAT)

// ---  interrupt set/mask defines  ----
#define ADXL_INT_DATA_READY        0x80
#define ADXL_INT_SINGLE_TAP        0x40
#define ADXL_INT_DOUBLE_TAP        0x20
#define ADXL_INT_ACTIVITY          0x10
#define ADXL_INT_INACTIVITY        0x08
#define ADXL_INT_FREE_FALL         0x04
#define ADXL_INT_WATERMARK         0x02
#define ADXL_INT_OVERRUN           0x01
                            

//--- sets the bit rate registers if necessary, enables the measure bit to turn on the adxl ---
void adxl_init(uint8_t range_select);
//--- enable interrupts and map to int1(leave map mask low) or int2(set map mask bit) ---
//---- usage:  adxl_set_interrupts((ADXL_INT_DATA_READY | ADXL_INT_FREE_FALL), ADXL_INT_FREE_FALL);
//       >> enables data_ready interrupt on pin INT1, enables free_fall interrupt on pin INT2 
void adxl_set_intertupts(uint8_t int_enable, uint8_t int_pin_mask);
//--- pulls data from one axis only ---
int16_t adxl_measure_axis(uint8_t axis_select);
//--- pulls data from x, y, z registers, puts it in array data_out, returns 1 if successful ---
uint8_t adxl_measure_xyz(int16_t *data_out);

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
void adxl_init(uint8_t range_select)
{
#ifdef TW_DATA_TRANSFER_MODE_FAST
    TWI_DISABLE();
    tw_set_br(100);
    TWI_ENABLE();
    tw_write_byte(ADXL_RATE_CODE_I2C_FAST, ADXL_SLA, ADXL_BW_RATE);
    TWI_DISABLE();
    tw_set_br(400);
    TWI_ENABLE();
#endif
#ifndef ADXL_FULL_RES
    ADXL_RANGE_SEL(range_select);
#endif
#ifdef ADXL_FULL_RES
    ADXL_RANGE_SEL((range_select|ADXL_RES_BIT));
#endif
    ADXL_ENABLE();    
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------

int16_t adxl_measure_axis(uint8_t axis_select)
{
    uint8_t data_in[2]; 
    while( !tw_read_block(&data_in[0], 2, ADXL_SLA, axis_select) ) ;
    return (int16_t)((data_in[1] << 8) | data_in[0]);
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
uint8_t adxl_measure_xyz(int16_t *data_out)
{  
#ifdef VERBOSE 
    uart_put('x'); uart_put('y'); uart_put('z'); uart_put('\n');
#endif
    uint8_t data_in[6]; 
    while( !tw_read_block(&data_in[0], 6, ADXL_SLA, ADXL_DATAX0) ) ;
    *data_out = (int16_t)( (data_in[1] << 8) | data_in[0] );
    *(data_out + 1) = (int16_t)( (data_in[3] << 8) | data_in[2] );  //make sure this casts 2's compliments properly
    *(data_out + 2) = (int16_t)( (data_in[5] << 8) | data_in[4] );
    return 1;
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
void adxl_set_intertupts(uint8_t int_enable, uint8_t int_pin_mask)
{
    ADXL_SLEEP(); //make sure to assign threshold values to tap, double-tap, etc.
    tw_write_byte(int_enable, ADXL_SLA, ADXL_INT_ENABLE);
    tw_write_byte(int_pin_mask, ADXL_SLA, ADXL_INT_MAP); 
    ADXL_ENABLE();
}

//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
void setup(void)
{
    uart_init(9600); 
    uart_put(':');
    uart_put(')');
    uart_put('\n');
    uart_print_int(100);
    uart_put('\n');
    uart_print_int(-100);
    uart_put('\n');
    tw_init();
    adxl_init(ADXL_RANGE_16G);
    uart_print_hex(tw_read_byte(ADXL_SLA, ADXL_DEVID));
    uart_put('\n');
}

void loop(void)
{   _delay_ms(20);
    int16_t measured[3];
    if(adxl_measure_xyz(&measured[0]));
    {
        uart_put('x');
        uart_put(' ');
        uart_print_int(measured[0]);
        uart_put(' ');
        uart_put('y');
        uart_put(' ');
        uart_print_int(measured[1]);
        uart_put(' ');
        uart_put('z');
        uart_put(' ');
        uart_print_int(measured[2]);
        uart_put('\n');
    }
}

