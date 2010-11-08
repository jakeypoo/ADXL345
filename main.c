#define F_CLK 1000000UL
#define F_CPU F_CLK
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
//#include "twi-utils.h"
//#include "adxl345-twi.h"

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

int main(void)
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

    while(1)
    {
        _delay_ms(20);
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
}
