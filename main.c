#define F_CLK 8000000UL
#define F_CPU 8000000UL 
#define ADXL_ALT_ADDRESS_LOW
#define TW_PULLUPS INTERNAL_PULLUPS
#define TW_DATA_TRANSFER_MODE_FAST        
//#define VERBOSE
#define PRESSUSE_SENS
#define HR_SENS
#define TW_DELAY 10 
#define ADXL_FULL_RES  //define to get full resolution, else set to 10 bit resolution

//some additional ADC stuff

#define _ADMUX_REFS   1
#define _ADMUX_ADLAR  1
#define SET_ADMUX(x)       ADMUX |= (uint8_t)(x)
#define _ADCSRA_ADPS  0x06  //64 prescalar
#define ADC_STARTC()       ADCSRA |= 0xC0
#define ADC_CLEAR_FLAG()     ADCSRA &= ~(1<<ADIF)


#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>  
#include <avr/pgmspace.h>
#include <util/delay.h>


void uart_init(uint16_t brate)
{
    uint16_t ubrr = (F_CPU/8UL/brate)-1;
    UBRR0L = (ubrr& 0xFF); 
    UBRR0H = (ubrr>>8);

    UCSR0A = 0x02;
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


#include "twi-utils.h"
#include "adxl345-twi.h"

uint8_t hr_mon =0;

ISR(PCINT0_vect)
{
    hr_mon=1;
}

int main(void)
{
    cli(); 
    TCCR0A = 0x02; //clear timer on compare match A
    TCCR0B = 0x03; //64 prescaler
    OCR0A = 244;
    TIMSK0 = 0x02;
#ifdef PRESSURE_SENS
    ADMUX = 0x60;
    ADCSRA = 0x86;
    SET_ADMUX(2);
#endif
#ifdef HR_SENS
    PCICR = 0x01;
    PCMSK0 = 0x04;
    PORTB &= ~(0x04);
    DDRB &= ~(0x04);
#endif 
    uart_init(19200); 
    tw_init();
    adxl_init(ADXL_RANGE_16G);
    uart_print_hex(tw_read_byte(ADXL_SLA, ADXL_DEVID));
    uart_put('\n');

    sei();

    while(1)
    {
        int16_t measured[3];
#ifdef PRESSURE_SENS 
        uint8_t adc_meas;
        ADC_STARTC();
        while(!(ADCSRA|1<<ADIF)) ;
        ADC_CLEAR_FLAG();
        adc_meas = ADCH;
#endif
#ifdef HR_SENS
        uart_print_uint(hr_mon);
        hr_mon = 0;
        uart_put(' ');
#endif
#ifdef PRESSURE_SENS 
        uart_print_uint(adc_meas);
        uart_put(' ');
#endif
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
    while(!(TIFR0 & 1<<OCF0A)) ;
    TIFR0 |= 1<<OCF0A;
    }
}
