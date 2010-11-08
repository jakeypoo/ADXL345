/*---------------- ADXL345 (TWI/I2C) -------------*/
/* j.telatnik                                2010 */
/* ---------------------------------------------- */
/* this library takes care of basic functions for */
/* communicating with the ADXL345 accelerometer   */
/* using the 2-wire interface (I2C). Only basic   */
/* streaming functions are supported, there is no */
/* FIFO functionality or tap/free-fall/activity   */
/* detection included. Code was tested on ATMEGA48*/
/* /88/128/328 devices only.                      */
/*                                                */
/*   NOTES:                                       */
/* -F_CLK must be defined in order to set the     */
/*    timer properly                              */
/* -see twi-utils.h for definitions required in   */
/*    order to initialize the 2-wire interface    */
/* -ADXL_ALT_ADDRESS_HIGH should be defined if the*/
/*    SDO pin is tied to Vcc, ADXL_ALT_ADDRESS_LOW*/
/*    should be used if tied to ground            */
/* -ADXL_FULL_RES should be defined to instruct   */
/*    the ADXL to use full resolution, resulting  */
/*    in values that are **4mg/LSB. If not defined*/
/*    the output is always 10-bit resolution for  */
/*    all ranges, and the resolution of the LSB   */
/*    should be calculated from the ADXL datasheet*/
/* **1mg = 9.81mm/sec^2 = 0.0322 ft/s^2           */
/*                                                */
/*  TO DOs:                                       */
/* -Advancecd fuctionality like interrupts, FIFO  */
/*    controls, tap/free-fall/(in)activity detec- */
/*    tion would be useful, as well as changing   */
/*    resolution during run time might be usefull,*/
/*    but I won't be adding it unless I need to   */
/* ---------------------------------------------- */


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
