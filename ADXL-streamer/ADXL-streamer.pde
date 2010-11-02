#include "adxl345-twi.h"

#define ADXL_ALT_ADDRESS_LOW
#define TW_PULLUPS_INTERNAL
#define TW_DATA_TRANSFER_MODE_FAST


void setup()
{
    tw_init();
    adxl_init(ADXL_RANGE_4G);
}
