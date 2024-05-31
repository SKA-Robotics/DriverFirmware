#include "ntc_driver.h"

// The following code was generated using: https://www.sebulli.com/ntc/
/**
 * The NTC table has 257 interpolation points.
 * Unit:0.1 °C
 *
 */
int NTC_table[257] = {
    3525, 2945, 2365, 2077, 1891, 1755, 1650, 1565, 1493, 1431, 1377, 1330,
    1287, 1248, 1213, 1181, 1151, 1123, 1097, 1072, 1050, 1028, 1007, 988,
    969,  952,  935,  919,  903,  888,  874,  860,  847,  834,  821,  809,
    797,  786,  775,  764,  754,  743,  733,  724,  714,  705,  696,  687,
    678,  670,  662,  653,  645,  638,  630,  622,  615,  608,  601,  593,
    587,  580,  573,  566,  560,  553,  547,  541,  535,  528,  522,  516,
    511,  505,  499,  493,  488,  482,  477,  471,  466,  461,  455,  450,
    445,  440,  435,  430,  425,  420,  415,  410,  405,  401,  396,  391,
    386,  382,  377,  373,  368,  364,  359,  355,  350,  346,  341,  337,
    333,  328,  324,  320,  316,  311,  307,  303,  299,  295,  291,  286,
    282,  278,  274,  270,  266,  262,  258,  254,  250,  246,  242,  238,
    234,  230,  226,  222,  218,  214,  211,  207,  203,  199,  195,  191,
    187,  183,  179,  176,  172,  168,  164,  160,  156,  152,  148,  144,
    141,  137,  133,  129,  125,  121,  117,  113,  109,  105,  101,  97,
    93,   90,   86,   82,   78,   73,   69,   65,   61,   57,   53,   49,
    45,   41,   37,   32,   28,   24,   20,   15,   11,   7,    2,    -2,
    -6,   -11,  -15,  -20,  -25,  -29,  -34,  -38,  -43,  -48,  -53,  -58,
    -63,  -68,  -73,  -78,  -83,  -88,  -93,  -99,  -104, -109, -115, -121,
    -126, -132, -138, -144, -150, -157, -163, -169, -176, -183, -190, -197,
    -204, -212, -219, -227, -235, -244, -252, -261, -270, -280, -290, -301,
    -311, -323, -335, -348, -362, -376, -392, -409, -428, -449, -472, -499,
    -531, -571, -624, -710, -796};

/**
 * \brief    Converts the ADC result into a temperature value.
 *
 *           P1 and p2 are the interpolating point just before and after the
 *           ADC value. The function interpolates between these two points
 *           The resulting code is very small and fast.
 *           Only one integer multiplication is used.
 *           The compiler can replace the division by a shift operation.
 *
 *           In the temperature range from 0°C to 100°C the error
 *           caused by the usage of a table is 0.141°C
 *
 * \param    adc_value  The converted ADC result
 * \return              The temperature in 0.1 °C
 *
 */
int NTC_ADC2Temperature(unsigned int adc_value) {

    int p1, p2;
    /* Estimate the interpolating point before and after the ADC value. */
    p1 = NTC_table[(adc_value >> 4)];
    p2 = NTC_table[(adc_value >> 4) + 1];

    /* Interpolate between both points. */
    return p1 - ((p1 - p2) * (adc_value & 0x000F)) / 16;
};