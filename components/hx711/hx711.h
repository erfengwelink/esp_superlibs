#ifndef _HX711_H_
#define _HX711_H_

#include <stdint.h>

typedef enum{
WEIGHT_0_KG = 0,
WEIGHT_5_KG = 5,
WEIGHT_11_KG = 11,
WEIGHT_17_KG = 17,
WEIGHT_27_KG = 27,
WEIGHT_37_KG = 37,
WEIGHT_47_KG = 47,
WEIGHT_57_KG = 57,
WEIGHT_67_KG = 67,
WEIGHT_77_KG = 77,
WEIGHT_87_KG = 87,
WEIGHT_OVERLOAD,
}Weight_t;

typedef struct _hx711
{
    //int gpioSck;
    //int gpioData;
    uint32_t pinSck;
    uint32_t pinData;
    unsigned int offset;
    unsigned int adc;
    int gain;
    // 1: channel A, gain factor 128
    // 2: channel B, gain factor 32
// 3: channel A, gain factor 64
} HX711;

void HX711_Init(HX711 *data);

HX711 *HX711_Tare(HX711 *data, uint8_t times);

Weight_t get_weight();

#endif