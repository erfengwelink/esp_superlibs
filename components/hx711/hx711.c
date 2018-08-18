#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "hx711.h"

static void HX711_offset_set(HX711 *data, unsigned int offset)
{
    data->offset = offset;
}

void HX711_Init(HX711 *data)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;

    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL << data->pinSck;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;

    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL << data->pinData;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(data->pinSck, 1);
    ets_delay_us(60);
    gpio_set_level(data->pinSck, 0);
    ets_delay_us(60);

    HX711_Tare(data, 10);
    HX711_offset_set(data, data->adc);
}

static unsigned int HX711_Value(HX711 *data)
{
    unsigned int Count;
    unsigned char i;
    gpio_set_level(data->pinSck, 0); //使能AD（PIN_SCL 置低）
    Count = 0;
    while (gpio_get_level(data->pinData) == 1)
        ; //AD转换未结束则等待，否则开始读取
    for (i = 0; i < 24; i++)
    {
        gpio_set_level(data->pinSck, 1); //PIN_SCL 置高（发送脉冲）
        Count = Count << 1;              //下降沿来时变量Count左移一位，右侧补零
        gpio_set_level(data->pinSck, 0); //PIN_SCL 置低
        if (gpio_get_level(data->pinData))
            Count++;
    }
    gpio_set_level(data->pinSck, 1);

    Count = Count ^ 0x800000; //第25个脉冲下降沿来时，转换数据
    gpio_set_level(data->pinSck, 0);
    return (unsigned int)Count;
}

static unsigned int HX711_Average_Value(HX711 *data, uint8_t times)
{
    unsigned int sum = 0;
    for (int i = 0; i < times; i++)
    {
        sum += HX711_Value(data);
    }

    return sum / times;
}

HX711 *HX711_Tare(HX711 *data, uint8_t times)
{
    unsigned int sum = HX711_Average_Value(data, times);
    if (data->offset)
    {
        data->adc = abs(sum - data->offset);
    }
    else
    {
        data->adc = sum;
    }
    //printf("sum= %lu\n", sum);
    return data;
}

static Weight_t get_KG(int wght)
{
    Weight_t rng = WEIGHT_0_KG;
    if (wght > 4)
    {
        if (wght < 8)
        {
            rng = WEIGHT_5_KG;
        }
        else if (wght > 8 && wght < 14)
        {
            rng = WEIGHT_11_KG;
        }
        else if (wght > 14 && wght < 22)
        {
            rng = WEIGHT_17_KG;
        }
        else if (wght > 22 && wght < 32)
        {
            rng = WEIGHT_27_KG;
        }
        else if (wght > 32 && wght < 42)
        {
            rng = WEIGHT_37_KG;
        }
        else if (wght > 42 && wght < 52)
        {
            rng = WEIGHT_47_KG;
        }
        else if (wght > 52 && wght < 62)
        {
            rng = WEIGHT_57_KG;
        }
        else if (wght > 62 && wght < 72)
        {
            rng = WEIGHT_67_KG;
        }
        else if (wght > 72 && wght < 82)
        {
            rng = WEIGHT_77_KG;
        }
        else if (wght > 82)
        {
            rng = WEIGHT_87_KG;
        }
    }
    return rng;
}

Weight_t get_weight(HX711 *data)
{
    int w = 0;
    //HX711 data = {4, 5, 0, 0, 1};
    //HX711_Init(&data);
    //HX711_Tare(&data, 10);
    w = (int)300.0 * data->adc / (1024 * 1024 * 4 - 1);
    printf("realtime: %dkg\n", w);
    return get_KG(w);
}