#include "types.h"

typedef struct              //时间包
{
    uint16_t year;          //0--年
    uint8_t month;          //1--月
    uint8_t day;            //2--日
    uint8_t hour;           //3--时
    uint8_t Minute;         //4--分
    uint8_t Second;         //5--分
} TimePackge;

extern TimePackge LocalTime(uint32_t Second);
