#include "types.h"

typedef struct              //ʱ���
{
    uint16_t year;          //0--��
    uint8_t month;          //1--��
    uint8_t day;            //2--��
    uint8_t hour;           //3--ʱ
    uint8_t Minute;         //4--��
    uint8_t Second;         //5--��
} TimePackge;

extern TimePackge LocalTime(uint32_t Second);
