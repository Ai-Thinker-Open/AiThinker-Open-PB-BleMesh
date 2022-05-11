
#include "appl_utils.h"


/********************************************
    函数功能:时间戳转换成时间
*********************************************/
static uint8_t const Year_a[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
static uint8_t const Year_b[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
static uint16_t const Year_C[4]= {365,365,366,365};


TimePackge LocalTime(uint32_t Second)
{
    TimePackge Times;
    uint8_t i;
    Second += 28800;  //北京时间是8:00:00  60*60*8=28800秒
    Times.Second=Second%60;
    Times.Minute=Second%3600/60;
    Times.hour=Second/3600%24;
    Times.month=0;
    Times.day=0;
    Second =Second/86400;  //共有多少天,1天=86400秒
    Second++;              //每年是从第1天算起，所以加1天
    //printf("\n从:%d年1月1日到今有%d天", Times.year,Second);
    Times.year=Second/1461;//4年
    Times.year =Times.year*4+1970;//
    Second=Second%1461;

    //printf("\n还有多少:%d天",Second);
    if(Second>0)
    {
        i=0;

        while(Second>Year_C[i])
        {
            Second-=Year_C[i];
            Times.year++;
            // printf("\n日期:%d年%d天", Times.year,Second);
            i++;
        }

        i=0;

        if(Times.year % 4==0)   //润年
        {
            while(Second>Year_b[i])
            {
                Second -= Year_b[i];
                Times.month++;
                i=Times.month%12;
            }
        }
        else
        {
            while(Second>Year_a[i])
            {
                Second -= Year_a[i];
                Times.month++;
                i=Times.month%12;
            }
        }

        Times.day=Second;
        Times.month++;
    }
    else
    {
        Times.year--;
        Times.day=1;
        Times.month=1;
    }

    return Times;
    //printf("日期:%d-%d-%d %d:%d:%d", Times.year,Times.month,Times.day,Times.hour,Times.Minute,Times.Second);
}


