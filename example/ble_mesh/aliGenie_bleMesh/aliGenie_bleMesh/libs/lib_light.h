#ifndef _LIB_LIGHT_H_
#define _LIB_LIGHT_H_
typedef struct color_rgb
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} COLOR_RGB_T;

extern void calc_color_rgb(unsigned int data, COLOR_RGB_T* color_rgb);
#endif

