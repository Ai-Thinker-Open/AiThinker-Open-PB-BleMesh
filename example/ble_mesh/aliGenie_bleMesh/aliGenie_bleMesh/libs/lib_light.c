#include "lib_light.h"

#define UNPACK_LE_2_BYTE(dst,src)\
    *((unsigned short int *)(dst))  = *((src) + 1); \
    *((unsigned short int *)(dst))  = *((unsigned short int *)(dst)) << 8; \
    *((unsigned short int *)(dst)) |= *((src) + 0);


/*
    void RGB2HSL(TColor AColor, double &H,double &S,double &L)
    {
    double R,G,B,Max,Min,del_R,del_G,del_B,del_Max;
    R = GetRValue(AColor) / 255.0;       //Where RGB values = 0 ÷ 255
    G = GetGValue(AColor) / 255.0;
    B = GetBValue(AColor) / 255.0;

    Min = min(R, min(G, B));    //Min. value of RGB
    Max = max(R, max(G, B));    //Max. value of RGB
    del_Max = Max - Min;        //Delta RGB value

    L = (Max + Min) / 2.0;

    if (del_Max == 0)           //This is a gray, no chroma...
    {
        //H = 2.0/3.0;          //Windows下S值为0时，H值始终为160（2/3*240）
        H = 0;                  //HSL results = 0 ÷ 1
        S = 0;
    }
    else                        //Chromatic data...
    {
        if (L < 0.5) S = del_Max / (Max + Min);
        else         S = del_Max / (2 - Max - Min);

        del_R = (((Max - R) / 6.0) + (del_Max / 2.0)) / del_Max;
        del_G = (((Max - G) / 6.0) + (del_Max / 2.0)) / del_Max;
        del_B = (((Max - B) / 6.0) + (del_Max / 2.0)) / del_Max;

        if      (R == Max) H = del_B - del_G;
        else if (G == Max) H = (1.0 / 3.0) + del_R - del_B;
        else if (B == Max) H = (2.0 / 3.0) + del_G - del_R;

        if (H < 0)  H += 1;
        if (H > 1)  H -= 1;
    }
    }
*/
double Hue2RGB(double v1, double v2, double vH)
{
    if (vH < 0) vH += 1;

    if (vH > 1) vH -= 1;

    if (6.0 * vH < 1) return v1 + (v2 - v1) * 6.0 * vH;

    if (2.0 * vH < 1) return v2;

    if (3.0 * vH < 2) return v1 + (v2 - v1) * ((2.0 / 3.0) - vH) * 6.0;

    return (v1);
}

void HSL2RGB(unsigned short int h,unsigned short int s,unsigned short int l,COLOR_RGB_T* rgb)
{
    unsigned char R,G,B;
    double H,S,L;
    double var_1, var_2;
    H = h/65535.0;
    S = s/65535.0;
    L = l/65535.0;

    if (S == 0)                       //HSL values = 0 ÷ 1
    {
        R = L * 255.0;                   //RGB results = 0 ÷ 255
        G = L * 255.0;
        B = L * 255.0;
    }
    else
    {
        if (L < 0.5) var_2 = L * (1 + S);
        else         var_2 = (L + S) - (S * L);

        var_1 = 2.0 * L - var_2;
        R = 255.0 * Hue2RGB(var_1, var_2, H + (1.0 / 3.0));
        G = 255.0 * Hue2RGB(var_1, var_2, H);
        B = 255.0 * Hue2RGB(var_1, var_2, H - (1.0 / 3.0));
    }

    rgb->r = R;
    rgb->g = G;
    rgb->b = B;
    return ;
}

void calc_color_rgb(unsigned int data, COLOR_RGB_T* color_rgb)
{
    unsigned short int val[3];
    UNPACK_LE_2_BYTE(val,(unsigned char*)data);//lightness
    UNPACK_LE_2_BYTE(val+1,(unsigned char*)data+2);//hue
    UNPACK_LE_2_BYTE(val+2,(unsigned char*)data+4);//Saturation
    HSL2RGB(val[1],val[2],val[0],color_rgb);
}

