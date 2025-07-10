#include "Inf_OLED.h"

uint8_t OLED_GRAM[128][8];

void OLED_WriteCmd(uint8_t cmd){
    // debug_printfln("OLED_WriteCmd:%x Started", cmd);
    uint8_t i;
    /* OLED_DC拉高，传输命令 */
    OLED_DC_LOW();
    /* OLED_CS拉低，片选，拉低有效 */
    OLED_CS_LOW();

    for (i = 0; i < 8; i++)
    {
        OLED_SCLK_LOW();
        if (cmd & 0x80){
            OLED_SDA_HIGH();
        }else{
            OLED_SDA_LOW();
        }
        OLED_SCLK_HIGH();
        cmd <<= 1;
    }

    OLED_CS_HIGH();
    OLED_DC_HIGH();
    // debug_printfln("OLED_WriteCmd:%x Ended", cmd);
}


void OLED_WriteData(uint8_t data){
    // debug_printfln("OLED_WriteData:%d Started", data);
    uint8_t i;
    /* OLED_DC拉i低，传输数据 */
    OLED_DC_HIGH();
    /* OLED_CS拉低，片选，拉低有效 */
    OLED_CS_LOW();

    for (i = 0; i < 8; i++)
    {
        OLED_SCLK_LOW();
        if (data & 0x80){
            OLED_SDA_HIGH();
        }else{
            OLED_SDA_LOW();
        }
        OLED_SCLK_HIGH();
        data <<= 1;
    }

    OLED_CS_HIGH();
    OLED_DC_HIGH();
    // debug_printfln("OLED_WriteData Ended");
}

void OLED_Refresh_Gram(void){
    uint8_t i, j;
    for (i = 0; i < 8; i++)
    {
        OLED_WriteCmd(0xb0 + i);
        OLED_WriteCmd(0x00);
        OLED_WriteCmd(0x10);
        for (j = 0; j < 128; j++)
        {
            OLED_WriteData(OLED_GRAM[j][i]);
        }
    }
}

/* 清屏函数 */
void OLED_Clear(void){
    uint8_t i, j;
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 128; j++)
        {
            OLED_GRAM[j][i] = 0x00;
        }
    }
    OLED_Refresh_Gram();
}


/* 初始化OLED */
void OLED_Init(void){
    OLED_RST_LOW();
    HAL_Delay(100);
    OLED_RST_HIGH();

    OLED_WriteCmd(0xAE);    //关闭显示
    OLED_WriteCmd(0x00);    //设置低列地址
    OLED_WriteCmd(0x10);    //设置高列地址
    OLED_WriteCmd(0x40);    //设置起始行地址
    OLED_WriteCmd(0x81);    //对比度设置
    OLED_WriteCmd(0xCF);    //对比度设置
    OLED_WriteCmd(0xA1);    //设置段重定义
    OLED_WriteCmd(0xC0);    //设置COM输出扫描方向
    OLED_WriteCmd(0xA6);    //正常显示
    OLED_WriteCmd(0xA8);    //设置多路复用率
    OLED_WriteCmd(0x3f);    //1/64
    OLED_WriteCmd(0xD3);    //设置显示偏移
    OLED_WriteCmd(0x00);   
    OLED_WriteCmd(0xd5);    //设置显示时钟分频因子/振荡器频率
    OLED_WriteCmd(0x80);    //设置时钟分频因子/振荡器频率
    OLED_WriteCmd(0xD9);    //设置预充电周期
    OLED_WriteCmd(0xF1);    //设置预充电周期
    OLED_WriteCmd(0xDA);    //设置COM引脚配置
    OLED_WriteCmd(0x12);    //设置COM引脚配置
    OLED_WriteCmd(0xDB);    //设置VCOMH
    OLED_WriteCmd(0x30);    //设置VCOMH取消选择级别
    OLED_WriteCmd(0x20);    //设置内存地址模式
    OLED_WriteCmd(0x02); 
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14);
    OLED_Clear();    
    OLED_WriteCmd(0xAF);    
}


/* 开启显示 */
void OLED_Display_On(void){
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF);    //开启显示
}


/* 关闭显示 */
void OLED_Display_Off(void){
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x10);
    OLED_WriteCmd(0xAE);    //关闭显示
}

/* 画点 */
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t){
    uint8_t pos, bx, temp = 0;
    if (x > 127 || y > 63)
        return;
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

/* 在指定位置显示一个字符 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';
    for (t = 0; t < size; t++)
    {
        if (size == 12)
            temp = oled_asc2_1206[chr][t];
        else
            temp = oled_asc2_1608[chr][t];
        
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80){
                OLED_DrawPoint(x, y, mode);
            }else{
                OLED_DrawPoint(x, y, !mode);
            }
            temp <<= 1;
            y++;
            if ((y - y0) == size){
                y = y0;
                x++;
                break;
            }
        }
    }
}

uint32_t oled_pow(uint8_t m, uint8_t n){
    uint32_t result = 1;
    while (n--)
        result *= m;
    return result;
}


/* 显示2个数字 */
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size / 2) * t, y, ' ', size, 1);
                continue;
            }
            else
            {
                enshow = 1;
            }            
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 1);
    }
}


/* 显示字符串 */
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *str, uint8_t size, uint8_t mode)
{
    while (*str != '\0')
    {
        if (x > 122)
        {
            x = 0;
            y += 16;
        }
        if (y > 58)
        {
            y = x = 0;
        }
        OLED_ShowChar(x, y, *str, size, mode);
        x += 8;
        str++;
    }
}

/* 显示汉字 */
void OLED_ShowCH(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    for (t = 0; t < size; t++)
    {
        temp = oled_CH_1616[chr][t];
        for (t1 = 0; t1 < 8; t1++){
            if (temp & 0x80){
                OLED_DrawPoint(x, y, mode);
            }else{
                OLED_DrawPoint(x, y, !mode);
            }
            temp <<= 1;
            y++;
            if ((y - y0) == size){
                y = y0;
                x++;
                break;
            }
        }
    }
}


/* 显示单个中文字符 */
void OLED_Show_CH(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
    OLED_ShowCH(x, y, chr*2, size, 1);
    OLED_ShowCH(x + size/2, y, chr*2+1, size, 1);
}

/* 显示中文字符串 */
void OLED_Show_CH_String(uint8_t x, uint8_t y, const uint8_t *str, uint8_t size, uint8_t mode)
{
    uint8_t temp, t, t1;
    uint8_t y0 = y;

    for (t = 0; t < size; t++)
    {
        temp = str[t];
        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp & 0x80){
                OLED_DrawPoint(x, y, mode);
            }else{
                OLED_DrawPoint(x, y, !mode);
            }
            temp <<= 1;
            y++;
            if ((y - y0) == size){
                y = y0;
                x++;
                break;
            }
        }
    }
}

