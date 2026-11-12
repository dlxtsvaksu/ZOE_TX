#ifndef __OLED_H__
#define __OLED_H__

#include "font.h"
#include "main.h"
#include "string.h"

typedef enum {
  OLED_COLOR_NORMAL = 0, // 正常模式 黑底白字
  OLED_COLOR_REVERSED    // 反色模式 白底黑字
} OLED_ColorMode;

void OLED_Init();
void OLED_DisPlay_On();
void OLED_DisPlay_Off();
HAL_StatusTypeDef OLED_I2C_Check(void);

/* 诊断用: I2C 通信失败计数(含重试后仍失败). 可在屏上显示该值, 观察上电是否丢包 */
extern volatile uint16_t oled_i2c_err;

void OLED_NewFrame();
void OLED_ShowFrame();
uint8_t OLED_FrameBusy(void);   // 1 = 一帧还在推送中(此时不要 NewFrame)
void OLED_SetPixel(uint8_t x, uint8_t y, OLED_ColorMode color);

void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, OLED_ColorMode color);
void OLED_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
void OLED_InvertRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void OLED_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, OLED_ColorMode color);
void OLED_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
void OLED_DrawFilledTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, OLED_ColorMode color);
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
void OLED_DrawFilledCircle(uint8_t x, uint8_t y, uint8_t r, OLED_ColorMode color);
void OLED_DrawEllipse(uint8_t x, uint8_t y, uint8_t a, uint8_t b, OLED_ColorMode color);
void OLED_DrawImage(uint8_t x, uint8_t y, const Image *img, OLED_ColorMode color);

void OLED_PrintASCIIChar(uint8_t x, uint8_t y, char ch, const ASCIIFont *font, OLED_ColorMode color);
void OLED_PrintASCIIString(uint8_t x, uint8_t y, char *str, const ASCIIFont *font, OLED_ColorMode color);
void OLED_PrintString(uint8_t x, uint8_t y, char *str, const Font *font, OLED_ColorMode color);
void OLED_DrawJoystick(uint16_t lx_adc, uint16_t ly_adc, uint16_t rx_adc, uint16_t ry_adc);
/* 通用量纲版: 显式给出数值范围(vmin/vmid/vmax), 可直接传入 CRSF 值或其他量纲 */
void OLED_DrawJoystickEx(uint16_t lx, uint16_t ly, uint16_t rx, uint16_t ry,
                         uint16_t vmin, uint16_t vmid, uint16_t vmax);
#endif // __OLED_H__