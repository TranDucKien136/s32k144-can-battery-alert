/*
 * lcd1602_i2c.c
 *
 * Driver LCD 16x2 qua PCF8574. So do chan PCF8574 -> LCD (loai module pho bien):
 *   P0 = RS, P1 = RW, P2 = EN, P3 = den nen, P4..P7 = D4..D7
 * LCD chay che do 4 bit: moi byte gui thanh 2 nua (4 bit cao truoc), moi nua kem xung EN.
 */
#include "lcd1602_i2c.h"
#include "sdk_project_config.h"
#include "osif.h"

#define LCD_RS              0x01u
#define LCD_EN              0x04u
#define LCD_BACKLIGHT       0x08u
#define LCD_I2C_TIMEOUT_MS  50u

static uint8_t s_lcdAddress = 0x27u;

static status_t lcd_write_raw(const uint8_t *buf, uint32_t len)
{
    /* Bus dung chung voi board slave nen phai dat lai dia chi truoc moi lan gui */
    (void)I2C_MasterSetSlaveAddress(&lpi2c0_instance, s_lcdAddress, false);
    return I2C_MasterSendDataBlocking(&lpi2c0_instance, buf, len, true, LCD_I2C_TIMEOUT_MS);
}

/* Gui 4 bit cao cua 'value' (dung khi khoi tao) */
static status_t lcd_write_nibble(uint8_t value, uint8_t mode)
{
    uint8_t b = (uint8_t)((value & 0xF0u) | mode | LCD_BACKLIGHT);
    uint8_t buf[2] = { (uint8_t)(b | LCD_EN), b };
    return lcd_write_raw(buf, 2u);
}

/* Gui 1 byte (lenh hoac ky tu) = 2 nua 4 bit trong cung 1 lan truyen I2C */
static status_t lcd_send(uint8_t value, uint8_t mode)
{
    uint8_t hi = (uint8_t)((value & 0xF0u) | mode | LCD_BACKLIGHT);
    uint8_t lo = (uint8_t)(((uint8_t)(value << 4u) & 0xF0u) | mode | LCD_BACKLIGHT);
    uint8_t buf[4] = { (uint8_t)(hi | LCD_EN), hi, (uint8_t)(lo | LCD_EN), lo };
    return lcd_write_raw(buf, 4u);
}

static void lcd_command(uint8_t cmd)
{
    (void)lcd_send(cmd, 0u);
}

status_t LCD1602_Init(uint8_t address)
{
    uint8_t backlight = LCD_BACKLIGHT;
    status_t status;

    s_lcdAddress = address;
    OSIF_TimeDelay(50u);                    /* LCD can > 40 ms sau khi cap nguon */

    /* Thu ghi 1 byte: neu khong co ACK thi khong co LCD o dia chi nay */
    status = lcd_write_raw(&backlight, 1u);
    if (status != STATUS_SUCCESS)
    {
        return status;
    }

    /* Trinh tu khoi dong che do 4 bit theo datasheet HD44780 */
    (void)lcd_write_nibble(0x30u, 0u); OSIF_TimeDelay(5u);
    (void)lcd_write_nibble(0x30u, 0u); OSIF_TimeDelay(2u);
    (void)lcd_write_nibble(0x30u, 0u); OSIF_TimeDelay(2u);
    (void)lcd_write_nibble(0x20u, 0u); OSIF_TimeDelay(2u);

    lcd_command(0x28u);                     /* 4 bit, 2 dong, font 5x8 */
    lcd_command(0x08u);                     /* tat hien thi */
    LCD1602_Clear();
    lcd_command(0x06u);                     /* con tro tu tang, khong dich man hinh */
    lcd_command(0x0Cu);                     /* bat hien thi, an con tro */

    return STATUS_SUCCESS;
}

void LCD1602_Clear(void)
{
    lcd_command(0x01u);
    OSIF_TimeDelay(3u);                     /* lenh clear mat ~1.6 ms */
}

void LCD1602_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t base = (row == 0u) ? 0x00u : 0x40u;
    lcd_command((uint8_t)(0x80u | (base + col)));
}

void LCD1602_Print(const char *str)
{
    while (*str != '\0')
    {
        (void)lcd_send((uint8_t)*str, LCD_RS);
        str++;
    }
}
