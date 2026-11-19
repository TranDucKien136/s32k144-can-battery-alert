/*
 * lcd1602_i2c.h
 *
 * LCD 16x2 (HD44780) qua module I2C PCF8574, dung chung bus LPI2C0 voi board slave.
 */
#ifndef LCD1602_I2C_H_
#define LCD1602_I2C_H_

#include <stdint.h>
#include "status.h"

/* Khoi tao LCD o dia chi I2C 'address' (thuong 0x27 hoac 0x3F).
 * Tra ve STATUS_SUCCESS neu LCD tra loi (ACK) tren bus. */
status_t LCD1602_Init(uint8_t address);
void LCD1602_Clear(void);
void LCD1602_SetCursor(uint8_t row, uint8_t col);
void LCD1602_Print(const char *str);

#endif /* LCD1602_I2C_H_ */
