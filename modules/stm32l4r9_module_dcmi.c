#include "stm32l4r9_module_dcmi.h"
// 13.1.4.2 Jpg mode capture
//
void write_i2c(u8, u8, u8);
#define OV5640_DEV_ADDR 0x3c

void write_i2c(u8 val1, u8 val2, u8 val3) {
  // wrapper for i2c hal driver
  u8 buffer[] = {val1, val2, val3};
  HAL_I2C_Master_Transmit(&hi2c1, OV5640_DEV_ADDR << 1, buffer, 3, 1000);
}

void ov5640_configure_jpeg_qsxga(void) {
  // 13.3.2 720 P
  write_i2c(0x31, 0x03, 0x11);
  write_i2c(0x30, 0x08, 0x82);
  write_i2c(0x30, 0x08, 0x42);
  write_i2c(0x31, 0x03, 0x03);
  write_i2c(0x30, 0x17, 0xff);
  write_i2c(0x30, 0x18, 0xff);
  write_i2c(0x30, 0x34, 0x1a);
  write_i2c(0x30, 0x35, 0x11);
  write_i2c(0x30, 0x36, 0x69);
  write_i2c(0x30, 0x37, 0x13);
  write_i2c(0x31, 0x08, 0x01);
  write_i2c(0x36, 0x30, 0x36);
  write_i2c(0x36, 0x31, 0x0e);
  write_i2c(0x36, 0x32, 0xe2);
  write_i2c(0x36, 0x33, 0x12);
  write_i2c(0x36, 0x21, 0xe0);
  write_i2c(0x37, 0x04, 0xa0);
  write_i2c(0x37, 0x03, 0x5a);
  write_i2c(0x37, 0x15, 0x78);
  write_i2c(0x37, 0x17, 0x01);
  write_i2c(0x37, 0x0b, 0x60);
  write_i2c(0x37, 0x05, 0x1a);
  write_i2c(0x39, 0x05, 0x02);
  write_i2c(0x39, 0x06, 0x10);
  write_i2c(0x39, 0x01, 0x0a);
  write_i2c(0x37, 0x31, 0x12);
  write_i2c(0x36, 0x00, 0x08);
  write_i2c(0x36, 0x01, 0x33);
  write_i2c(0x30, 0x2d, 0x60);
  write_i2c(0x36, 0x20, 0x52);
  write_i2c(0x37, 0x1b, 0x20);
  write_i2c(0x47, 0x1c, 0x50);
  write_i2c(0x3a, 0x13, 0x43);
  write_i2c(0x3a, 0x18, 0x00);
  write_i2c(0x3a, 0x19, 0xf8);
  write_i2c(0x36, 0x35, 0x13);
  write_i2c(0x36, 0x36, 0x03);
  write_i2c(0x36, 0x34, 0x40);
  write_i2c(0x36, 0x22, 0x01);
  write_i2c(0x3c, 0x01, 0x34);
  write_i2c(0x3c, 0x04, 0x28);
  write_i2c(0x3c, 0x05, 0x98);
  write_i2c(0x3c, 0x06, 0x00);
  write_i2c(0x3c, 0x07, 0x07);
  write_i2c(0x3c, 0x08, 0x00);
  write_i2c(0x3c, 0x09, 0x1c);
  write_i2c(0x3c, 0x0a, 0x9c);
  write_i2c(0x3c, 0x0b, 0x40);
  write_i2c(0x38, 0x20, 0x41);
  write_i2c(0x38, 0x21, 0x27);
  write_i2c(0x38, 0x14, 0x31);
  write_i2c(0x38, 0x15, 0x31);
  write_i2c(0x38, 0x00, 0x00);
  write_i2c(0x38, 0x01, 0x00);
  write_i2c(0x38, 0x02, 0x00);
  write_i2c(0x38, 0x03, 0xfa);
  write_i2c(0x38, 0x04, 0x0a);
  write_i2c(0x38, 0x05, 0x3f);
  write_i2c(0x38, 0x06, 0x06);
  write_i2c(0x38, 0x07, 0xa9);
  write_i2c(0x38, 0x08, 0x05);
  write_i2c(0x38, 0x09, 0x00);
  write_i2c(0x38, 0x0a, 0x02);
  write_i2c(0x38, 0x0b, 0xd0);
  write_i2c(0x38, 0x0c, 0x07);
  write_i2c(0x38, 0x0d, 0x64);
  write_i2c(0x38, 0x0e, 0x02);
  write_i2c(0x38, 0x0f, 0xe4);
  write_i2c(0x38, 0x10, 0x00);
  write_i2c(0x38, 0x11, 0x10);
  write_i2c(0x38, 0x12, 0x00);
  write_i2c(0x38, 0x13, 0x04);
  write_i2c(0x36, 0x18, 0x00);
  write_i2c(0x36, 0x12, 0x29);
  write_i2c(0x37, 0x08, 0x64);
  write_i2c(0x37, 0x09, 0x52);
  write_i2c(0x37, 0x0c, 0x03);
  write_i2c(0x3a, 0x02, 0x02);
  write_i2c(0x3a, 0x03, 0xe4);
  write_i2c(0x3a, 0x08, 0x01);
  write_i2c(0x3a, 0x09, 0xbc);
  write_i2c(0x3a, 0x0a, 0x01);
  write_i2c(0x3a, 0x0b, 0x72);
  write_i2c(0x3a, 0x0e, 0x01);
  write_i2c(0x3a, 0x0d, 0x02);
  write_i2c(0x3a, 0x14, 0x02);
  write_i2c(0x3a, 0x15, 0xe4);
  write_i2c(0x40, 0x01, 0x02);
  write_i2c(0x40, 0x04, 0x02);
  write_i2c(0x30, 0x00, 0x00);
  write_i2c(0x30, 0x02, 0x00);
  write_i2c(0x30, 0x04, 0xff);
  write_i2c(0x30, 0x06, 0xff);
  write_i2c(0x30, 0x0e, 0x58);
  write_i2c(0x30, 0x2e, 0x00);
  write_i2c(0x43, 0x00, 0x30);
  write_i2c(0x50, 0x1f, 0x00);
  write_i2c(0x47, 0x13, 0x02);
  write_i2c(0x44, 0x07, 0x04);
  write_i2c(0x44, 0x0e, 0x00);
  write_i2c(0x46, 0x0b, 0x35);
  write_i2c(0x46, 0x0c, 0x22);
  write_i2c(0x38, 0x24, 0x04);
  write_i2c(0x50, 0x00, 0xa7);
  write_i2c(0x50, 0x01, 0x83);
  write_i2c(0x51, 0x80, 0xff);
  write_i2c(0x51, 0x81, 0xf2);
  write_i2c(0x51, 0x82, 0x00);
  write_i2c(0x51, 0x83, 0x14);
  write_i2c(0x51, 0x84, 0x25);
  write_i2c(0x51, 0x85, 0x24);
  write_i2c(0x51, 0x86, 0x09);
  write_i2c(0x51, 0x87, 0x09);
  write_i2c(0x51, 0x88, 0x09);
  write_i2c(0x51, 0x89, 0x75);
  write_i2c(0x51, 0x8a, 0x54);
  write_i2c(0x51, 0x8b, 0xe0);
  write_i2c(0x51, 0x8c, 0xb2);
  write_i2c(0x51, 0x8d, 0x42);
  write_i2c(0x51, 0x8e, 0x3d);
  write_i2c(0x51, 0x8f, 0x56);
  write_i2c(0x51, 0x90, 0x46);
  write_i2c(0x51, 0x91, 0xf8);
  write_i2c(0x51, 0x92, 0x04);
  write_i2c(0x51, 0x93, 0x70);
  write_i2c(0x51, 0x94, 0xf0);
  write_i2c(0x51, 0x95, 0xf0);
  write_i2c(0x51, 0x96, 0x03);
  write_i2c(0x51, 0x97, 0x01);
  write_i2c(0x51, 0x98, 0x04);
  write_i2c(0x51, 0x99, 0x12);
  write_i2c(0x51, 0x9a, 0x04);
  write_i2c(0x51, 0x9b, 0x00);
  write_i2c(0x51, 0x9c, 0x06);
  write_i2c(0x51, 0x9d, 0x82);
  write_i2c(0x51, 0x9e, 0x38);
  write_i2c(0x53, 0x81, 0x1e);
  write_i2c(0x53, 0x82, 0x5b);
  write_i2c(0x53, 0x83, 0x08);
  write_i2c(0x53, 0x84, 0x0a);
  write_i2c(0x53, 0x85, 0x7e);
  write_i2c(0x53, 0x86, 0x88);
  write_i2c(0x53, 0x87, 0x7c);
  write_i2c(0x53, 0x88, 0x6c);
  write_i2c(0x53, 0x89, 0x10);
  write_i2c(0x53, 0x8a, 0x01);
  write_i2c(0x53, 0x8b, 0x98);
  write_i2c(0x53, 0x00, 0x08);
  write_i2c(0x53, 0x01, 0x30);
  write_i2c(0x53, 0x02, 0x10);
  write_i2c(0x53, 0x03, 0x00);
  write_i2c(0x53, 0x04, 0x08);
  write_i2c(0x53, 0x05, 0x30);
  write_i2c(0x53, 0x06, 0x08);
  write_i2c(0x53, 0x07, 0x16);
  write_i2c(0x53, 0x09, 0x08);
  write_i2c(0x53, 0x0a, 0x30);
  write_i2c(0x53, 0x0b, 0x04);
  write_i2c(0x53, 0x0c, 0x06);
  write_i2c(0x54, 0x80, 0x01);
  write_i2c(0x54, 0x81, 0x08);
  write_i2c(0x54, 0x82, 0x14);
  write_i2c(0x54, 0x83, 0x28);
  write_i2c(0x54, 0x84, 0x51);
  write_i2c(0x54, 0x85, 0x65);
  write_i2c(0x54, 0x86, 0x71);
  write_i2c(0x54, 0x87, 0x7d);
  write_i2c(0x54, 0x88, 0x87);
  write_i2c(0x54, 0x89, 0x91);
  write_i2c(0x54, 0x8a, 0x9a);
  write_i2c(0x54, 0x8b, 0xaa);
  write_i2c(0x54, 0x8c, 0xb8);
  write_i2c(0x54, 0x8d, 0xcd);
  write_i2c(0x54, 0x8e, 0xdd);
  write_i2c(0x54, 0x8f, 0xea);
  write_i2c(0x54, 0x90, 0x1d);
  write_i2c(0x55, 0x80, 0x02);
  write_i2c(0x55, 0x83, 0x40);
  write_i2c(0x55, 0x84, 0x10);
  write_i2c(0x55, 0x89, 0x10);
  write_i2c(0x55, 0x8a, 0x00);
  write_i2c(0x55, 0x8b, 0xf8);
  write_i2c(0x58, 0x00, 0x23);
  write_i2c(0x58, 0x01, 0x14);
  write_i2c(0x58, 0x02, 0x0f);
  write_i2c(0x58, 0x03, 0x0f);
  write_i2c(0x58, 0x04, 0x12);
  write_i2c(0x58, 0x05, 0x26);
  write_i2c(0x58, 0x06, 0x0c);
  write_i2c(0x58, 0x07, 0x08);
  write_i2c(0x58, 0x08, 0x05);
  write_i2c(0x58, 0x09, 0x05);
  write_i2c(0x58, 0x0a, 0x08);
  write_i2c(0x58, 0x0b, 0x0d);
  write_i2c(0x58, 0x0c, 0x08);
  write_i2c(0x58, 0x0d, 0x03);
  write_i2c(0x58, 0x0e, 0x00);
  write_i2c(0x58, 0x0f, 0x00);
  write_i2c(0x58, 0x10, 0x03);
  write_i2c(0x58, 0x11, 0x09);
  write_i2c(0x58, 0x12, 0x07);
  write_i2c(0x58, 0x13, 0x03);
  write_i2c(0x58, 0x14, 0x00);
  write_i2c(0x58, 0x15, 0x01);
  write_i2c(0x58, 0x16, 0x03);
  write_i2c(0x58, 0x17, 0x08);
  write_i2c(0x58, 0x18, 0x0d);
  write_i2c(0x58, 0x19, 0x08);
  write_i2c(0x58, 0x1a, 0x05);
  write_i2c(0x58, 0x1b, 0x06);
  write_i2c(0x58, 0x1c, 0x08);
  write_i2c(0x58, 0x1d, 0x0e);
  write_i2c(0x58, 0x1e, 0x29);
  write_i2c(0x58, 0x1f, 0x17);
  write_i2c(0x58, 0x20, 0x11);
  write_i2c(0x58, 0x21, 0x11);
  write_i2c(0x58, 0x22, 0x15);
  write_i2c(0x58, 0x23, 0x28);
  write_i2c(0x58, 0x24, 0x46);
  write_i2c(0x58, 0x25, 0x26);
  write_i2c(0x58, 0x26, 0x08);
  write_i2c(0x58, 0x27, 0x26);
  write_i2c(0x58, 0x28, 0x64);
  write_i2c(0x58, 0x29, 0x26);
  write_i2c(0x58, 0x2a, 0x24);
  write_i2c(0x58, 0x2b, 0x22);
  write_i2c(0x58, 0x2c, 0x24);
  write_i2c(0x58, 0x2d, 0x24);
  write_i2c(0x58, 0x2e, 0x06);
  write_i2c(0x58, 0x2f, 0x22);
  write_i2c(0x58, 0x30, 0x40);
  write_i2c(0x58, 0x31, 0x42);
  write_i2c(0x58, 0x32, 0x24);
  write_i2c(0x58, 0x33, 0x26);
  write_i2c(0x58, 0x34, 0x24);
  write_i2c(0x58, 0x35, 0x22);
  write_i2c(0x58, 0x36, 0x22);
  write_i2c(0x58, 0x37, 0x26);
  write_i2c(0x58, 0x38, 0x44);
  write_i2c(0x58, 0x39, 0x24);
  write_i2c(0x58, 0x3a, 0x26);
  write_i2c(0x58, 0x3b, 0x28);
  write_i2c(0x58, 0x3c, 0x42);
  write_i2c(0x58, 0x3d, 0xce);
  write_i2c(0x50, 0x25, 0x00);
  write_i2c(0x3a, 0x0f, 0x30);
  write_i2c(0x3a, 0x10, 0x28);
  write_i2c(0x3a, 0x1b, 0x30);
  write_i2c(0x3a, 0x1e, 0x26);
  write_i2c(0x3a, 0x11, 0x60);
  write_i2c(0x3a, 0x1f, 0x14);
  write_i2c(0x30, 0x08, 0x02);
  write_i2c(0x30, 0x35, 0x21);
}
