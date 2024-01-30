#pragma once
#include "./st_hal/i2c.h"
#include "ext_error_codes.h"
#include "ext_typedefs.h"
#include "hwlist_agent.h"
#include "stm32l4r9_module_i2c.h"
#include "stm32l4r9_module_ov5640.h"
#include "stm32l4r9xx.h"

#define MAX_DMA_TRS_SIZE 65535
#define IMG_METADATA_MAX_BYTESIZE 100

/* External interfaces */
void ov5640_configure_jpeg_720p(void);    // NOTE: WORKING
void ov5640_configure_jpeg_1080p(void);   // FIX: NOT WORKING
void ov5640_configure_jpeg_QXGA(void);    // FIX: NOT WORKING
void ov5640_configure_color_bar(void);    // NOTE: WORKING
void ov5640_configure_color_square(void); // NOTE: WORKING
void ov5640_c_advanced_awb(void);         // NOTE: WORKING
void ov5640_c_simple_awb(void);           // XXX: UNTESTED
void ov5640_c_quality_high(void);         // NOTE: WORKING
void ov5640_c_quality_high(void);         // XXX: UNTESTED
void ov5640_c_quality_high(void);         // XXX: UNTESTED

void ov5640_test_fun(void);

u32 dcmi_init(void);
u32 dcmi_dmachannel_init(void);
u32 dcmi_dmamux_init(void);
u32 dcmi_dma_enable(void);
u32 dcmi_peripheral_init(void);
