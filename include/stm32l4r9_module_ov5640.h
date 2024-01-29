#pragma once
#include "./st_hal/i2c.h"
#include "ext_error_codes.h"
#include "ext_typedefs.h"
#include "hwlist_agent.h"
#include "stm32l4r9_module_clk_config.h"
#include "stm32l4r9_module_i2c.h"

u32 ov5640_init(void);

/*configuraiton presets */
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
