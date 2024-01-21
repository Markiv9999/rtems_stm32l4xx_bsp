#include "./st_hal/i2c.h"
#include "ext_error_codes.h"
#include "ext_typedefs.h"
#include "ov5640.h"
#include "ov5640_reg.h"

/* External interfaces */
void ov5640_configure_jpeg_qsxga(void);
void dcmi_cfg_transfer(void);
void dcmi_cfg_periph(void);
