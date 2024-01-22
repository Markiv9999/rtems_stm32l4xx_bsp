#include "./st_hal/i2c.h"
#include "ext_error_codes.h"
#include "ext_typedefs.h"
#include "ov5640.h"
#include "ov5640_reg.h"

struct jpeg_image {
  u8 *head_ptr;
  u8 *tail_ptr;
  size_t img_size;
};

/* External interfaces */
void ov5640_configure_jpeg_720p(void);    // functional
void ov5640_configure_jpeg_1080p(void);   // non functional
void ov5640_configure_color_bar(void);    // functional
void ov5640_configure_color_square(void); // functional
u32 *dcmi_cfg_transfer(void);
void dcmi_cfg_periph(void);
void dcmi_buffer_analisis(struct jpeg_image *, u32 *);
