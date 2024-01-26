#include "ext_error_codes.h"
#include "ext_typedefs.h"

/* hardwarelist requirements */
#include "hwlist_agent.h"

extern struct Node *hw_head;

/* public interfaces */
u32 system_clock_init(void);
u32 sensor_clock_init(void);
