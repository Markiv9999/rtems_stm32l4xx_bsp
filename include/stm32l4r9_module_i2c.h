#pragma once
#include "ext_typedefs.h"
#include "gpio.h"
#include "i2c.h"

/* hardwarelist requirements */
#include "hwlist_agent.h"
extern struct Node *hw_head;

u32 i2c1_init(void);
