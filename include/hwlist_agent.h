#pragma once
#include "ext_typedefs.h"
#include "hwlist_handlers.h"
#include "stm32l4r9_module_uart.h"
#include <string.h>

/* register hardware modules */
/* */

extern struct Node *hw_head;

void hwlist_init(void);

void hwlist_require(struct Node **head, u32 (*child_if)(void),
                    void (*father_if)(void));

void hwlist_agent_iterate(struct Node *node);
