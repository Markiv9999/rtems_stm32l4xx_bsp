#pragma once
#include "ext_typedefs.h"
#include "hwlist_handlers.h"

void hwlist_init(void);

void hwlist_require(struct Node *head, u32 (*child_if)(void),
                    void (*father_if)(void));
