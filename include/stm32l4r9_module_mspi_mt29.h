#pragma once
#include <stdint.h>
#include <string.h>

#include "stm32l4r9_module_mspi.h"
#include "stm32l4r9xx.h"
// the difference between the board header files should be solved via a patch
// via an additional header file an example is for instance the instruction
// register
// - for stm32l476 is in the CRR register
// - for stm32l4r9 is in a dedicated IR register
// to make a correct patch file, start by including the new processor,
// thereafter fix the incorrect defines via the new definitions that
// act as a patch

#include "ext_error_codes.h"
#include "ext_typedefs.h"

#define MT29_PAGE_W_SIZE 512
#define MT29_PAGE_SIZE (MT29_PAGE_W_SIZE * 4)
#define MT29_BLOCK_SIZE (MT29_PAGE_B_SIZE * 64)
#define MT29_IC_SIZE (MT29_BLOCK_B_SIZE * 2048)

#define WEL_MASK 0x03U
#define WEL_MATCH 0x02U

// data
//

struct nand_addr {
  uint32_t block;
  uint32_t page;
  uint32_t column;
};

struct mspi_device {
  // device specific method function pointers
  struct mspi_cmd (*write_unlock)(void *);
  struct mspi_cmd (*write_lock)(void *);
  struct mspi_cmd (*write_enable)(void *);
  struct mspi_cmd (*get_status)(void *);
  struct mspi_cmd (*page_read_from_nand)(void *);
  struct mspi_cmd (*page_read_from_cache_SINGLE)(void *);
  struct mspi_cmd (*page_read_from_cache_QUAD)(void *);
  struct mspi_cmd (*page_load_SINGLE)(void *);
  struct mspi_cmd (*page_load_QUAD)(void *);
  struct mspi_cmd (*page_program)(void *);
  struct mspi_cmd (*block_erase)(void *);

  struct mspi_cmd (*wait_write_enable)(void *); // TODO implement
  struct mspi_cmd (*wait_program_complete)(void *);
};

// methods

struct mspi_device mspi_device_constr(void);
