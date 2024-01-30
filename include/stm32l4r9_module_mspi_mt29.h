#pragma once
#include <stdint.h>

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

#define MT29_PAGE_W_SIZE 1024
#define MT29_PAGE_SIZE (MT29_PAGE_W_SIZE * 4)

#define MT29_MAX_PAGEn 64
#define MT29_MAX_BLOCKn 2048

#define WEL_MASK 0x03U
#define WEL_MATCH 0x02U

#define OIP_MASK 0x01UL
#define OIP_MATCH 0x00UL
// data
//

struct nand_addr {
  uint32_t block;
  uint32_t page;
  uint32_t column;
};

struct mspi_device {
  // device specific method function pointers
  struct mspi_cmd (*ecc_disable)(void *);
  struct mspi_cmd (*ecc_enable)(void *);
  struct mspi_cmd (*write_unlock)(void *);
  struct mspi_cmd (*write_lock)(void *);
  struct mspi_cmd (*write_enable)(void *);
  struct mspi_cmd (*write_enable_polled)(void *);
  struct mspi_cmd (*get_status)(void *);
  struct mspi_cmd (*get_unlock_status)(void *);
  struct mspi_cmd (*get_config_status)(void *);
  struct mspi_cmd (*page_read_from_nand)(void *);
  struct mspi_cmd (*page_read_from_cache_SINGLE)(void *);
  struct mspi_cmd (*page_read_from_cache_QUAD)(void *);
  struct mspi_cmd (*page_load_SINGLE)(void *);
  struct mspi_cmd (*page_load_QUAD)(void *);
  struct mspi_cmd (*page_program)(void *);
  struct mspi_cmd (*block_erase)(void *);

  struct mspi_cmd (*wait_write_enable)(void *);
  struct mspi_cmd (*wait_oip)(void *);
};

// methods

struct mspi_device mspi_device_constr(void);
