#pragma once
/**
* this module has the purpose of initializing a multi-spi interface and
* executing device specific commands, with the final objective of interacting
* with memory modules.
*
* -V
*/

#include <stdint.h>
#include "error_codes.h"
#include "typedefs.h"


/**
 * the module shall not include board/device specific informations,
 * therefore board cmsis headers are not included
 *
 * the methods used for board/device implementation shall be overloaded,
 * therefore defined as a standard interface
 * INCLUDE HERE:
 * - Board specific mspi header file
 * - Device specific mspi header file
 */ /** the following interfaces have been defined:
 * BOARD
 * void mspi_gpio_cfg(void)
 * void mspi_sys_cfg(void)
 * DEVICE
 * void mspi_dev_cfg(void)
 *
 * -V
 */


struct mspi_cmd{
  /* this structure is the command specification
   */

  uint8_t is_double_mem;

  u32 fun_mode;
  u32 instr_mode;
  u32 addr_mode;
  u32 data_mode;

  u32 addr_size;
  u32 data_size;

  unsigned int instr_cmd;
  unsigned int addr_cmd;
  unsigned int data_cmd;
};

struct mspi_interface {
  u8 interface_select;
  u32 speed_hz;
  u32 op_timeout;

  //u32* dma_peripheral_addr; -- not needed set up at configuration of dma controller
  //u32* dma_memory_addr; -- not needed set up at configuration of dma controller

  // function handling function pointers
  void (*registers_cleanup)(void);
  u16 (*wait_busy)(void);
  //u16 (*mspi_data_tx_handler)(void); -- not needed, using only dma
  //u16 (*mspi_data_rx_handler)(void); -- not needed, using only dma
};

/**
* --------------------------------------------------------------------------- *
*       INTERFACE SPECIFIC METHODS
* --------------------------------------------------------------------------- *
*/
int   mspi_init(void);
void  mspi_gpio_cfg(void);
void  mspi_sys_cfg(void);
void  mspi_dev_cfg(void);
void  mspi_interface_cleanup(void);
u16   mspi_interface_wait_busy(void);

// methods declarations
u16 mspi_transfer_dma( struct mspi_cmd (*device_fun_handler)(void *), struct mspi_interface ,void* );
u16 mspi_autopoll_wait( struct mspi_cmd (*device_fun_handler)(void *),struct mspi_interface ,void*,u32 ,u32);
