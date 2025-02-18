#include "stm32l4r9_module_mspi_mt29.h"

#define IS_DUAL_MEM 0U;

/**
 * --------------------------------------------------------------------------- *
 *       DEVICE SPECIFIC COMMAND FUNCTIONS
 * --------------------------------------------------------------------------- *
 */

struct mspi_cmd mspi_device_ecc_disable(void *);
struct mspi_cmd mspi_device_ecc_enable(void *);
struct mspi_cmd mspi_device_write_unlock(void *);
struct mspi_cmd mspi_device_write_lock(void *);
struct mspi_cmd mspi_device_write_enable(void *);
struct mspi_cmd mspi_device_get_status(void *);
struct mspi_cmd mspi_device_get_unlock_status(void *);
struct mspi_cmd mspi_device_get_config_status(void *);
struct mspi_cmd mspi_device_page_read_from_nand(void *);
struct mspi_cmd mspi_device_page_read_from_cache_SINGLE(void *);
struct mspi_cmd mspi_device_page_read_from_cache_QUAD(void *);
struct mspi_cmd mspi_device_page_load_SINGLE(void *);
struct mspi_cmd mspi_device_page_load_QUAD(void *);
struct mspi_cmd mspi_device_page_program(void *);
struct mspi_cmd mspi_device_block_erase(void *);

struct mspi_cmd mspi_device_wait_write_enable(void *);
struct mspi_cmd mspi_device_wait_oip(void *);

u32 RA_gen(struct nand_addr nand_addr);
u32 CA_gen(struct nand_addr nand_addr);

/**
 * --------------------------------------------------------------------------- *
 *       ADDRESS MANAGEMENT LOCAL FUNCTIONS
 * --------------------------------------------------------------------------- *
 */

u32 RA_gen(struct nand_addr nand_addr) {
  u32 RA = {0};
  /*
  // OLD
  RA |= (nand_addr.block << 17);
  RA |= (nand_addr.page << 11);
  RA >>= 8;
  */
  // NEW
  RA |= ((nand_addr.block & 0x7FF) << 6);
  RA |= (nand_addr.page & 0x3F);
  return RA;

  // old and new are indifferent
}

u32 CA_gen(struct nand_addr nand_addr) {
  u32 CA = {0};
  // CA |= ((nand_addr.block & 0x01) << 12); // OLD
  CA |= (nand_addr.column); // NEW
  return CA;
}

/**
 * --------------------------------------------------------------------------- *
 *       COMMAND DEFINITION
 * --------------------------------------------------------------------------- *
 */

// TODO: reduce the scope of these enums only to this file
enum fun_modes {
  WRITE = 0,
  READ,
  AUTOPOLLING,
  MEMMAPPED,
};

enum phase_modes {
  NP = 0,
  SINGLE,
  DOUBLE,
  QUAD,
  OCTA,
};

enum addr_size {
  _8_bit = 0,
  _16_bit,
  _24_bit,
  _32_bit,
};

struct mspi_cmd mspi_device_ecc_disable(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x1F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xB0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;
  cmd.data_cmd = 0b00000000;
  cmd.data_cmd_embedded = 0x1;

  return cmd;
}

struct mspi_cmd mspi_device_ecc_enable(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x1F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xB0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;
  cmd.data_cmd = 0b00010000;
  cmd.data_cmd_embedded = 0x1;

  return cmd;
}

struct mspi_cmd mspi_device_write_lock(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x1F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xA0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;
  cmd.data_cmd = 0b01111100;
  cmd.data_cmd_embedded = 0x1;

  return cmd;
}

struct mspi_cmd mspi_device_write_unlock(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x1F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xA0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;
  cmd.data_cmd = 0b00000010;
  cmd.data_cmd_embedded = 0x1;

  return cmd;
}

struct mspi_cmd mspi_device_write_enable(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x06;

  return cmd;
}

struct mspi_cmd mspi_device_get_unlock_status(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x0F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xA0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;

  return cmd;
}

struct mspi_cmd mspi_device_get_config_status(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x0F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xB0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;

  return cmd;
}

struct mspi_cmd mspi_device_get_status(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x0F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xC0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;

  return cmd;
}

/*
struct mspi_cmd mspi_device_get_id(void *placeholder) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x0F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xC0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;

  return cmd;
}
*/

struct mspi_cmd mspi_device_page_read_from_nand(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x13;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _24_bit;

  cmd.addr_cmd = RA_gen(*(struct nand_addr *)nand_addr);

  cmd.data_mode = NP;

  return cmd;
}

struct mspi_cmd mspi_device_page_read_from_cache_SINGLE(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x03;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _16_bit;
  cmd.addr_cmd = CA_gen(*(struct nand_addr *)nand_addr);

  cmd.dummy_size = 0x8;

  cmd.data_mode = SINGLE;
  cmd.data_size = MT29_PAGE_SIZE - 1;

  return cmd;
}

struct mspi_cmd mspi_device_page_read_from_cache_QUAD(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x6B;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _16_bit;
  cmd.addr_cmd = CA_gen(*(struct nand_addr *)nand_addr);

  cmd.dummy_size = 0x8;

  cmd.data_mode = QUAD;
  cmd.data_size = MT29_PAGE_SIZE - 1;

  return cmd;
}

struct mspi_cmd mspi_device_page_load_SINGLE(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x02;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _16_bit;
  cmd.addr_cmd = CA_gen(*(struct nand_addr *)nand_addr);

  cmd.data_mode = SINGLE;
  cmd.data_size = MT29_PAGE_SIZE - 1;

  return cmd;
}

struct mspi_cmd mspi_device_page_load_QUAD(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x32;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _16_bit;
  cmd.addr_cmd = CA_gen(*(struct nand_addr *)nand_addr);

  cmd.data_mode = QUAD;
  cmd.data_size = MT29_PAGE_SIZE - 1;

  return cmd;
}

struct mspi_cmd mspi_device_page_program(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x10;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _24_bit;
  cmd.addr_cmd = RA_gen(*(struct nand_addr *)nand_addr);

  return cmd;
}

struct mspi_cmd mspi_device_block_erase(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0xD8;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _24_bit;
  cmd.addr_cmd = RA_gen(*(struct nand_addr *)nand_addr);

  return cmd;
}

struct mspi_cmd mspi_device_wait_write_enable(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = AUTOPOLLING;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x0F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xC0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;
  cmd.data_cmd_embedded = 0x0;

  cmd.autopoll_mask = WEL_MASK;
  cmd.autopoll_match = WEL_MATCH;

  return cmd;
}

struct mspi_cmd mspi_device_wait_oip(void *nand_addr) {
  struct mspi_cmd cmd = {0};
  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = AUTOPOLLING;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x0F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xC0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;
  cmd.data_cmd_embedded = 0x0;

  cmd.autopoll_mask = OIP_MASK;
  cmd.autopoll_match = OIP_MATCH;

  return cmd;
}

/**
 * --------------------------------------------------------------------------- *
 *       DEVICE OBJECT CONSTRUCTOR
 * --------------------------------------------------------------------------- *
 */

struct mspi_device mspi_device_constr(void) {
  struct mspi_device dev = {0};
  dev.ecc_disable = &mspi_device_ecc_disable;
  dev.ecc_enable = &mspi_device_ecc_enable;
  dev.write_unlock = &mspi_device_write_unlock;
  dev.write_lock = &mspi_device_write_lock;
  dev.write_enable = &mspi_device_write_enable;
  dev.write_enable_polled = &mspi_device_wait_write_enable;
  dev.wait_oip = &mspi_device_wait_oip;
  dev.get_status = &mspi_device_get_status;
  dev.get_unlock_status = &mspi_device_get_unlock_status;
  dev.get_config_status = &mspi_device_get_config_status;
  dev.page_read_from_nand = &mspi_device_page_read_from_nand;
  dev.page_read_from_cache_SINGLE = &mspi_device_page_read_from_cache_SINGLE;
  dev.page_read_from_cache_QUAD = &mspi_device_page_read_from_cache_QUAD;
  dev.page_load_SINGLE = &mspi_device_page_load_SINGLE;
  dev.page_load_QUAD = &mspi_device_page_load_QUAD;
  dev.page_program = &mspi_device_page_program;
  dev.block_erase = &mspi_device_block_erase;

  // dev.wait_program_complete       = &mspi_device_wait_program_complete;

  return dev;
}
