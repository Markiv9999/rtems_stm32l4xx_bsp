#include "stm32l4r9_module_mspi_mt29.h"

#define IS_DUAL_MEM 0U;

/**
 * --------------------------------------------------------------------------- *
 *       DEVICE SPECIFIC COMMAND FUNCTIONS
 * --------------------------------------------------------------------------- *
 */

struct mspi_cmd mspi_device_write_unlock(void *);
struct mspi_cmd mspi_device_write_lock(void *);
struct mspi_cmd mspi_device_write_enable(void *);
struct mspi_cmd mspi_device_get_status(void *);
struct mspi_cmd mspi_device_page_read_from_nand(void *);
struct mspi_cmd mspi_device_page_read_from_cache_SINGLE(void *);
struct mspi_cmd mspi_device_page_read_from_cache_QUAD(void *);
struct mspi_cmd mspi_device_page_load_SINGLE(void *);
struct mspi_cmd mspi_device_page_load_QUAD(void *);
struct mspi_cmd mspi_device_page_program(void *);
struct mspi_cmd mspi_device_block_erase(void *);

struct mspi_cmd mspi_device_wait_write_enable(void *);
// struct mspi_cmd mspi_device_wait_program_complete(void);
/**
 * --------------------------------------------------------------------------- *
 *       ADDRESS MANAGEMENT LOCAL FUNCTIONS
 * --------------------------------------------------------------------------- *
 */

u32 RA_gen(struct nand_addr nand_addr) {
  u32 RA = 0UL;
  RA |= (nand_addr.block << 17);
  RA |= (nand_addr.page << 11);
  RA >>= 8;
  return RA;
}

u32 CA_gen(struct nand_addr nand_addr) {
  u32 CA = 0UL;
  CA |= ((nand_addr.block & 0x01) << 12);
  CA |= (nand_addr.column);
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

struct mspi_cmd mspi_device_write_unlock(void *placeholder) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x1F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xA0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;
  cmd.data_cmd = 0x00;

  return cmd;
}

struct mspi_cmd mspi_device_write_lock(void *placeholder) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

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

  return cmd;
}

struct mspi_cmd mspi_device_write_enable(void *placeholder) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x06;

  return cmd;
}

struct mspi_cmd mspi_device_get_status(void *placeholder) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

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

struct mspi_cmd mspi_device_page_read_from_nand(void *nand_addr) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

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
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x03;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _16_bit;
  cmd.addr_cmd = CA_gen(*(struct nand_addr *)nand_addr);

  cmd.data_mode = SINGLE;
  cmd.data_size = MT29_PAGE_SIZE - 1;

  return cmd;
}

struct mspi_cmd mspi_device_page_read_from_cache_QUAD(void *nand_addr) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = READ;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0xEB;

  cmd.addr_mode = QUAD;
  cmd.addr_size = _16_bit;
  cmd.addr_cmd = CA_gen(*(struct nand_addr *)nand_addr);

  cmd.data_mode = QUAD;
  cmd.data_size = MT29_PAGE_SIZE - 1;

  return cmd;
}

struct mspi_cmd mspi_device_page_load_SINGLE(void *nand_addr) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

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
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

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
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

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
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = WRITE;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0xD8;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _24_bit;
  cmd.addr_cmd = RA_gen(*(struct nand_addr *)nand_addr);

  return cmd;
}

struct mspi_cmd mspi_device_wait_write_enable(void *) {
  struct mspi_cmd cmd;
  memset(cmd, 0, sizeof(struct mspi_cmd));

  cmd.is_double_mem = IS_DUAL_MEM;
  cmd.fun_mode = AUTOPOLLING;

  cmd.instr_mode = SINGLE;
  cmd.instr_cmd = 0x0F;

  cmd.addr_mode = SINGLE;
  cmd.addr_size = _8_bit;
  cmd.addr_cmd = 0xC0;

  cmd.data_mode = SINGLE;
  cmd.data_size = _8_bit;

  return cmd;
}

/**
 * --------------------------------------------------------------------------- *
 *       DEVICE OBJECT CONSTRUCTOR
 * --------------------------------------------------------------------------- *
 */

struct mspi_device mspi_device_constr(void) {
  struct mspi_device dev = {0};
  dev.write_unlock = &mspi_device_write_unlock;
  dev.write_lock = &mspi_device_write_lock;
  dev.write_enable = &mspi_device_write_enable;
  dev.get_status = &mspi_device_get_status;
  dev.page_read_from_nand = &mspi_device_page_read_from_nand;
  dev.page_read_from_cache_SINGLE = &mspi_device_page_read_from_cache_SINGLE;
  dev.page_read_from_cache_QUAD = &mspi_device_page_read_from_cache_QUAD;
  dev.page_load_SINGLE = &mspi_device_page_load_SINGLE;
  dev.page_load_QUAD = &mspi_device_page_load_QUAD;
  dev.page_program = &mspi_device_page_program;
  dev.block_erase = &mspi_device_block_erase;

  dev.write_enable = &mspi_device_write_enable; // TODO implement
  // dev.wait_program_complete       = &mspi_device_wait_program_complete;

  return dev;
}
