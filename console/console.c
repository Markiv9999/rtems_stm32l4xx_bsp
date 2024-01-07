/* TODO: remove comments and move them to docs */

#include <rtems/console.h>
/* NOTE: What does rtems/console.h contain?
 * Contains the generic console drvice driver interface for all boards
 * It is the interface between the boards drivers and the standard C libarary
 * */

#include <rtems/termiostypes.h>

#include <bsp.h>
#include <console.h>
#include <rtems/bspIo.h> //for printk?

#include "hal_uart.h"

/* NOTE: it also includes what seems to be a dvice specification header:
 * #include <dev/serial/zynq-uart.h>
 * the file is located in:
 * ./src/rtems/bsps/include/dev/serial/zynq-uart.h
 * it is therefore a generic BSP include (device specifically)
 * it seems that only a couple of bsp have those generic shared headers,
 * and the files semms to be quite old (circa 2014) therefore it seems like a
 * legacy implementation
 *
 * the file inclues the only header:  rtems/termiosdevice.h
 *
 */

/* TODO: determine classes for each of the low level consol driver
 * implementation then cosnsider at least 5 drivers developed in the last 5
 * years and schematize the types of implementation. This work is useful for the
 * Thesis
 * (1) ./x86_64/amd64/console/eficonsole.c: * Copyright (C) 2023 Karel Gardas
 * ---
 * (2)./aarch64/xilinx-zynqmp/console/console.c: * Copyright (C) 2022 On-Line
 * Applications Research Corporation (OAR)
 * ---
 * (3)./aarch64/raspberrypi/console/console.c: * Copyright (C) 2022 Mohd Noor
 * Aman
 * ---
 * (4)./aarch64/xilinx-versal/console/console.c: * Copyright (C) 2021
 * Gedare Bloom <gedare@rtems.org>
 * ---
 * (5)./microblaze/microblaze_fpga/console/console-io.c: * Copyright (C) 2021
 * On-Line Applications Research Corporation (OAR)
 *    utilizes dev/serial/uartlite.h
 * ---
 * (6) ./riscv/noel/console/console-config.c: * Copyright (c) 2021 Cobham
 * Gaisler AB.
 *
 */

/* NOTE:it seems that each bsp uses a different structure for the uart instance
 * context. Therefore it is maybe better ot start from the necessary items used
 * for the rtems_device_install method
 * the method needs:
 * - device fie path template
 * - persistent device handler
 *    (how to obtain it?) initialized in ./shared/dev/serial/zynq-uart.c
 *    the directory ./shared/dev/serial contains a lot of implementations that
 * can be potentially re used or used as an example
 *    the structure is declared in ./include/dev/serial/zynq-uart.h as extern
 *    of the type rtems_termios_device_handler
 * - device flow control handler (optional)
 * - device context, obtaied via the macro
 * RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER:
 * defined in: ./cpukit/include/rtems/termiosdevice.h:
 * Macro Body:
 * The body of the macro is enclosed in curly braces { ... }, which means it's
 * designed to initialize a structure or a similar compound data type. Inside
 * the macro body, there are several elements: {
 * RTEMS_INTERRUPT_LOCK_INITIALIZER(name) }: This initializes an interrupt lock
 * structure. The RTEMS_INTERRUPT_LOCK_INITIALIZER is another macro that
 * initializes an interrupt lock. The name passed to the macro is used here,
 * likely as a label or identifier for the lock.
 *    rtems_termios_device_lock_acquire_default:
 *    This is a function pointer, likely pointing to a default lock acquire
 * function. In the context of RTEMS, this function would be used to acquire a
 * lock on the termios device context.
 * rtems_termios_device_lock_release_default: Similar to the previous entry,
 * this is a function pointer, probably pointing to a default lock release
 * function. It would be used to release the lock on the termios device context.
 */

// NOTE: to be honest i feel like since i will use the CMSIS headers i will not
// need to use a superset of the rtems termios device context. i can try to work
// with just it and the cmsis headers

/* It seems that device specific contexes are usally some expansion of the
 * termios device context */

/* - from include - just as reference
#define RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER( name ) \
 * { \
 * { RTEMS_INTERRUPT_LOCK_INITIALIZER( name ) }, \
 *   rtems_termios_device_lock_acquire_default, \
 *   rtems_termios_device_lock_release_default \
 * }
 */

#define STM32L4_UART_DEFAULT_BAUD 1000
/* TODO: move prototypes */
void stm32l4_uart_initialize(rtems_termios_device_context *base);

static bool stm32l4_uart_first_open(rtems_termios_tty *tty,
                                    rtems_termios_device_context *base,
                                    struct termios *term,
                                    rtems_libio_open_close_args_t *args);

static void stm32l4_uart_last_close(rtems_termios_tty *tty,
                                    rtems_termios_device_context *base,
                                    rtems_libio_open_close_args_t *args);

int stm32l4_uart_read_polled(rtems_termios_device_context *base);

void stm32l4_uart_write_polled(rtems_termios_device_context *base, char c);

static bool stm32l4_uart_set_attributes(rtems_termios_device_context *context,
                                        const struct termios *term);

/* ===== IMPLEMENTATION OF METHODS ========================================== */
/* ----- sub-block I -------------------------------------------------------- */
void stm32l4_uart_initialize(rtems_termios_device_context *base) {
  /* TODO: link */
}

static bool stm32l4_uart_first_open(rtems_termios_tty *tty,
                                    rtems_termios_device_context *base,
                                    struct termios *term,
                                    rtems_libio_open_close_args_t *args) {

  rtems_termios_set_initial_baud(tty, STM32L4_UART_DEFAULT_BAUD);
  stm32l4_uart_initialize(base);

  return true;
}
/* ----- sub-block II ------------------------------------------------------- */
static void stm32l4_uart_last_close(rtems_termios_tty *tty,
                                    rtems_termios_device_context *base,
                                    rtems_libio_open_close_args_t *args) {
  /* TODO: link */
  /* TODO: evaluate if you actually need it - see zynq implementation */
}

/* ----- sub-block III ------------------------------------------------------ */
int stm32l4_uart_read_polled(rtems_termios_device_context *base) {
  /* TODO: link */
}

/* ----- sub-block IV ------------------------------------------------------- */
void stm32l4_uart_write_polled(rtems_termios_device_context *base, char c) {
  /* TODO: link */
}

/* ----- sub-block V -------------------------------------------------------- */
static bool stm32l4_uart_set_attributes(rtems_termios_device_context *context,
                                        const struct termios *term) {
  /* TODO: link */
}

/* ===== CONFIGURATION ====================================================== */
/* ----- generate context instances array -----------------------------------
 */
rtems_termios_device_context stm32l4_uart_instances[3] = {
    RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("stm32l4 UART 1"),
    RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("stm32l4 UART 2"),
    RTEMS_TERMIOS_DEVICE_CONTEXT_INITIALIZER("stm32l4 UART 3"),
};

/* ----- construct the device handler structure ----------------------------- */
const rtems_termios_device_handler stm32l4_uart_handler_polled = {
    .first_open = stm32l4_uart_first_open,
    .last_close = stm32l4_uart_last_close, // maybe can be null?
    .poll_read = stm32l4_uart_read_polled,
    .write = stm32l4_uart_write_polled,
    .set_attributes = stm32l4_uart_set_attributes,
    .ioctl = NULL,
    .mode = TERMIOS_POLLED};

/* ===== IMPLEMENTATION ===================================================== */
/* ----- install the termios devices ---------------------------------------- */
rtems_status_code console_initialize(rtems_device_major_number major,
                                     rtems_device_minor_number minor,
                                     void *arg) {

  rtems_termios_initialize();

  size_t i;
  for (i = 0; i < RTEMS_ARRAY_SIZE(stm32l4_uart_instances); ++i) {
    char uart[] = "/dev/ttySX";

    uart[sizeof(uart) - 2] = (char)('0' + i);
    rtems_termios_device_install(&uart[0], &stm32l4_uart_handler_polled, NULL,
                                 &stm32l4_uart_instances[i]);

    /*
     * TODO: See if it can be removed, not sure what it does
    if (i == BSP_CONSOLE_MINOR) {
      link(&uart[0], CONSOLE_DEVICE_NAME);
    }
    */
  }

  return RTEMS_SUCCESSFUL;
}

/* ----- initialize bsp debug console interface ----------------------------- */

static void stm32l4_debug_console_early_init(char c);

static void stm32l4_debug_console_early_init(char c) {
  uart_init(UART2, STM32L4_UART_DEFAULT_BAUD);
  /* TODO: Put back */
  // uart_write_byte(UART2, c);
  /* TEST: just for testing */
  uart_write_buf(UART2, "play victory dance\n\r", 20);
}

/**
 * @ingroup RTEMSAPIKernelCharIO
 *
 * @brief This function pointer references the kernel character output
 *   implementation.
 *
 * This function pointer shall never be NULL.  It shall be provided by the
 * BSP and statically initialized.  The referenced function shall output
 * exactly the character specified by the parameter.  In particular, it
 * shall not perform character translations, for example ``NL`` to ``CR``
 * followed by
 * ``NR``.  The function shall not block.
 */
BSP_output_char_function_type BSP_output_char =
    stm32l4_debug_console_early_init;

/**
 * @ingroup RTEMSAPIKernelCharIO
 *
 * @brief This function pointer may reference the kernel character input
 *   implementation.
 *
 * This function pointer may be NULL.  It may reference a function provided by
 * the BSP.  Referenced functions shall dequeue the least recently received
 * character from the device and return it as an unsigned character.  If no
 * character is enqueued on the device, then the function shall immediately
 * return the value minus one.
 */
BSP_polling_getchar_function_type BSP_poll_char =
    NULL; // TODO: add proper pointer
