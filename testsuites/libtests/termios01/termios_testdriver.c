/*
 *  This file contains a test fixture termios device driver
 *
 *  COPYRIGHT (c) 1989-2009.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#include "tmacros.h"
#include <rtems/libio.h>
#include <stdlib.h>
#include <assert.h>
#include <termios.h>
#include "termios_testdriver.h"

int termios_test_driver_inbyte_nonblocking( int port )
{
  return -1;
}

void termios_test_driver_outbyte_polled(
  int  port,
  char ch
)
{
}

int termios_test_driver_write_support (int minor, const char *buf, int len)
{
  int nwrite = 0;

  while (nwrite < len) {
#if (TERMIOS_TEST_DRIVER_USE_INTERRUPTS)
    termios_test_driver_outbyte_interrupt( minor, *buf++ );
#else
    termios_test_driver_outbyte_polled( minor, *buf++ );
#endif
    nwrite++;
  }
  return nwrite;
}


/*
 *  Set Attributes Handler
 */
int termios_test_driver_set_attributes(
  int                   minor,
  const struct termios *t
)
{
  uint32_t               ulBaudDivisor;
  int                    baud_requested;
  int                    number;
  rtems_interrupt_level  Irql;
  const char            *parity = "NONE";
  const char            *char_size = "5";
  const char            *stop = "NONE";

  baud_requested = t->c_cflag & CBAUD;

  number = rtems_termios_baud_to_number( baud_requested );

  /*
   *  Parity
   */
  if (t->c_cflag & PARENB) {
    parity = "EVEN";
    if (!(t->c_cflag & PARODD))
      parity = "ODD";
  }

  /*
   *  Character Size
   */
  if (t->c_cflag & CSIZE) {
    switch (t->c_cflag & CSIZE) {
      case CS5:  char_size = "5"; break;
      case CS6:  char_size = "6"; break;
      case CS7:  char_size = "7"; break;
      case CS8:  char_size = "8"; break;
    }
  }

  /*
   *  Stop Bits
   */
  if (t->c_cflag & CSTOPB)
    stop = "2";
  else
    stop = "1";

  printf(
    "set_attributes - B%d %s-%s-%s\n",
    number,
    char_size,
    parity,
    stop
  );
  return 0;
}

/*
 *  Test Device Driver Entry Points
 */
rtems_device_driver termios_test_driver_initialize(
  rtems_device_major_number  major,
  rtems_device_minor_number  minor,
  void                      *arg
)
{
  rtems_status_code sc;

  rtems_termios_initialize();

  /*
   *  Register Device Names
   */
  puts(
    "Termios_test_driver - rtems_io_register "
      TERMIOS_TEST_DRIVER_DEVICE_NAME " - OK"
  );
  sc = rtems_io_register_name( TERMIOS_TEST_DRIVER_DEVICE_NAME, major, 0 );
  directive_failed( sc, "rtems_io_register_name" );

  return RTEMS_SUCCESSFUL;
}

rtems_device_driver termios_test_driver_open(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  rtems_status_code sc;
  int               rc;
  rtems_libio_open_close_args_t *args = arg;
  static const rtems_termios_callbacks Callbacks = {
    NULL,                                    /* firstOpen */
    NULL,                                    /* lastClose */
    termios_test_driver_inbyte_nonblocking,  /* pollRead */
    termios_test_driver_write_support,       /* write */
    termios_test_driver_set_attributes,      /* setAttributes */
    NULL,                                    /* stopRemoteTx */
    NULL,                                    /* startRemoteTx */
    0                                        /* outputUsesInterrupts */
  };

  if ( minor > 2 ) {
    puts( "ERROR - Termios_testdriver - only 1 minor supported" );
    rtems_test_exit(0);
  }

  sc = rtems_termios_open (major, minor, arg, &Callbacks);
  directive_failed( sc, "rtems_termios_open" );

  puts( "Termios_test_driver - rtems_set_initial_baud - bad baud - OK" );
  rc = rtems_termios_set_initial_baud( args->iop->data1, 5000 );
  if ( rc != -1 ) {
    printf( "ERROR - return %d\n", rc );
    rtems_test_exit(0);
  }

  puts( "Termios_test_driver - rtems_set_initial_baud - 38400 - OK" );
  rc = rtems_termios_set_initial_baud( args->iop->data1, 38400 );
  if ( rc ) {
    printf( "ERROR - return %d\n", rc );
    rtems_test_exit(0);
  }

  return RTEMS_SUCCESSFUL;
}

rtems_device_driver termios_test_driver_close(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  return rtems_termios_close (arg);
}

rtems_device_driver termios_test_driver_read(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  return rtems_termios_read (arg);
}

rtems_device_driver termios_test_driver_write(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  return rtems_termios_write (arg);
}

rtems_device_driver termios_test_driver_control(
  rtems_device_major_number major,
  rtems_device_minor_number minor,
  void                    * arg
)
{
  return rtems_termios_ioctl (arg);
}
