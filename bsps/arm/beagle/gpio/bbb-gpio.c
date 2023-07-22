/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief Support for the BeagleBone Black.
 */

/**
 * Copyright (c) 2015 Ketul Shah <ketulshah1993 at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

/* BSP specific function definitions for BeagleBone Black.
 * It is totally beased on Generic GPIO API definition. 
 * For more details related to GPIO API please have a 
 * look at libbbsp/shared/include/gpio.h
 */

#include <bsp/beagleboneblack.h>
#include <bsp/irq-generic.h>
#include <bsp/gpio.h>
#include <bsp/bbb-gpio.h>
#include <libcpu/am335x.h>

#include <assert.h>
#include <stdlib.h>

/* Currently these definitions are for BeagleBone Black board only
 * Later on Beagle-xM board support can be added in this code.
 * After support gets added if condition should be removed
 */
#if IS_AM335X
/* 
struct that helps store the values from gpio fdt 
offset - offset from the base address
base - base address of GPIO register
irq - interrupt vector numbers associated with each gpio bank
intr - type of interrupt associatec with the GPIO pin
mode - mode of the GPIO pin
*/
typedef struct bbb_gpio
{
  int offset;
  uint32_t base;
  rtems_vector_number irq;
  rtems_gpio_interrupt intr;
  int mode;
  char * property;

}bbb_gpio;
static const uint32_t gpio_bank_addrs[] = 
  { AM335X_GPIO0_BASE,
  	AM335X_GPIO1_BASE, 
  	AM335X_GPIO2_BASE, 
  	AM335X_GPIO3_BASE };

static const rtems_vector_number gpio_bank_vector[] =
  { AM335X_INT_GPIOINT0A,
  	AM335X_INT_GPIOINT1A,
  	AM335X_INT_GPIOINT2A,
  	AM335X_INT_GPIOINT3A };

/* Get the value of Base Register + Offset */
uint32_t static inline bbb_reg(uint32_t bank, uint32_t reg)
{
  return (gpio_bank_addrs[bank] + reg);
}

static rtems_status_code bbb_select_pin_function(
  uint32_t bank,
  uint32_t pin,
  uint32_t type
) {

  if ( type == BBB_DIGITAL_IN ) {
    mmio_set(bbb_reg(bank, AM335X_GPIO_OE), BIT(pin));
  } else {
    mmio_clear(bbb_reg(bank, AM335X_GPIO_OE), BIT(pin));
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_multi_set(uint32_t bank, uint32_t bitmask)
{
  mmio_set(bbb_reg(bank, AM335X_GPIO_SETDATAOUT), bitmask);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_multi_clear(uint32_t bank, uint32_t bitmask)
{
  mmio_set(bbb_reg(bank, AM335X_GPIO_CLEARDATAOUT), bitmask);

  return RTEMS_SUCCESSFUL;
}

uint32_t rtems_gpio_bsp_multi_read(uint32_t bank, uint32_t bitmask)
{
  return (bbb_reg(bank, AM335X_GPIO_DATAIN) & bitmask);
}

rtems_status_code rtems_gpio_bsp_set(uint32_t bank, uint32_t pin)
{
  mmio_set(bbb_reg(bank, AM335X_GPIO_SETDATAOUT), BIT(pin));

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_clear(uint32_t bank, uint32_t pin)
{
  mmio_set(bbb_reg(bank, AM335X_GPIO_CLEARDATAOUT), BIT(pin));

  return RTEMS_SUCCESSFUL;
}

uint32_t rtems_gpio_bsp_get_value(uint32_t bank, uint32_t pin)
{
  return (mmio_read(bbb_reg(bank, AM335X_GPIO_DATAIN)) & BIT(pin));
}

rtems_status_code rtems_gpio_bsp_select_input(
  uint32_t bank,
  uint32_t pin,
  void *bsp_specific
) {
  return bbb_select_pin_function(bank, pin, BBB_DIGITAL_IN);
}

rtems_status_code rtems_gpio_bsp_select_output(
  uint32_t bank,
  uint32_t pin,
  void *bsp_specific
) {
  return bbb_select_pin_function(bank, pin, BBB_DIGITAL_OUT);
}

rtems_status_code rtems_gpio_bsp_select_specific_io(
  uint32_t bank,
  uint32_t pin,
  uint32_t function,
  void *pin_data
) {
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_set_resistor_mode(
  uint32_t bank,
  uint32_t pin,
  rtems_gpio_pull_mode mode
) {
  /* TODO: Add support for setting up resistor mode */
  return RTEMS_NOT_DEFINED;
}

rtems_vector_number rtems_gpio_bsp_get_vector(uint32_t bank)
{
  return gpio_bank_vector[bank];
}

uint32_t rtems_gpio_bsp_interrupt_line(rtems_vector_number vector)
{
  uint32_t event_status;
  uint8_t bank_nr = 0;

  /* Following loop will get the bank number from vector number */
  while (bank_nr < GPIO_BANK_COUNT && vector != gpio_bank_vector[bank_nr])
  {
  	bank_nr++;
  }

  /* Retrieve the interrupt event status. */
  event_status = mmio_read(bbb_reg(bank_nr, AM335X_GPIO_IRQSTATUS_0));

  /* Clear the interrupt line. */
  mmio_write(
    (bbb_reg(bank_nr, AM335X_GPIO_IRQSTATUS_0)), event_status);
  
  return event_status;
}

rtems_status_code rtems_gpio_bsp_enable_interrupt(
  uint32_t bank,
  uint32_t pin,
  rtems_gpio_interrupt interrupt
) {
  
  /* Enable IRQ generation for the specific pin */
  mmio_set(bbb_reg(bank, AM335X_GPIO_IRQSTATUS_SET_0), BIT(pin));
  
  switch ( interrupt ) {
    case FALLING_EDGE:
      /* Enables asynchronous falling edge detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_FALLINGDETECT), BIT(pin));
      break;
    case RISING_EDGE:
      /* Enables asynchronous rising edge detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_RISINGDETECT), BIT(pin));
      break;
    case BOTH_EDGES:
      /* Enables asynchronous falling edge detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_FALLINGDETECT), BIT(pin));

      /* Enables asynchronous rising edge detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_RISINGDETECT), BIT(pin));
      break;
    case LOW_LEVEL:
      /* Enables pin low level detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_LEVELDETECT0), BIT(pin));
      break;
    case HIGH_LEVEL:
       /* Enables pin high level detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_LEVELDETECT1), BIT(pin));
      break;
    case BOTH_LEVELS:
      /* Enables pin low level detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_LEVELDETECT0), BIT(pin));

      /* Enables pin high level detection. */
      mmio_set(bbb_reg(bank, AM335X_GPIO_LEVELDETECT1), BIT(pin));
      break;
    case NONE:
    default:
      return RTEMS_UNSATISFIED;
  }

  /* The detection starts after 5 clock cycles as per AM335X TRM
   * This period is required to clean the synchronization edge/
   * level detection pipeline
   */
  asm volatile("nop"); asm volatile("nop"); asm volatile("nop");
  asm volatile("nop"); asm volatile("nop");
  
  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_disable_interrupt(
  uint32_t bank,
  uint32_t pin,
  rtems_gpio_interrupt interrupt
) {
  /* Clear IRQ generation for the specific pin */
  mmio_write(bbb_reg(bank, AM335X_GPIO_IRQSTATUS_CLR_0), BIT(pin));

  switch ( interrupt ) {
    case FALLING_EDGE:
      /* Disables asynchronous falling edge detection. */
      mmio_clear(bbb_reg(bank, AM335X_GPIO_FALLINGDETECT), BIT(pin));
      break;
    case RISING_EDGE:
      /* Disables asynchronous rising edge detection. */
      mmio_clear(bbb_reg(bank, AM335X_GPIO_RISINGDETECT), BIT(pin));
      break;
    case BOTH_EDGES:
      /* Disables asynchronous falling edge detection. */
      mmio_clear(bbb_reg(bank, AM335X_GPIO_FALLINGDETECT), BIT(pin));

      /* Disables asynchronous rising edge detection. */
      mmio_clear(bbb_reg(bank, AM335X_GPIO_RISINGDETECT), BIT(pin));
      break;
    case LOW_LEVEL:
      /* Disables pin low level detection. */
      mmio_clear(bbb_reg(bank, AM335X_GPIO_LEVELDETECT0), BIT(pin));
      break;
    case HIGH_LEVEL:
      /* Disables pin high level detection. */
       mmio_clear(bbb_reg(bank, AM335X_GPIO_LEVELDETECT1), BIT(pin));
      break;
    case BOTH_LEVELS:
      /* Disables pin low level detection. */
      mmio_clear(bbb_reg(bank, AM335X_GPIO_LEVELDETECT0), BIT(pin));

      /* Disables pin high level detection. */
      mmio_clear(bbb_reg(bank, AM335X_GPIO_LEVELDETECT1), BIT(pin));
      break;
    case NONE:
    default:
      return RTEMS_UNSATISFIED;
  }

  /* The detection starts after 5 clock cycles as per AM335X TRM
   * This period is required to clean the synchronization edge/
   * level detection pipeline
   */
  asm volatile("nop"); asm volatile("nop"); asm volatile("nop");
  asm volatile("nop"); asm volatile("nop");

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_multi_select(
  rtems_gpio_multiple_pin_select *pins,
  uint32_t pin_count,
  uint32_t select_bank
) {
  uint32_t register_address;
  uint32_t select_register;
  uint8_t i;

  register_address = gpio_bank_addrs[select_bank] + AM335X_GPIO_OE;

  select_register = REG(register_address);

  for ( i = 0; i < pin_count; ++i ) {
    if ( pins[i].function == DIGITAL_INPUT ) {
      select_register |= BIT(pins[i].pin_number);
    } else if ( pins[i].function == DIGITAL_OUTPUT ) {
      select_register &= ~BIT(pins[i].pin_number);
    } else { /* BSP_SPECIFIC function. */
      return RTEMS_NOT_DEFINED;
    }
  }

  REG(register_address) = select_register;

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_bsp_specific_group_operation(
  uint32_t bank,
  uint32_t *pins,
  uint32_t pin_count,
  void *arg
) {
  return RTEMS_NOT_DEFINED;
}
// function used to initalize the fdt (gpio)
/*
@param
* node -> node of the device tree
* pin -> the bbb_gpio struct in which the values from fdt will be stored
* node_offset -> the offest in each bank
* prop -> property associated with the node
* intr_mode -> type of interrupt associatec with the GPIO pin
* pin_mode -> mode of the GPIO pin
*/
rtems_status_code gpio_init_from_property(
  phandle_t node,
  struct bbb_gpio *pin, 
  int node_offset,
  const char *prop,
  rtems_gpio_interrupt intr_mode,
  int pin_mode
)
{
  rtems_status_code   status = RTEMS_SUCCESSFUL;

  const char *p;
  int err;
  char * gpio_bank;
  int gpio_bank_no;
  char * gpio;
  int irq_no;
  
  pin -> offset = node_offset;
  pin -> intr = intr_mode;
  err = rtems_ofw_get_prop(node, prop, (void *)p, sizeof(p));
  if(err<0 || err > MAX_PROPERTY_LEN){
   status = RTEMS_UNSATISFIED;
  }
  pin -> property = p;
  err = rtems_ofw_get_prop(node,"ti,hwmods",(void *)gpio_bank,sizeof(gpio_bank) );
  if(err<0 || err > 5){
   status = RTEMS_UNSATISFIED;
  }
  if(strcmp(gpio_bank,"gpio0")){
    pin -> base = AM335X_GPIO0_BASE;
  }
  else if (strcmp(gpio_bank,"gpio1")){
    pin -> base = AM335X_GPIO1_BASE;
  }
  else if (strcmp(gpio_bank,"gpio2")){
    pin -> base = AM335X_GPIO2_BASE;
  }
  else if (strcmp(gpio_bank,"gpio3")){
    pin -> base = AM335X_GPIO3_BASE;
  }
  else{
    status = RTEMS_UNSATISFIED;
  }
   
  err = rtems_ofw_get_prop(node,"interrupts", (void *)irq_no, sizeof(irq_no));
  if(err<0 ){
   status = RTEMS_UNSATISFIED;
  }
  

  status = rtems_gpio_bsp_enable_interrupt(pin -> base , pin -> offset , pin -> intr );
  if(pin_mode == 1){
    pin -> mode = pin_mode;
    bbb_select_pin_function(pin -> base, pin -> offset, BBB_DIGITAL_IN);
  }
  else if(pin_mode == 2){
    pin -> mode = pin_mode;
    bbb_select_pin_function(pin -> base, pin -> offset, BBB_DIGITAL_OUT);
  }
  else{
    status = RTEMS_UNSATISFIED;
  } 
  
  return status;
  
}
void beagle_gpio_init(phandle_t node)
{
  int                   err;
  rtems_vector_number   irq;
  rtems_ofw_memory_area reg;
  bbb_gpio *gpio;
  rtems_status_code sc;
 

  if (!rtems_ofw_is_node_compatible(node, "ti,omap4-gpio"))
    /* We cannot handle this device */
    printk("Device is not compatable");
    return ;


  err = rtems_ofw_get_interrupts(node, &irq, sizeof(irq));
  if (err < 1) {
    printk("gpio: cannot register device, irq missing in device tree\n");
    return ;
  }

  err = rtems_ofw_get_reg(node, &reg, sizeof(reg));
  if (err <= 0) {
    printk("gpio: cannot register device, regs field missing\n");
    return ;
  }
  // sc = gpio_init_from_property(node,9,"gpios",RISING_EDGE,BBB_DIGITAL_IN)
 
}

#endif /* IS_AM335X */

/* For support of BeagleboardxM */
#if IS_DM3730

/* Currently this section is just to satisfy
 * GPIO API and to make the build successful.
 * Later on support can be added here.
 */

rtems_status_code rtems_gpio_bsp_multi_set(uint32_t bank, uint32_t bitmask)
{
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_multi_clear(uint32_t bank, uint32_t bitmask)
{
  return RTEMS_NOT_DEFINED;
}

uint32_t rtems_gpio_bsp_multi_read(uint32_t bank, uint32_t bitmask)
{
  return -1;
}

rtems_status_code rtems_gpio_bsp_set(uint32_t bank, uint32_t pin)
{
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_clear(uint32_t bank, uint32_t pin)
{
  return RTEMS_NOT_DEFINED;
}

uint32_t rtems_gpio_bsp_get_value(uint32_t bank, uint32_t pin)
{
  return -1;
}

rtems_status_code rtems_gpio_bsp_select_input(
  uint32_t bank,
  uint32_t pin,
  void *bsp_specific
) {
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_select_output(
  uint32_t bank,
  uint32_t pin,
  void *bsp_specific
) {
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_select_specific_io(
  uint32_t bank,
  uint32_t pin,
  uint32_t function,
  void *pin_data
) {
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_set_resistor_mode(
  uint32_t bank,
  uint32_t pin,
  rtems_gpio_pull_mode mode
) {
  return RTEMS_NOT_DEFINED;
}

rtems_vector_number rtems_gpio_bsp_get_vector(uint32_t bank)
{
  return -1;
}

uint32_t rtems_gpio_bsp_interrupt_line(rtems_vector_number vector)
{
  return -1;
}

rtems_status_code rtems_gpio_bsp_enable_interrupt(
  uint32_t bank,
  uint32_t pin,
  rtems_gpio_interrupt interrupt
) {
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_disable_interrupt(
  uint32_t bank,
  uint32_t pin,
  rtems_gpio_interrupt interrupt
) {
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_multi_select(
  rtems_gpio_multiple_pin_select *pins,
  uint32_t pin_count,
  uint32_t select_bank
) {
  return RTEMS_NOT_DEFINED;
}

rtems_status_code rtems_gpio_bsp_specific_group_operation(
  uint32_t bank,
  uint32_t *pins,
  uint32_t pin_count,
  void *arg
) {
  return RTEMS_NOT_DEFINED;
}

#endif /* IS_DM3730 */
