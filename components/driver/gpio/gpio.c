/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/*******************************************************************************
* @file   gpio.c
* @brief  Contains all functions support for gpio and iomux driver
* @version  0.0
* @date   19. Oct. 2017
* @author qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "types.h"
#include "string.h"
#include "ap_cp.h"
#include "clock.h"
#include "pwrmgr.h"
#include "gpio.h"
#include "error.h"
#include "log.h"



enum {
  GPIO_PIN_ASSI_NONE = 0,
  GPIO_PIN_ASSI_OUT,
  GPIO_PIN_ASSI_IN_IRQ,
  GPIO_PIN_ASSI_IN_WAKEUP,
  GPIO_PIN_ASSI_IN_IRQ_AND_WAKEUP,
};

typedef struct {
  bool          enable;
  uint8_t       pin_state;
  gpioin_Hdl_t  posedgeHdl;
  gpioin_Hdl_t  negedgeHdl;
}gpioin_Ctx_t;


typedef struct {
  bool          state;
  uint8_t       pin_assignments[NUMBER_OF_PINS];
  gpioin_Ctx_t  irq_ctx[NUMBER_OF_PINS];

}gpio_Ctx_t;


static gpio_Ctx_t m_gpioCtx = {
  .state = FALSE,
  .pin_assignments = {0,},
};

static void pad_pe_control(GPIO_Pin_e pin, BitAction_e value) {
  uint32_t  bit = BIT(pin & 0x1f);
  if (value) {
    BM_SET(REG_PAD_EN(pin), bit);       //set bit
  }
  else {
    BM_CLR(REG_PAD_EN(pin), bit);       //clear bit
  }
}


//static void pad_ds_control(GPIO_Pin_e pin, BitAction_e value) {
//  uint32_t  bit = BIT(pin & 0x1f);
//  if (value) {
//    BM_SET(REG_PAD_DS(pin), bit);       //set bit
//  }
//  else {
//    BM_CLR(REG_PAD_DS(pin), bit);       //clear bit
//  }
//}


static void io_wakeup_control(GPIO_Pin_e pin, BitAction_e value) {
  uint32_t  bit = BIT(pin & 0x1f);
  if (value) {
    BM_SET(REG_IO_WAKEUP_EN(pin), bit);       //set bit
  }
  else {
    BM_CLR(REG_IO_WAKEUP_EN(pin), bit);       //clear bit
  }
}


static int gpio_interrupt_enable(GPIO_Pin_e pin, IO_Wakeup_Pol_e type)
{
  uint32_t gpio_tmp;
  if (pin >= NUMBER_OF_IRQ_PINS)
    return PPlus_ERR_NOT_SUPPORTED;
  gpio_tmp = *(volatile uint32_t *)0x40008038;
  gpio_tmp |= (1 << pin); //edge sensitive
  *(volatile uint32_t *)0x40008038 = gpio_tmp;

  gpio_tmp = *(volatile uint32_t *)0x40008034;
  gpio_tmp &= ~(1 << pin); //unmask interrupt
  *(volatile uint32_t *)0x40008034 = gpio_tmp;

  gpio_tmp = *(volatile uint32_t *)0x4000803c;
  if (type == POSEDGE)
    gpio_tmp |= (1 << pin);
  else
    gpio_tmp &= ~(1 << pin);
  *(volatile uint32_t *)0x4000803c = gpio_tmp;

  gpio_tmp = *(volatile uint32_t *)0x40008030;
  gpio_tmp |= (1 << pin); //enable interrupt
  *(volatile uint32_t *)0x40008030 = gpio_tmp;
  return PPlus_SUCCESS;
}


static int gpio_interrupt_disable(GPIO_Pin_e pin)
{
  //enable gpio 15 gpio falling edge intteruput
  uint32_t gpio_tmp = *(volatile uint32_t *)0x40008038;
  if (pin >= NUMBER_OF_IRQ_PINS)
    return PPlus_ERR_NOT_SUPPORTED;

  gpio_tmp = *(volatile uint32_t *)0x40008034;
  gpio_tmp |= (1 << pin); //mask interrupt
  *(volatile uint32_t *)0x40008034 = gpio_tmp;


  gpio_tmp = *(volatile uint32_t *)0x40008030;
  gpio_tmp &= ~(1 << pin); //disable interrupt
  *(volatile uint32_t *)0x40008030 = gpio_tmp;
  return PPlus_SUCCESS;
}


static void gpioin_event_pin(GPIO_Pin_e pin, IO_Wakeup_Pol_e type)
{
  gpioin_Ctx_t* p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);

  if (p_irq_ctx[pin].posedgeHdl && (type == POSEDGE)) {
    //LOG("POS\n");
    p_irq_ctx[pin].posedgeHdl(pin,POSEDGE);
  }
  else if (p_irq_ctx[pin].negedgeHdl && (type == NEGEDGE)) {
    //LOG("NEG\n");
    p_irq_ctx[pin].negedgeHdl(pin,NEGEDGE);
  }
}


static void gpioin_wakeup_trigger(GPIO_Pin_e pin)
{
  uint8_t pin_state = (uint8_t)hal_gpio_read(pin);

  IO_Wakeup_Pol_e type = pin_state ? POSEDGE : NEGEDGE;

  if (m_gpioCtx.irq_ctx[pin].pin_state != pin_state)
    gpioin_event_pin(pin, type);

}



static void gpioin_event(uint32 int_status, uint32 polarity)
{
  int i;
  gpioin_Ctx_t* p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);
  //LOG("GI:%x,%x\n",int_status,polarity);
  for (i = 0; i < NUMBER_OF_IRQ_PINS; i++) {
    if (int_status & (1ul << i)) {
      IO_Wakeup_Pol_e type = (polarity & BIT(i)) ? POSEDGE : NEGEDGE;
      gpioin_event_pin((GPIO_Pin_e)i, type);

      //reconfig interrupt
      if (p_irq_ctx[i].posedgeHdl && p_irq_ctx[i].negedgeHdl) //both raise and fall
      {
        type = (type == POSEDGE) ? NEGEDGE : POSEDGE;
        gpio_interrupt_enable((GPIO_Pin_e)i, type);
      }
      else if (p_irq_ctx[i].posedgeHdl) //raise
      {
        gpio_interrupt_enable((GPIO_Pin_e)i, POSEDGE);
      }
      else if (p_irq_ctx[i].negedgeHdl) //fall
      {
        gpio_interrupt_enable((GPIO_Pin_e)i, NEGEDGE);
      }
    }
  }
}


static void gpio_sleep_handler(void)
{
  int i;
  IO_Wakeup_Pol_e pol;
  for (i = 0; i < NUMBER_OF_PINS; i++) {
    //config wakeup
    if (m_gpioCtx.pin_assignments[i] > GPIO_PIN_ASSI_IN_IRQ)
    {
      pol = hal_gpio_read((GPIO_Pin_e)i) ? NEGEDGE : POSEDGE;
      hal_gpio_wakeup_set((GPIO_Pin_e)i, pol);
    }
    //prepare interrupt
    if (i < NUMBER_OF_PINS &&
      (m_gpioCtx.pin_assignments[i] > GPIO_PIN_ASSI_IN_IRQ)
      )
    {
      m_gpioCtx.irq_ctx[i].pin_state = hal_gpio_read((GPIO_Pin_e)i);
    }
  }
}

static void gpio_wakeup_handler(void)
{
  int i;
  NVIC_SetPriority((IRQn_Type)GPIO_IRQ, IRQ_PRIO_HAL);
  NVIC_EnableIRQ((IRQn_Type)GPIO_IRQ);

  for (i = 0; i < NUMBER_OF_PINS; i++) {
    if (m_gpioCtx.irq_ctx[i].enable) {
      //resume gpio irq
      hal_gpioin_enable((GPIO_Pin_e)i);

      //trigger gpio irq manually
      gpioin_wakeup_trigger((GPIO_Pin_e)i);
    }
  }
}



/*static*/ int gpio_pin0to3_pin31to34_control(GPIO_Pin_e pin, uint8_t en) {
  if (pin < 4) {
    if (en) {
      write_reg(0x40003814, read_reg(0x40003814) | BIT(pin));
    }
    else {
      write_reg(0x40003814, read_reg(0x40003814)&(~BIT(pin)));
    }
  }
  else {
    if (en) {
      write_reg(0x40003814, read_reg(0x40003814) | BIT(pin - 18));
    }
    else {
      write_reg(0x40003814, read_reg(0x40003814)&(~BIT(pin - 18)));
    }
  }
  return PPlus_SUCCESS;
}


void __attribute__((used)) hal_GPIO_IRQHandler(void)
{
  uint32 polarity = AP_GPIOA->int_polarity;
  uint32 st = AP_GPIOA->int_status;

  //clear interrupt
  AP_GPIOA->porta_eoi = st;
  gpioin_event(st, polarity);

}



int hal_gpio_write(GPIO_Pin_e pin, uint8_t en) {
  hal_gpio_pin_init(pin, OEN);           //set output enable
  uint32_t  bit;
  if (pin < 18) {
    bit = BIT(pin);
    if (en) {
      BM_SET(reg_gpio_swporta_dr, bit);       //set pin output(set bit)
    }
    else {
      BM_CLR(reg_gpio_swporta_dr, bit);       //set pin input(clear bit)
    }
  }
  else {
    bit = BIT(pin - 18);
    if (en) {
      BM_SET(reg_gpio_swportb_dr, bit);       //set pin output(set bit)
    }
    else {
      BM_CLR(reg_gpio_swportb_dr, bit);       //set pin input(clear bit)
    }
  }
  return PPlus_SUCCESS;
}

void hal_gpio_fast_write(GPIO_Pin_e pin, uint8_t en) {
  uint32_t  bit;
  if (pin < 18) {
    bit = BIT(pin);
    if (en) {
      BM_SET(reg_gpio_swporta_dr, bit);       //set pin output(set bit)
    }
    else {
      BM_CLR(reg_gpio_swporta_dr, bit);       //set pin input(clear bit)
    }
  }
  else {
    bit = BIT(pin - 18);
    if (en) {
      BM_SET(reg_gpio_swportb_dr, bit);       //set pin output(set bit)
    }
    else {
      BM_CLR(reg_gpio_swportb_dr, bit);       //set pin input(clear bit)
    }
  }
}

int hal_gpio_toggle(GPIO_Pin_e pin) {
  hal_gpio_pin_init(pin, OEN);           //set output enable
  uint32_t  bit;
  if (pin < 18) {
    bit = BIT(pin);

      BM_SET(reg_gpio_swporta_dr, bit);       //set pin output(set bit)

      BM_CLR(reg_gpio_swporta_dr, bit);       //set pin input(clear bit)

  }
  else {
    bit = BIT(pin - 18);

      BM_SET(reg_gpio_swportb_dr, bit);       //set pin output(set bit)
  

      BM_CLR(reg_gpio_swportb_dr, bit);       //set pin input(clear bit)

  }
  return PPlus_SUCCESS;
}

bool hal_gpio_read(GPIO_Pin_e pin)
{
  uint32_t r;
  uint32_t bit;
  uint32_t en;
  if (pin < 18) {
    bit = BIT(pin);
    en = BM_IS_SET(reg_gpio_ioe_porta, bit);
    if (en) {
      r = read_reg(reg_gpio_swporta_dr);
    }
    else {
      r = read_reg(reg_gpio_ext_porta);
    }
    r = (r & bit);
  }
  else {
    bit = BIT(pin - 18);
    en = BM_IS_SET(reg_gpio_ioe_portb, bit);
    if (en) {
      r = read_reg(reg_gpio_swportb_dr);
    }
    else {
      r = read_reg(reg_gpio_ext_portb);
    }
    r = (r & bit);
  }

  return r ? TRUE : FALSE;
}



int hal_gpio_pin_init(GPIO_Pin_e pin, GPIO_ioe type) {
  uint32_t bit;

  if ((pin < P4) || ((pin <= P34) && (pin >= P31)))
    gpio_pin0to3_pin31to34_control(pin, 1);

  hal_gpio_fmux(pin, Bit_DISABLE);     //disable fullmux function
  if (pin < 18) {
    bit = BIT(pin);
    if (type) {
      BM_SET(reg_gpio_ioe_porta, bit);       //set pin output(set bit)
    }
    else {
      BM_CLR(reg_gpio_ioe_porta, bit);       //set pin input(clear bit)
    }
  }
  else {
    bit = BIT(pin - 18);
    if (type) {
      BM_SET(reg_gpio_ioe_portb, bit);       //set pin output(set bit)
    }
    else {
      BM_CLR(reg_gpio_ioe_portb, bit);       //set pin input(clear bit)
    }
  }
  return PPlus_SUCCESS;
}


int hal_gpio_cfg_analog_io(GPIO_Pin_e pin, BitAction_e value) {
  uint32_t  bit = BIT(pin - 11);
  if (value) {
    pad_pe_control(pin, Bit_DISABLE);  //pad en disable
    hal_gpio_pull_set(pin, FLOATING);    //set pin pull up/down floating
    BM_SET(REG_ANALOG_IO, bit);       //set bit
  }
  else {
    BM_CLR(REG_ANALOG_IO, bit);       //clear bit
  }
  return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          hal_gpio_DS_control
 *
 * @brief       This function process for enable or disable pad driver strenth
 *
 * input parameters
 *
 * @param       GPIO_Pin_e pin: gpio pin number
 *              BitAction_e value: enable(Bit_ENABLE) or disable(Bit_DISABLE)
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int hal_gpio_DS_control(GPIO_Pin_e pin, BitAction_e value){
	uint32_t	bit = BIT(pin & 0x1f);
	if(value){
		BM_SET(REG_PAD_DS(pin), bit);       //set bit
	}else{
		BM_CLR(REG_PAD_DS(pin), bit);       //clear bit
	}
	return PPlus_SUCCESS;
}

int hal_gpio_fmux(GPIO_Pin_e pin, BitAction_e value)
{
    uint32_t  bit = BIT(pin & 0x1f);
    
    if (value) { 
        BM_SET(REG_FMUX_EN_FUC(pin), bit);
    }
    else {
        BM_CLR(REG_FMUX_EN_FUC(pin), bit);
    }
    return PPlus_SUCCESS;
}

int hal_gpio_fmux_set(GPIO_Pin_e pin, Fmux_Type_e type) {

  hal_gpio_fmux(pin, Bit_ENABLE);      //enable fullmux fuction; enable or disable 

  FMUX_FUNIO_SELECT(pin, type);      //select fmux pin function,the type is spi/uart/iic and so on
  return PPlus_SUCCESS;
}


int hal_gpio_pull_set(GPIO_Pin_e pin, IO_Pull_Type_e type) {
  uint32_t  itype = (uint32_t)type;
  uint8_t   index = (pin % 10) * 3 + 1;
  uint32_t  bit = ~(BIT(index) | BIT(index + 1));
  bit = (*REG_IOPULL_IO(pin)) & bit;
  itype = (itype << index) | bit;
  *REG_IOPULL_IO(pin) = itype;
  return PPlus_SUCCESS;
}

int  hal_gpio_wakeup_set(GPIO_Pin_e pin, IO_Wakeup_Pol_e type) {
  uint8_t   index = (pin % 10) * 3;
  io_wakeup_control(pin, Bit_ENABLE);      //enable wakeup function
  uint32_t  bit = BIT(index);

  if (type) {
    BM_SET(REG_IO_Wakeuo_Pol(pin), bit);       //set bit
  }
  else {
    BM_CLR(REG_IO_Wakeuo_Pol(pin), bit);       //clear bit
  }
  return PPlus_SUCCESS;
}


int hal_gpioin_enable(GPIO_Pin_e pin)
{
  gpioin_Ctx_t* p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);
  IO_Wakeup_Pol_e type = NEGEDGE;
  uint32 pinVal = 0;

  if (p_irq_ctx[pin].posedgeHdl == NULL && p_irq_ctx[pin].negedgeHdl == NULL)
    return PPlus_ERR_NOT_REGISTED;

  if (pin >= NUMBER_OF_IRQ_PINS)
    m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_IN_WAKEUP;
  else
    m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_IN_IRQ_AND_WAKEUP;

  p_irq_ctx[pin].enable = TRUE;

  hal_gpio_pin_init(pin, IE);

  //hal_gpio_pull_set(pin, PULL_DOWN); //??need disccuss
  if (p_irq_ctx[pin].posedgeHdl && p_irq_ctx[pin].negedgeHdl) //both raise and fall
  {
    pinVal = hal_gpio_read(pin);
    type = pinVal ? NEGEDGE : POSEDGE;
    gpio_interrupt_enable(pin, type);
    return PPlus_SUCCESS;
  }
  else if (p_irq_ctx[pin].posedgeHdl) //raise
  {
    type = POSEDGE;
  }
  else if (p_irq_ctx[pin].negedgeHdl) //fall
  {
    type = NEGEDGE;
  }

  gpio_interrupt_enable(pin, type);
  return PPlus_SUCCESS;
}


int hal_gpioin_disable(GPIO_Pin_e pin)
{
  gpioin_Ctx_t* p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);
  if (pin > 17)
    return PPlus_ERR_NOT_SUPPORTED;


  p_irq_ctx[pin].enable = FALSE;
  m_gpioCtx.pin_assignments[pin] = GPIO_PIN_ASSI_NONE;

  hal_gpio_pin_init(pin, IE);

  return gpio_interrupt_disable(pin);
}



int hal_gpioin_register(GPIO_Pin_e pin, gpioin_Hdl_t posedgeHdl, gpioin_Hdl_t negedgeHdl)
{
  int ret;
  gpioin_Ctx_t* p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);

  hal_gpioin_disable(pin);
  p_irq_ctx[pin].posedgeHdl = posedgeHdl;
  p_irq_ctx[pin].negedgeHdl = negedgeHdl;
  ret = hal_gpioin_enable(pin);
  if (ret != PPlus_SUCCESS)
    hal_gpioin_disable(pin);
  return ret;

}

int hal_gpioin_unregister(GPIO_Pin_e pin)
{
  gpioin_Ctx_t* p_irq_ctx = &(m_gpioCtx.irq_ctx[0]);
  if (pin > 17)
    return PPlus_ERR_NOT_SUPPORTED;

  hal_gpioin_disable(pin);
  p_irq_ctx[pin].negedgeHdl = NULL;
  p_irq_ctx[pin].posedgeHdl = NULL;
  return PPlus_SUCCESS;
}



int hal_gpio_init(void)
{
  if (m_gpioCtx.state)
    return PPlus_ERR_INVALID_STATE;

  memset(&m_gpioCtx, 0, sizeof(m_gpioCtx));
  m_gpioCtx.state = TRUE;

  //disable all channel irq,unmask all channel
  AP_GPIOA->inten = 0;
  AP_GPIOA->intmask = 0;

  //disable all wakeup pin
  AP_WAKEUP->io_wu_mask_31_0 = 0;
  AP_WAKEUP->io_wu_mask_34_32 = 0;

  NVIC_SetPriority((IRQn_Type)GPIO_IRQ, IRQ_PRIO_HAL);
  NVIC_EnableIRQ((IRQn_Type)GPIO_IRQ);

  hal_pwrmgr_register(MOD_GPIO, gpio_sleep_handler, gpio_wakeup_handler);

  return PPlus_SUCCESS;
}




void hal_gpio_p00_to_hclk_div8_enable(void)
{
	REG_DMUX_EN_FUC |= (1<<0);
}

void hal_gpio_p00_to_hclk_div8_disable(void)
{
	REG_DMUX_EN_FUC &= ~(1<<0);
}

void hal_gpio_p01_to_pclk_div4_enable(void)
{
	REG_DMUX_EN_FUC |= (1<<1);
}

void hal_gpio_p01_to_pclk_div4_disable(void)
{
	REG_DMUX_EN_FUC &= ~(1<<1);
}

void hal_gpio_p24_to_rc32k_enable(void)
{
	REG_DMUX_EN_FUC |= (1<<6);
}

void hal_gpio_p24_to_rc32k_disable(void)
{
	REG_DMUX_EN_FUC &= ~(1<<6);
}

void hal_gpio_p25_to_xtal_clk32k_enable(void)
{
	REG_DMUX_EN_FUC |= (1<<7);
}

void hal_gpio_p25_to_xtal_clk32k_disable(void)
{
	REG_DMUX_EN_FUC &= ~(1<<7);
}








