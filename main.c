/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#define MY_APP_LOG "S->MAD_ver_1.0"    //MAD -> My Application Development

#define BUTTON_PRESSED  0

/* GPIO_PRT0_IN, pin 4 Read-Only Register.
 * Software Read-Only, Hardware Write-Only
 * Using const keyword here.
 */
#define GPIO_PORT_0_PININ_4   (uint32_t*)0x40320010

/*GPIO_PRT13_OUT, Keeping pin 7 OUT for port-13 register
 *GPIO_PRT13_CFG, Keeping GPIO strong for pin 7
 *GPIO_PRT13_OUT_SET, setting pin HIGH for pin 7
 *GPIO_PRT13_OUT_CLR, setting pin LOW for pin 7
 */
#define GPIO_PORT_13_PINOUT_7 (uint32_t*)0x40320680
#define GPIO_PORT_13_CONFIG_7 (uint32_t*)0x403206A8
#define GPIO_PORT_13_OUTSET_7 (uint32_t*)0x40320688
#define GPIO_PORT_13_OUTCLR_7 (uint32_t*)0x40320684

/*GPIO_PRT11_OUT, Keeping pin 1 OUT for port-11 register
 *GPIO_PRT11_CFG, Keeping GPIO strong for pin 1
 *GPIO_PRT11_OUT_SET, setting pin HIGH for pin 1
 *GPIO_PRT11_OUT_CLR, setting pin LOW for pin 1
 */
#define GPIO_PORT_11_PINOUT_1 (uint32_t*)0x40320580
#define GPIO_PORT_11_CONFIG_1 (uint32_t*)0x403205A8
#define GPIO_PORT_11_OUTSET_1 (uint32_t*)0x40320588
#define GPIO_PORT_11_OUTCLR_1 (uint32_t*)0x40320584

/*GPIO_PRT0_OUT, Keeping pin 3 OUT for port-0 register
 *GPIO_PRT0_CFG, Keeping GPIO strong for pin 3
 *GPIO_PRT0_OUT_SET, setting pin HIGH for pin 3
 *GPIO_PRT0_OUT_CLR, setting pin LOW for pin 3
 */
#define GPIO_PORT_0_PINOUT_3 (uint32_t*)0x40320000
#define GPIO_PORT_0_CONFIG_3 (uint32_t*)0x40320028
#define GPIO_PORT_0_OUTSET_3 (uint32_t*)0x40320008
#define GPIO_PORT_0_OUTCLR_3 (uint32_t*)0x40320004

/*GPIO_PRT1_OUT, Keeping pin 5 OUT for port-1 register
 *GPIO_PRT1_CFG, Keeping GPIO strong for pin 5
 *GPIO_PRT1_OUT_SET, setting pin HIGH for pin 5
 *GPIO_PRT1_OUT_CLR, setting pin LOW for pin 5
 */
#define GPIO_PORT_1_PINOUT_5 (uint32_t*)0x40320080
#define GPIO_PORT_1_CONFIG_5 (uint32_t*)0x403200A8
#define GPIO_PORT_1_OUTSET_5 (uint32_t*)0x40320088
#define GPIO_PORT_1_OUTCLR_5 (uint32_t*)0x40320084

void Gpio_peripheral_init(void)
{
	
    uint32_t *reg_add_port= GPIO_PORT_1_PINOUT_5;
    uint32_t *reg_add_config= GPIO_PORT_1_CONFIG_5;

    uint32_t *reg_add_port1= GPIO_PORT_0_PINOUT_3;
    uint32_t *reg_add_config1= GPIO_PORT_0_CONFIG_3;

    uint32_t *reg_add_port2= GPIO_PORT_11_PINOUT_1;
    uint32_t *reg_add_config2= GPIO_PORT_11_CONFIG_1;

    uint32_t *reg_add_port3= GPIO_PORT_1_PINOUT_5;
    uint32_t *reg_add_config3= GPIO_PORT_1_CONFIG_5;

    uint32_t *reg_add_port4= GPIO_PORT_13_PINOUT_7;
    uint32_t *reg_add_config4= GPIO_PORT_13_CONFIG_7;

    *reg_add_port    |= (1<<5);
	*reg_add_config  |= (6<<20);

	*reg_add_port1   |= (1<<3);
	*reg_add_config1 |= (6<<12);

	*reg_add_port2   |= (1<<0);
	*reg_add_config2 |= (6<<4);

	*reg_add_port3   |= (1<<1);
	*reg_add_config3 |= (6<<4);

	*reg_add_port4   |= (1<<7);
	*reg_add_config4 |= (6<<28);

	*reg_add_config1 |=(1<<19);


}

void Blink_led(void)
{
	

    uint32_t *reg_add_outset= GPIO_PORT_1_OUTSET_5;
    uint32_t *reg_add_outclr= GPIO_PORT_1_OUTCLR_5;

    uint32_t *reg_add_outset1= GPIO_PORT_0_OUTSET_3;
    uint32_t *reg_add_outclr1= GPIO_PORT_0_OUTCLR_3;

    uint32_t *reg_add_outset2= GPIO_PORT_11_OUTSET_1;
    uint32_t *reg_add_outclr2= GPIO_PORT_11_OUTCLR_1;

    uint32_t *reg_add_outset3= GPIO_PORT_1_OUTSET_5;
    uint32_t *reg_add_outclr3= GPIO_PORT_1_OUTCLR_5;

    uint32_t *reg_add_outset4= GPIO_PORT_13_OUTSET_7;
    uint32_t *reg_add_outclr4= GPIO_PORT_13_OUTCLR_7;

    uint32_t const *reg_add_pinread = GPIO_PORT_0_PININ_4;
    uint32_t const pin_read = *reg_add_pinread;


    if(pin_read == BUTTON_PRESSED)
   {
        *reg_add_outclr |= (1<<5);
	    cyhal_system_delay_ms(100);
	    *reg_add_outset |= (1<<5);
	    cyhal_system_delay_ms(100);

	    *reg_add_outclr1 |= (1<<3);
	     cyhal_system_delay_ms(100);
	    *reg_add_outset1 |= (1<<3);
	     cyhal_system_delay_ms(100);

	    *reg_add_outclr2 |= (1<<1);
	     cyhal_system_delay_ms(100);
	    *reg_add_outset2 |= (1<<1);
	     cyhal_system_delay_ms(100);

	    *reg_add_outclr3 |= (1<<1);
	     cyhal_system_delay_ms(100);
	    *reg_add_outset3 |= (1<<1);
	     cyhal_system_delay_ms(100);

	    *reg_add_outclr4 |= (1<<7);
	     cyhal_system_delay_ms(300);
	    *reg_add_outset4 |= (1<<7);
	     cyhal_system_delay_ms(100);
   }
    else
    {
    	// Keeping pin high to set LED OFF
    	    *reg_add_outset  |= (1<<5);
    	    *reg_add_outset1 |= (1<<3);
    	    *reg_add_outset2 |= (1<<1);
    	    *reg_add_outset3 |= (1<<1);
    	    *reg_add_outset4 |= (1<<7);
    }
}


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

     Gpio_peripheral_init();

    for (;;)
    {

    	Blink_led();
    }
}

/* [] END OF FILE */
