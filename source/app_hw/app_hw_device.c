/*******************************************************************************
 * File Name: app_hw_device.c
 *
 * Description: This file contains functions for initialization and usage of
 *              device peripheral GPIO for LED and button. It also
 *              contains FreeRTOS timer implementations.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 ******************************************************************************/

/*******************************************************************************
 * Header Files
 ******************************************************************************/

#include "inttypes.h"
#include <FreeRTOS.h>
#include <task.h>
#include "timers.h"
#include "app_bt_bonding.h"
#include "app_flash_common.h"
#include "cycfg_gap.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_device.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define APP_BTN_PRESS_SHORT_MIN        (50)
#define APP_BTN_PRESS_SHORT_MAX        (250)
#define APP_BTN_PRESS_5S               (5000)
#define APP_BTN_PRESS_10S              (10000)
#define APP_TIMEOUT_MS_BTN             (1)
#define APP_TIMEOUT_LED_INDICATE       (500)

#define MAXIMUM_LED_BLINK_COUNT        (11)
/* Interrupt priority for GPIO connected to button */
#define GPIO_INTERRUPT_PRIORITY         (4)
/* Stack size for button task */
#define BTN_TASK_STACK_SIZE             (512u)
/* Task Priority button Task */
#define BTN_TASK_PRIORITY               (2)

/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/

/* This timer is used to toggle LED to indicate button press type */
TimerHandle_t ms_timer_led_indicate;

/* ms_timer_btn is a periodic timer that ticks every millisecond.
 * This timer is used to measure the duration of button press events */
TimerHandle_t ms_timer_btn;

/* Handle of the button task */
TaskHandle_t button_handle;

/* Variable to keep track of LED blink count for long button press */
static uint8_t led_indicate_count;

/* Variable to hold the duration in milli second for button press */
static uint32_t timer_count_ms;

/* Variable used to determine if button is pressed or not */
static bool is_btn_pressed;

/* The time stamp at which button is pressed */
static uint32_t btn_press_start;

/* To check if the device has entered pairing mode to connect and bond with a new device */
bool pairing_mode = FALSE;

/* For button press interrupt */
cyhal_gpio_callback_data_t btn_cb_data =
{
    .callback     = app_bt_gpio_interrupt_handler,
    .callback_arg = NULL
};


/**
 * Function Name: app_bt_timeout_ms_btn
 *
 * Function Description:
 *   @brief The function invoked on timeout of FreeRTOS millisecond timer. It is
 *          used to print logs over the serial terminal every 10 seconds and to
 *          keep store time elapsed required to calculate button press duration.
 *
 *   @param Timerhandle_t timer_handle: unused
 *
 *   @return None
 *
 */
void app_bt_timeout_ms_btn(TimerHandle_t timer_handle)
{
    timer_count_ms++;
    if((APP_BTN_PRESS_5S == (timer_count_ms - btn_press_start)) && (is_btn_pressed))
    {
#ifndef CYW89829_BLE
        /* Start LED blink indicate for 5 more seconds */
        cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_ON);
#endif
        xTimerStart(ms_timer_led_indicate, 0);
    }
}

/**
 * Function Name: app_bt_timeout_led_indicate
 *
 * Function Description:
 *   @brief The function is invoked on timeout of FreeRTOS milliseconds timer.
 *          It is used to toggle the LED used for indication of button press
 *          duration. The LED will blink for 10 seconds to show that the button
 *          can be released to clear the bond data.
 *
 *   @param Timerhandle_t timer_handle: unused
 *
 *   @return None
 *
 */
void app_bt_timeout_led_indicate(TimerHandle_t timer_handle)
{
    led_indicate_count++;
#ifndef CYW89829_BLE
    cyhal_gpio_toggle(CYBSP_USER_LED2);
#endif
    if(led_indicate_count == MAXIMUM_LED_BLINK_COUNT)
    {
        xTimerStop(ms_timer_led_indicate, 0);
#ifndef CYW89829_BLE
        cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_OFF);
#endif
        led_indicate_count = 0;
    }
}

/**
 * Function Name: app_bt_interrupt_config
 *
 * Function Description:
 *   @brief This function initializes a pin as input that triggers interrupt on
 *   falling edges.
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_interrupt_config(void)
{
    cy_rslt_t result=0;

    /* Initialize the user button */
    result = cyhal_gpio_init(CYBSP_USER_BTN,
                             CYHAL_GPIO_DIR_INPUT,
                             CYHAL_GPIO_DRIVE_PULLDOWN,
                             CYBSP_BTN_OFF);
    if(CY_RSLT_SUCCESS != result)
    {
        printf("Button GPIO initialization failed");
        CY_ASSERT(0);
    }

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &btn_cb_data);

    cyhal_gpio_enable_event(CYBSP_USER_BTN,
                            CYHAL_GPIO_IRQ_BOTH,
                            GPIO_INTERRUPT_PRIORITY,
                            true);
    UNUSED_VARIABLE(result);
}

/**
 * Function Name: app_bt_gpio_interrupt_handler
 *
 * Function Description:
 *   @brief GPIO interrupt handler.
 *
 *   @param void *handler_arg (unused)
 *   @param cyhal_gpio_irq_event_t (unused)
 *
 *   @return None
 *
 */
void app_bt_gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(CYHAL_GPIO_IRQ_FALL == event)
    {
        xTimerStartFromISR(ms_timer_btn, &xHigherPriorityTaskWoken);
    }

    vTaskNotifyGiveFromISR(button_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

/**
 * Function Name: app_bt_hw_init
 *
 * Function Description:
 *   @brief This function initializes the LEDs and FreeRTOS Timers.
 *
 *   @param None
 *
 *   @return None
 *
 */

void app_bt_hw_init()
{
    cyhal_gpio_init(CYBSP_USER_LED1 , CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
#ifndef CYW89829_BLE
    cyhal_gpio_init(CYBSP_USER_LED2 , CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
#endif
    app_bt_interrupt_config();

    /* Starting a log print timer for button press duration
     * and application traces  */
    ms_timer_btn = xTimerCreate("ms_timer_btn",
                            pdMS_TO_TICKS(APP_TIMEOUT_MS_BTN),
                            pdTRUE,
                            NULL,
                            app_bt_timeout_ms_btn);
    if(NULL == ms_timer_btn)
    {
        printf("Failed to create FreeRTOS milli second timer! \n");
        CY_ASSERT(0);
    }
    else
    {
        xTimerStart(ms_timer_btn, 0);
    }

    /* Starting a 500 ms timer for indication LED used to show button press duration */
    ms_timer_led_indicate = xTimerCreate("ms_timer_led_indicate",
                                          pdMS_TO_TICKS(APP_TIMEOUT_LED_INDICATE),
                                          pdTRUE,
                                          NULL,
                                          app_bt_timeout_led_indicate);
    if(NULL == ms_timer_led_indicate)
    {
        printf("Failed to create FreeRTOS 500 ms timer! \n");
        CY_ASSERT(0);
    }

    if( pdPASS != xTaskCreate(button_task, "Button task", BTN_TASK_STACK_SIZE,
                              NULL, BTN_TASK_PRIORITY, &button_handle))
    {
        printf("Failed to create Button task!\n");
        CY_ASSERT(0);
    }
}

/**
 * Function Name: button_task
 *
 * Function Description:
 *   @brief This is a FreeRTOS task that handles the button press events.
 *
 *   @param None
 *
 *   @return None
 *
 */
void button_task(void *arg)
{
    static uint32_t btn_press_duration = 0;
    cy_rslt_t rslt = CY_RSLT_SUCCESS;
    wiced_result_t result = WICED_BT_SUCCESS;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Check if button is pressed */
        if(CYBSP_BTN_PRESSED == cyhal_gpio_read(CYBSP_USER_BTN))
        {
            /* Update the flag to indicate button is pressed
             * This is used to determine the indication LED pattern
             * corresponding to button press time*/
            is_btn_pressed = TRUE;

            /* Store the time stamp at which button is pressed */
            btn_press_start = timer_count_ms;

            /* Turn ON the LED, it is decided based on time and button state
             * whether to turn it off, blink or keep it ON */
#ifndef CYW89829_BLE
            cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_ON);
#endif
        }

        /* Check if button is released */
        else if(0 != btn_press_start)
        {
            /* Update the flag to indicate button is released */
            is_btn_pressed = FALSE;
#ifndef CYW89829_BLE
            cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_OFF);
#endif
            /* Calculate the time duration for which the button was pressed */
            btn_press_duration = timer_count_ms - btn_press_start;

            if(app_bt_run_mode == APP_BT_RUN_MODE_EXCHANGE_EAD_KEYS)
            {
                /* Check if button press is short and start scanning*/
                if((btn_press_duration > APP_BTN_PRESS_SHORT_MIN) &&
                (btn_press_duration <= APP_BTN_PRESS_SHORT_MAX))
                {
                    printf("Short button press is detected \n");
                    /* Turn off the LED since it is a short button press */
#ifndef CYW89829_BLE
                    cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_OFF);
#endif
                    /* Stop the advertisements before starting to scan */
                    if(BTM_BLE_ADVERT_OFF != wiced_bt_ble_get_current_advert_mode())
                    {
                        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
                        if(WICED_BT_SUCCESS != result)
                        {
                            printf("Failed to stop advertisement, result = %x\n", result);
                        }
                    }
                    if (MAX_PERIPHERALS > num_peripherals)
                    {
                        /* Start scanning upon short button press */
                        result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY,
                                                    true, app_scan_result_handler);

                        /* Failed to start scanning. Stop program execution */
                        if ((WICED_BT_PENDING != result) && (WICED_BT_BUSY != result))
                        {
                            printf("Failed to start scanning!, result = %d \n",
                                    result);
                            CY_ASSERT(0);
                        }
                    }
                    else
                    {
                        printf("Maximum limit for number of peripherals reached!\r\n");
                    }
                }
                /* Check if button is pressed for 5 seconds and enter pairing mode wherein the device can connect
                and bond to a new peer device */
                else if((btn_press_duration > APP_BTN_PRESS_5S) && (btn_press_duration < APP_BTN_PRESS_10S))
                {
                    printf("Entering Pairing Mode: Connect, Pair and Bond with a new peer device...\n");
#ifdef PSOC6_BLE
                    if(BTM_BLE_SCAN_MODE_ACTIVE == cy_bt_cfg_scan_settings.scan_mode)
                    {
                        pairing_mode = TRUE;
                        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF,
                                                            0,
                                                            NULL);
                        /* Refer to Note 1 in Document History section of Readme.md */
                        result = wiced_bt_ble_address_resolution_list_clear_and_disable();
                        if(WICED_BT_SUCCESS == result)
                        {
                            printf("Address resolution list cleared successfully \n");
                        }
                        else
                        {
                            printf("Failed to clear address resolution list \n");
                        }
                        /* Start scanning after clearing address resolution list */
                        result = wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, true,
                                                app_scan_result_handler);

                        /* Failed to start scanning. Stop program execution */
                        if ((WICED_BT_PENDING != result) && (WICED_BT_BUSY != result))
                        {
                            printf("Failed to start scanning!, result = %d \n", result);
                            CY_ASSERT(0);
                        }
                    }
#endif
                    /* Stop the LED indication if user presses the button for more than 5 seconds
                    but releases before 10 seconds */
                    xTimerStop(ms_timer_led_indicate, 0);
                    /* Reset the number of blinks to 0 */
                    led_indicate_count = 0;
                    /* Turn off the LED */
#ifndef CYW89829_BLE
                    cyhal_gpio_write(CYBSP_USER_LED2 , CYBSP_LED_STATE_OFF);
#endif
                }
            }

            /* Check if button is pressed for 10 seconds and delete the bond info from NVRAM */
            if(btn_press_duration > APP_BTN_PRESS_10S)
            {
                printf("Button pressed for more than 10 seconds,"
                       "attempting to clear bond info\n");
                /* Reset Kv-store library, this will clear the flash */
                rslt = mtb_kvstore_reset(&kvstore_obj);
                if(CY_RSLT_SUCCESS == rslt)
                {
                    printf("Successfully reset kv-store library,"
                            "Please reset the device to generate new keys!\n");
                }
                else
                {
                    printf("failed to reset kv-store libray\n");
                }

                /* Clear peer link keys, identity keys structure and ead key material */
                memset(&bond_info, 0, sizeof(bond_info));
                memset(&identity_keys, 0, sizeof(identity_keys));
                memset(&ble_ead_key_material, 0, sizeof(ble_ead_key_material));

                /* Reboot */
                Cy_SysPm_TriggerSoftReset();

            }
            xTimerStop(ms_timer_btn, 0);
        }
    }
}
