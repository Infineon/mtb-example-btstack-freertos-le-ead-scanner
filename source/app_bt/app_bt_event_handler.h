/*******************************************************************************
 * File Name: app_bt_event_handler.h
 *
 * Description: This file is the public interface of app_bt_event_handler.c
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

#ifndef SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_
#define SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define APP_BT_RUN_MODE_EXCHANGE_EAD_KEYS   1
#define APP_BT_RUN_MODE_EAD                 2

#define MAX_NUM_CONNECTIONS   (4)
#define MAX_PERIPHERALS       (1)

/*******************************************************************************
 * Structures
 *******************************************************************************/
typedef struct
{
    uint16_t                     conn_id;        /* connection ID of the peer */
    wiced_bt_device_address_t    remote_addr;    /* remote peer device address */
    uint16_t                     peer_mtu;       /* peer MTU */
    wiced_bt_dev_role_t          link_role;      /* Device role */
    wiced_bt_transport_t         transport;      /* BR/EDR oR LE transport */
    wiced_bool_t                 connected;      /* TRUE - connected, FALSE - not connected */
    wiced_bt_ble_address_type_t  addr_type;      /* Peer device BDA type */
    bool                         are_we_central; /* Flag to indicate if our device is GAP Central/Peripheral */
    bool                         is_read_notify_char_cplt; /* Flags to check if read response */
    bool                         is_read_cccd_cplt;        /* is received before initiating */
    bool                         is_read_blink_char_cplt;  /* a new read request */
    bool                         is_read_ead_keys_cplt;    /* a new read ead keys request */
    bool                         is_read_ead_cccd_cplt;    /* a new read ead cccd keys request */
    bool                         flag_indication_sent;     /* to store the state of indication confirmation */
} hello_client_peer_info_t;
/*******************************************************************************
 * Variable Definitions
 ******************************************************************************/
extern uint8_t adv_device_name[15];
extern hello_client_peer_info_t peer_info[MAX_NUM_CONNECTIONS];
extern uint16_t current_conn_id;
extern uint8_t app_bt_run_mode;
/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
wiced_result_t
app_bt_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data);

void app_bt_application_init(void);

void app_bt_adv_stop_handler(void);
void app_scan_result_handler(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);

#endif /* SOURCE_APP_BT_APP_BT_EVENT_HANDLER_H_ */
