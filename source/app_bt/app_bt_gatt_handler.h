/*******************************************************************************
 * File Name: app_bt_gatt_handler.h
 *
 * Description: This file is the public interface of app_bt_gatt_handler.c
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

#ifndef SOURCE_APP_BT_APP_BT_GATT_HANDLER_H_
#define SOURCE_APP_BT_APP_BT_GATT_HANDLER_H_


/*******************************************************************************
* Header Files
*******************************************************************************/

#include "wiced_bt_gatt.h"
#include "cycfg_gatt_db.h"

/*******************************************************************************
*        Macros
*******************************************************************************/
#define NOTIFY      (0x01)
#define INDICATE    (0x02)
/*******************************************************************************
*        Structures
*******************************************************************************/
typedef struct
{
    uint16_t hs_start_handle;
    uint16_t hs_end_handle;
    uint16_t hs_char_notify_handle;
    uint16_t hs_char_val_notify_handle;
    uint16_t hs_char_blink_handle;
    uint16_t hs_char_val_blink_handle;
    uint16_t hs_cccd_handle;
    uint16_t hs_char_ead_key_handle;
    uint16_t hs_char_val_ead_key_handle;
    uint16_t hs_ead_cccd_handle;
    uint8_t service_found;
} hs_discovery_data_t;

typedef struct
{
    uint16_t   conn_id;          /* Connection ID of the peer */
    uint16_t   service_handle;  /* The service handle associated to the connection
                                   This helps to identify the specific GATT server */
    bool       is_service_free; /* To identify if hello sensor service is free in local device */
}local_gatt_db_info_t;

extern uint8_t uuid_service_hello_sensor[MAX_UUID_SIZE];
/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

wiced_bt_gatt_status_t
app_bt_gatt_callback(wiced_bt_gatt_evt_t event,
                     wiced_bt_gatt_event_data_t *p_data);
wiced_bt_gatt_status_t
app_bt_gatt_req_handler(wiced_bt_gatt_attribute_request_t *p_attr_req);
wiced_bt_gatt_status_t
app_bt_gatt_conn_status_handler(wiced_bt_gatt_connection_status_t *p_conn_status);
wiced_bt_gatt_status_t
app_bt_gatt_req_read_handler(uint16_t conn_id,
                             wiced_bt_gatt_opcode_t opcode,
                             wiced_bt_gatt_read_t *p_read_req,
                             uint16_t len_req);
wiced_bt_gatt_status_t
app_bt_gatt_req_write_handler(uint16_t conn_id,
                              wiced_bt_gatt_opcode_t opcode,
                              wiced_bt_gatt_write_req_t *p_write_req,
                              uint16_t len_req);
wiced_bt_gatt_status_t
app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                     wiced_bt_gatt_opcode_t opcode,
                                     wiced_bt_gatt_read_by_type_t *p_read_req,
                                     uint16_t len_requested);
wiced_bt_gatt_status_t
app_bt_gatt_operation_cplt_handler(wiced_bt_gatt_operation_complete_t *p_operation_cplt);
wiced_bt_gatt_status_t
app_bt_gatt_connection_up(wiced_bt_gatt_connection_status_t *p_status);
wiced_bt_gatt_status_t
app_bt_gatt_connection_down(wiced_bt_gatt_connection_status_t *p_status);
gatt_db_lookup_table_t
*app_bt_find_by_handle(uint16_t handle);
wiced_bt_gatt_status_t
app_bt_set_value(uint16_t attr_handle,
                 uint8_t *p_val,
                 uint16_t len);
void app_bt_free_buffer(uint8_t *p_event_data);
void* app_bt_alloc_buffer(int len);
void app_bt_write_peer_cccd(uint8_t* cccd_val, uint16_t conn_id);
void app_bt_write_peer_ead_cccd(uint8_t* cccd_val, uint16_t conn_id);
void app_bt_send_message(wiced_bt_gatt_operation_complete_t* op_cplt, uint16_t handle);
void app_bt_read_peer_attributes(uint16_t conn_id);
wiced_bt_gatt_status_t
app_bt_discovery_result_handler(wiced_bt_gatt_discovery_result_t *discovery_result);
wiced_bt_gatt_status_t
app_bt_service_discovery_handler(wiced_bt_gatt_discovery_complete_t *discovery_complete);
typedef void(*pfn_free_buffer_t) (uint8_t *);
void app_bt_store_peer_info(wiced_bt_gatt_connection_status_t* peer_conn_info);
uint8_t app_bt_get_peer_index(uint16_t conn_id);
void app_bt_update_gatt_db(uint8_t att_val_id, uint16_t conn_id, uint16_t handle);
void app_bt_local_gatt_db_info_init(void);

uint16_t app_bt_get_service_handle_from_conn_id(uint16_t conn_id);
uint16_t app_bt_get_peripheral_conn_id(uint16_t handle);
#endif /* SOURCE_APP_BT_APP_BT_GATT_HANDLER_H_ */
