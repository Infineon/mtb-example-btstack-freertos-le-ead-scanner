/*******************************************************************************
 * File Name: app_bt_gatt_handler.c
 *
 * Description: This file contains the task that handles GATT events for both
 *              server and client roles.
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
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "app_bt_bonding.h"
#include "app_flash_common.h"
#include "cycfg_gap.h"
#include "app_bt_utils.h"
#include "app_bt_event_handler.h"
#include "app_bt_gatt_handler.h"
#include "app_hw_device.h"
#include "wiced_bt_l2c.h"
#ifdef ENABLE_BT_SPY_LOG
#include "cybt_debug_uart.h"
#endif

/*******************************************************************************
*        Macros
*******************************************************************************/
#define ATT_CCCD         (1)
#define ATT_BLINK        (2)
#define ATT_DEVICE_NAME  (3)
#define CCCD_LEN         ATT_BLINK

/* Macros to calculate handles from service handle in local GATT DB */
#define NOTIFY_ATT_OFFSET (2)
#define BLINK_ATT_OFFSET  (5)
#define CCCD_ATT_OFFSET   (3)
#define DEVICE_NAME_ATT_OFFSET (7)
#define EAD_ATT_OFFSET    (9)
#define EAD_CCCD_ATT_OFFSET (10)

/* UUID of custom Hello Sensor service */
#define __UUID_SERVICE_HELLO_SENSOR 0x38, 0x28, 0x2E, 0x5F, 0xA5, 0x1E, 0xC7, 0xA4, 0xC2, 0x46, 0x47, 0x74, 0xB6, 0xC7, 0x81, 0x2F
#define __UUID_CHARACTERISTIC_HELLO_SENSOR_NOTIFY                0xB1, 0xAB, 0x23, 0xA3, 0x9C, 0xBE, 0x4B, 0xBC, 0xC3, 0x4F, 0xFE, 0x68, 0xF4, 0xA9, 0xD9, 0x07
#define __UUID_CHARACTERISTIC_HELLO_SENSOR_BLINK                 0x5A, 0x06, 0xFF, 0x11, 0xD8, 0xBC, 0x79, 0x88, 0x76, 0x49, 0x85, 0x21, 0xB9, 0x3F, 0x39, 0x7C
#define __UUID_CHARACTERISTIC_HELLO_SENSOR_DEVICE_NAME           0x1F, 0x66, 0xC4, 0x90, 0x02, 0x78, 0xAF, 0x90, 0x87, 0x4B, 0x6B, 0x0C, 0x49, 0x16, 0x54, 0x82
#define __UUID_CHARACTERISTIC_HELLO_SENSOR_EAD_KEY_MATERIAL      0x88, 0x2B

/*******************************************************************************
*        Variable Definitions
*******************************************************************************/
/* Variable to store handles of GATT DB from connected Peripheral from service discovery */
hs_discovery_data_t hs_discovery_data;

/* UUID of custom Hello Sensor service */
uint8_t uuid_service_hello_sensor[MAX_UUID_SIZE] = {__UUID_SERVICE_HELLO_SENSOR};
uint8_t uuid_char_hello_sensor_notify[MAX_UUID_SIZE] = {__UUID_CHARACTERISTIC_HELLO_SENSOR_NOTIFY};
uint8_t uuid_char_hello_sensor_blink[MAX_UUID_SIZE] = {__UUID_CHARACTERISTIC_HELLO_SENSOR_BLINK};
uint8_t uuid_char_hello_sensor_ead[2] = {__UUID_CHARACTERISTIC_HELLO_SENSOR_EAD_KEY_MATERIAL};

/* Variable that stores connection ID of peripheral against service handle in local GATT DB
   The application is tested for 3(MAX_PERIPHERALS) peripheral connections */
static local_gatt_db_info_t local_gatt_db_info[MAX_PERIPHERALS];

/* Variables sent to BTSTACK during read request and write request */
static uint8_t read_resp_data[30];

/* Connection ID of peer Central device */
static uint8_t central_connid;

/* Track the number of peripheral connections */
uint8_t num_peripherals;

wiced_timer_t ead_read_keys_timer;

/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
/*******************************************************************************
 * GATT Common - Both Server and Client related functions
 ******************************************************************************/
static void le_ead_read_keys_timer_cb(wiced_timer_callback_arg_t cb_params)
{
    /* Reboot */
    Cy_SysPm_TriggerSoftReset();
}

/**
 * Function Name: app_bt_gatt_event_callback
 *
 * Function Description:
 *   @brief This function handles GATT events from the BT stack.
 *
 *   @param wiced_bt_gatt_evt_t event                : LE GATT event code of one
 *          byte length
 *   @param wiced_bt_gatt_event_data_t *p_event_data : Pointer to LE GATT event
 *                                                    structures
 *
 *   @return wiced_bt_gatt_status_t                  : See possible status
 *           codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_callback(wiced_bt_gatt_evt_t event,
                            wiced_bt_gatt_event_data_t *p_event_data)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    /* Call the appropriate callback function based on the GATT event type,
     * and pass the relevant event parameters to the callback function */
    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            gatt_status = app_bt_gatt_conn_status_handler(&p_event_data->connection_status);
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            gatt_status = app_bt_discovery_result_handler(&p_event_data->discovery_result);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            gatt_status = app_bt_service_discovery_handler(&p_event_data->discovery_complete);
            if(hs_discovery_data.service_found == 2)
            {
                /* If the custom Hello Sensor service is found, read all the readable attributes
                   and store in local GATT DB */
                app_bt_read_peer_attributes(p_event_data->discovery_complete.conn_id);
            }
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            gatt_status = app_bt_gatt_req_handler(&p_event_data->attribute_request);

            break;

        case GATT_OPERATION_CPLT_EVT:
            gatt_status = app_bt_gatt_operation_cplt_handler(&p_event_data->operation_complete);
            break;

            /* GATT buffer request, typically sized to max of bearer mtu - 1 */
        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_event_data->buffer_request.buffer.p_app_rsp_buffer =
            app_bt_alloc_buffer(p_event_data->buffer_request.len_requested);
            p_event_data->buffer_request.buffer.p_app_ctxt = (void *)app_bt_free_buffer;
            gatt_status = WICED_BT_GATT_SUCCESS;
            break;

            /* GATT buffer transmitted event,
             * check \ref wiced_bt_gatt_buffer_transmitted_t*/
        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free =
            (pfn_free_buffer_t)p_event_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function
             * to free it. */
            if (NULL != pfn_free)
                pfn_free(p_event_data->buffer_xmitted.p_app_data);

            gatt_status = WICED_BT_GATT_SUCCESS;
        }
            break;

        default:
            gatt_status = WICED_BT_GATT_SUCCESS;
               break;
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_gatt_conn_status_handler
 *
 * Function Description:
 *   @brief This callback function handles connection status changes.
 *
 *   @param wiced_bt_gatt_connection_status_t *p_conn_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                          : See possible
 *    status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_conn_status_handler(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    if (p_conn_status->connected)
    {
        wiced_init_timer(&ead_read_keys_timer, le_ead_read_keys_timer_cb, NULL, WICED_SECONDS_TIMER);
        return app_bt_gatt_connection_up(p_conn_status);
    }
    else
    {
        wiced_deinit_timer(&ead_read_keys_timer);
        return app_bt_gatt_connection_down(p_conn_status);
    }
}

/**
 * Function Name: app_bt_gatt_connection_up
 *
 * Function Description:
 *   @brief This function is invoked when connection is established
 *
 *   @param wiced_bt_gatt_connection_status_t *p_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                     : See possible status
 *   codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    uint8_t peer_index;
    wiced_result_t result;
    wiced_bt_gatt_status_t gatt_status =  WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_discovery_param_t service_discovery_setup =
    {
        .s_handle = 0x01,
        .e_handle = 0xffff,
        .uuid.len = LEN_UUID_128,
        .uuid.uu.uuid128 = {__UUID_SERVICE_HELLO_SENSOR}
    };

    printf("Connected to peer device: ");
    print_bd_address(p_status->bd_addr);
    /* Store the latest connection ID for easy access */
    current_conn_id = p_status->conn_id;
    printf("Connection ID '%d' \n", current_conn_id);

    /* Store and print the connection information of the peer in a local variable
       This data will be used in GATT and Security operations following the connection */
    app_bt_store_peer_info(p_status);
    for (int i = 0; i < MAX_NUM_CONNECTIONS; i++)
    {
        if (peer_info[i].conn_id != 0) {
            printf("index = %d, conn_id = %d, addr_type = %d, transport = %d\n", i, peer_info[i].conn_id, peer_info[i].addr_type, peer_info[i].transport);
            print_bd_address(peer_info[i].remote_addr);
        }
    }

    /* Check if the connected device is Peripheral */
    /* if are_we_central flag is set we are in central role for that connection*/
    peer_index = app_bt_get_peer_index(p_status->conn_id);
    if(peer_index >= 4){
        printf("This app only supports upto 4 connections (including Central device)\n");
        return WICED_BT_GATT_SUCCESS;
    }
    if(peer_info[peer_index].are_we_central)
    {

        /*Disconnect from the newly connected device if maximum peripheral limit reached*/
        if(MAX_PERIPHERALS == num_peripherals)
        {
            printf("Reached maximum peripheral limit, disconnecting from recently connected hello sensor! \r\n");
            wiced_bt_gatt_disconnect(current_conn_id);

            /* If no Central device is connected to us yet, start Advertisements */
            if(!central_connid)
            {
                result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
                if(WICED_BT_SUCCESS != result)
                {
                    printf("Failed to start advertisement, result = %x\n", result);
                }
            }

            return WICED_BT_GATT_SUCCESS;
        }

        /* Increment the number of peripherals connected count */
        num_peripherals++;

        /* Send connection parameter update request to peripheral */
        if(!wiced_bt_l2cap_update_ble_conn_params(p_status->bd_addr,CY_BT_CONN_MIN_INTERVAL,
                                                 CY_BT_CONN_MAX_INTERVAL, CY_BT_CONN_LATENCY,
                                                 CY_BT_CONN_SUPERVISION_TIMEOUT))
        {
            printf("Failed to Send Connection update parameter request \r\n");
        }

        /* If no Central device is connected to us yet, start Advertisements */
        if(!central_connid)
        {
            result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
            if(WICED_BT_SUCCESS != result)
            {
                printf("Failed to start advertisement, result = %x\n", result);
            }
        }

        /* Update local GATT DB with device name of connected peripheral. This function call will
           associate a free custom hello sensor service(out of the 3 services) to the connected peripheral */
        app_bt_update_gatt_db(ATT_DEVICE_NAME, p_status->conn_id, 0);

        /* Start Service Discovery to get handles of the attributes in connected GATT Server */
        /* Reset the flag to indicate that the custom hello service is not found yet. This is
           used to identify when a custom hello sensor service is found */
        hs_discovery_data.service_found = 0;
        gatt_status = wiced_bt_gatt_client_send_discover(p_status->conn_id,
                                            GATT_DISCOVER_SERVICES_BY_UUID,
                                                    &service_discovery_setup);
        if(WICED_BT_GATT_SUCCESS != gatt_status)
        {
            printf("GATT Discovery request failed. Error code: %d, "
                    "Conn id: %d\n", gatt_status, p_status->conn_id);
        }
        else
        {
            printf("Service Discovery Started\n");
        }
    }
    /* Check if the connected device is Central */
    else
    {
        /* Store the connection ID of the peer if it is central for easy access */
        central_connid = p_status->conn_id;
    }

    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function Name: app_bt_gatt_connection_down
 *
 * Function Description:
 *   @brief This function is invoked when connection is disconnected
 *
 *   @param wiced_bt_gatt_connection_status_t *p_status :
 *          Pointer to data that has connection details
 *
 *   @return wiced_bt_gatt_status_t                     : See possible status
 *           codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_connection_down(wiced_bt_gatt_connection_status_t *p_status)
{
    wiced_result_t result;
    uint8_t peer_index;
    printf("Peer device disconnected: ");
    print_bd_address(p_status->bd_addr);

    printf("conn_id:%d reason:%s\n", p_status->conn_id,
           get_bt_gatt_disconn_reason_name(p_status->reason));

    peer_index = app_bt_get_peer_index(p_status->conn_id);

    if(peer_index >= 4){
        printf("This app only supports upto 4 connections (including Central device)\n");
        return WICED_BT_GATT_SUCCESS;
    }

    /*If peripheral has disconnected, decrement the number of peripheral connection count */
    if(peer_info[peer_index].are_we_central)
    {
        /* Search for the custom Hello Sensor service used by the disconnected peripheral and free it for the new connection */
        for(uint8_t index = 0; index < MAX_PERIPHERALS; index++)
        {
            if(p_status->conn_id == local_gatt_db_info[index].conn_id)
            {
                /* Update the flag in local gatt db info to indicate that the service is free for next connection */
                /*Note: Any stored data will continue to exist until a new hello sensor device connects and overwrites it.*/
                local_gatt_db_info[index].is_service_free = TRUE;
            }
        }
        num_peripherals--;
    }
    else
    {
        central_connid = 0;
        /* Start advertisement as central has disconnected*/
        result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);

        if(WICED_BT_SUCCESS != result)
        {
            printf("Failed to start advertisement, result = %x\n", result);
        }
    }

    /* Clear the disconnected peer info */
    memset(&(peer_info[peer_index].conn_id), 0, sizeof(hello_client_peer_info_t));

    /* Reset the stored connection IDs and flag */
    current_conn_id = 0;
    hs_discovery_data.service_found = 0;

    return WICED_BT_GATT_SUCCESS;
}


/*******************************************************************************
 * GATT Server
 ******************************************************************************/
/**
 * Function Name: app_bt_gatt_req_handler
 *
 * Function Description:
 *   @brief This function handles GATT server events from the BT stack.
 *
 * @param p_attr_req             : Pointer to LE GATT connection status
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_handler(wiced_bt_gatt_attribute_request_t *p_attr_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    uint8_t peer_index;
    uint16_t peri_conn_id;
    switch ( p_attr_req->opcode )
    {
        case GATT_REQ_READ:
             /* Attribute read request */
            gatt_status =
            app_bt_gatt_req_read_handler(p_attr_req->conn_id,
                                                p_attr_req->opcode,
                                                &p_attr_req->data.read_req,
                                                p_attr_req->len_requested);
             break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
            /* Attribute write request */
            gatt_status =
            app_bt_gatt_req_write_handler(p_attr_req->conn_id,
                                                p_attr_req->opcode,
                                                &p_attr_req->data.write_req,
                                                p_attr_req->len_requested);
            /* Based on the Handle to which write is received, forward it to corresponding Peripheral */
            /* Add code for any action required when an attribute is written */
            /*Get connection ID of the peripheral to which the request needs to be forwarded*/
            peri_conn_id = app_bt_get_peripheral_conn_id(p_attr_req->data.write_req.handle);
            printf("Peripheral connection id: %d \n",peri_conn_id);
            switch (p_attr_req->data.write_req.handle)
            {
                /* By writing into Characteristic Client Configuration descriptor
                    peer Central can enable or disable notification/indication on required Peripheral */
                case HDLD_HELLO_SENSOR_1_NOTIFY_CLIENT_CHAR_CONFIG:
                {
                    // printf("write handler inside set value\n");
                    // printf("len cccd = %d\n", p_attr_req->data.write_req.val_len);
                    if((p_attr_req->data.write_req.val_len == CCCD_LEN) && (num_peripherals != 0))
                    {
                        // printf("p_val[0] = %d, p_val[1] = %d\n", p_attr_req->data.write_req.p_val[0], p_attr_req->data.write_req.p_val[1]);
                        app_bt_update_gatt_db(ATT_CCCD, peri_conn_id, p_attr_req->data.write_req.handle);
                    }
                }
                break;

                /* Central can blink the LED on Peripheral by writing into Blink characteristic */
                case HDLC_HELLO_SENSOR_1_BLINK_VALUE:
                {
                    if((p_attr_req->data.write_req.val_len == app_hello_sensor_1_blink_len) && (num_peripherals != 0))
                    {
                        app_bt_update_gatt_db(ATT_BLINK, peri_conn_id, p_attr_req->data.write_req.handle);
                    }
                }
                break;

                case HDLD_GATT_SERVICE_CHANGED_CLIENT_CHAR_CONFIG:
                    gatt_status = WICED_BT_GATT_SUCCESS;
                break;
            }

            break;

        case GATT_REQ_MTU:
            gatt_status =
            wiced_bt_gatt_server_send_mtu_rsp(p_attr_req->conn_id,
                                              p_attr_req->data.remote_mtu,
                                              CY_BT_MTU_SIZE);
            break;

        case GATT_HANDLE_VALUE_NOTIF:
            break;

        case GATT_REQ_READ_BY_TYPE:
            gatt_status =
            app_bt_gatt_req_read_by_type_handler(p_attr_req->conn_id,
                                                       p_attr_req->opcode,
                                                       &p_attr_req->data.read_by_type,
                                                       p_attr_req->len_requested);
            break;

        case GATT_HANDLE_VALUE_CONF:
            {
                peer_index = app_bt_get_peer_index(p_attr_req->conn_id);
                if(peer_index >= 4){
                    printf("This app only supports upto 4 connections (including Central device)\n");
                    return WICED_BT_GATT_SUCCESS;
                }
                printf("Indication Confirmation received \n");
                peer_info[peer_index].flag_indication_sent = FALSE;
            }
            break;

        default:
            printf("ERROR: Unhandled GATT Connection Request case: %d\n",
                    p_attr_req->opcode);
            break;
    }

    return gatt_status;
}

/**
 * Function Name: app_bt_discovery_result_handler
 *
 * Function Description:
 *   @brief This function handles discovery results from bluetooth stack.
 *
 *   @param wiced_bt_gatt_discovery_result_t *discovery_result : Pointer to data
 *                                            that has discovery details.
 *
 *   @return wiced_bt_gatt_status_t: See possible status codes in
 *                                   wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_discovery_result_handler(wiced_bt_gatt_discovery_result_t *discovery_result)
{
    wiced_bt_gatt_status_t gatt_status =  WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_discovery_type_t discovery_type;
    discovery_type = discovery_result->discovery_type;
    switch (discovery_type)
    {
        case GATT_DISCOVER_SERVICES_BY_UUID:
            if(!memcmp(uuid_service_hello_sensor, discovery_result->discovery_data.group_value.service_type.uu.uuid128, MAX_UUID_SIZE))
            {
                hs_discovery_data.hs_start_handle = discovery_result->discovery_data.group_value.s_handle;
                hs_discovery_data.hs_end_handle = discovery_result->discovery_data.group_value.e_handle;
                printf("Custom Hello Sensor Service Found, Start Handle = %d, End Handle = %d \n",
                        hs_discovery_data.hs_start_handle,
                        hs_discovery_data.hs_end_handle);
            }
            break;

        case GATT_DISCOVER_CHARACTERISTICS:
            if(!memcmp(uuid_char_hello_sensor_notify, discovery_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid128, MAX_UUID_SIZE))
            {
                hs_discovery_data.hs_char_notify_handle = discovery_result->discovery_data.characteristic_declaration.handle;
                hs_discovery_data.hs_char_val_notify_handle = discovery_result->discovery_data.characteristic_declaration.val_handle;
                printf("Hello Sensor Notify characteristic handle = %d, "
                       "Hello Sensor Notify characteristic value handle = %d\n",
                        hs_discovery_data.hs_char_notify_handle,
                        hs_discovery_data.hs_char_val_notify_handle);
            }
            else if(!memcmp(uuid_char_hello_sensor_blink, discovery_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid128, MAX_UUID_SIZE))
            {
                hs_discovery_data.hs_char_blink_handle = discovery_result->discovery_data.characteristic_declaration.handle;
                hs_discovery_data.hs_char_val_blink_handle = discovery_result->discovery_data.characteristic_declaration.val_handle;
                printf("Hello Sensor Blink characteristic handle = %d, "
                       "Hello Sensor Blink characteristic value handle = %d\n",
                        hs_discovery_data.hs_char_blink_handle,
                        hs_discovery_data.hs_char_val_blink_handle);
            }
            else if(!memcmp(uuid_char_hello_sensor_ead, &discovery_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid16, 2))
            {
                hs_discovery_data.hs_char_ead_key_handle = discovery_result->discovery_data.characteristic_declaration.handle;
                hs_discovery_data.hs_char_val_ead_key_handle = discovery_result->discovery_data.characteristic_declaration.val_handle;
                printf("Hello Sensor EAD characteristic handle = %d, "
                       "Hello Sensor EAD characteristic value handle = %d\n",
                        hs_discovery_data.hs_char_ead_key_handle,
                        hs_discovery_data.hs_char_val_ead_key_handle);
            }
            break;

        case GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS:
            if(__UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION ==
               discovery_result->discovery_data.char_descr_info.type.uu.uuid16)
            {
                if(hs_discovery_data.service_found == 0)
                {
                    hs_discovery_data.hs_cccd_handle = discovery_result->discovery_data.char_descr_info.handle;
                    printf("Hello Sensor Notify CCCD found, Handle = %d\n",
                        hs_discovery_data.hs_cccd_handle);
                }
                else if(hs_discovery_data.service_found == 1)
                {
                    hs_discovery_data.hs_ead_cccd_handle = discovery_result->discovery_data.char_descr_info.handle;
                    printf("Hello Sensor EAD CCCD found, Handle = %d\n",
                        hs_discovery_data.hs_ead_cccd_handle);
                }
                hs_discovery_data.service_found++;

            }
            break;

        default:
            break;
        }

    return gatt_status;
}

/**
 * Function Name: app_bt_service_discovery_handler
 *
 * Function Description:
 *   @brief This function handles service discovery for custom Hello Sensor
 *          service.
 *
 *   @param wiced_bt_gatt_discovery_complete_t *discovery_complete : Pointer to
 *                     data that has type and status of the completed discovery.
 *
 *   @return wiced_bt_gatt_status_t: See possible status codes in
 *           wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_service_discovery_handler(wiced_bt_gatt_discovery_complete_t *discovery_complete)
{
    wiced_bt_gatt_status_t gatt_status =  WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_discovery_param_t char_discovery_setup = {0};
    wiced_bt_gatt_discovery_type_t discovery_type;
    discovery_type = discovery_complete->discovery_type;
    switch (discovery_type)
    {
        case GATT_DISCOVER_SERVICES_BY_UUID:
        {
            char_discovery_setup.s_handle = hs_discovery_data.hs_start_handle;
            char_discovery_setup.e_handle = hs_discovery_data.hs_end_handle;
            gatt_status = wiced_bt_gatt_client_send_discover(discovery_complete->conn_id,
                                                GATT_DISCOVER_CHARACTERISTICS,
                                                &char_discovery_setup);
            if(WICED_BT_GATT_SUCCESS != gatt_status)
                printf("GATT characteristics discovery failed! Error code = %d\n", gatt_status);
        }
        break;

        case GATT_DISCOVER_CHARACTERISTICS:
        {
            char_discovery_setup.s_handle = hs_discovery_data.hs_start_handle;
            char_discovery_setup.e_handle = hs_discovery_data.hs_end_handle;
            gatt_status = wiced_bt_gatt_client_send_discover(discovery_complete->conn_id,
                                                            GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,
                                                            &char_discovery_setup);

            if(WICED_BT_GATT_SUCCESS != gatt_status)
                printf("GATT CCCD discovery failed! Error code = %d\n", gatt_status);
        }
        break;

        default:
            break;
    }
    return gatt_status;
}

/**
 * Function Name: app_bt_gatt_req_read_handler
 *
 * Function Description:
 *   @brief This function handles Read Requests received from the client device
 *
 *   @param conn_id              : Connection ID
 *   @param opcode               : LE GATT request type opcode
 *   @param p_read_req           : Pointer to read request containing the handle
 *          to read
 *   @param len_req              : length of data requested
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_read_handler( uint16_t conn_id,
                                    wiced_bt_gatt_opcode_t opcode,
                                    wiced_bt_gatt_read_t *p_read_req,
                                    uint16_t len_req)
{

    gatt_db_lookup_table_t  *puAttribute;
    int          attr_len_to_copy;
    uint8_t     *from;
    int          to_send;


    puAttribute = app_bt_find_by_handle(p_read_req->handle);
    if (NULL == puAttribute)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->cur_len;

    printf("Read request received: conn_id:%d Handle:%x offset:%d len:%d\n ",
            conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return WICED_BT_GATT_INVALID_OFFSET;
    }

    to_send = MIN(len_req, attr_len_to_copy - p_read_req->offset);
    from = ((uint8_t *)puAttribute->p_data) + p_read_req->offset;
    /* No need for context, as buff not allocated */
    return wiced_bt_gatt_server_send_read_handle_rsp(conn_id,
                                                     opcode,
                                                     to_send,
                                                     from,
                                                     NULL);
}

/**
 * Function Name: app_bt_gatt_req_write_handler
 *
 * Function Description:
 *   @brief This function handles Write Requests received from the client device
 *
 *   @param conn_id                : Connection ID
 *   @param opcode                 : LE GATT request type opcode
 *   @param p_write_req            : Pointer to LE GATT write request
 *   @param len_req                : length of data requested
 *
 *   @return wiced_bt_gatt_status_t: See possible status codes in
 *                                   wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_write_handler(uint16_t conn_id,
                                    wiced_bt_gatt_opcode_t opcode,
                                    wiced_bt_gatt_write_req_t *p_write_req,
                                    uint16_t len_req)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;

    printf("Write Request/Command received: conn_id:%d Handle:0x%x offset:%d len:%d\n",
           conn_id, p_write_req->handle, p_write_req->offset, p_write_req->val_len);

    /* Set the value that has come through write request in the local GATT DB */
    gatt_status = app_bt_set_value(p_write_req->handle,
                                         p_write_req->p_val,
                                         p_write_req->val_len);

    if(WICED_BT_GATT_SUCCESS != gatt_status)
    {
        printf("Failed to set value in local GATT DB, result: 0x%x\n", gatt_status);
    }
    else
    {
        if(GATT_REQ_WRITE == opcode)
        {
            wiced_bt_gatt_server_send_write_rsp(conn_id,
                                                opcode,
                                                p_write_req->handle);
        }
    }
    return (gatt_status);
}

/**
 * Function Name: app_bt_set_value
 *
 * Function Description:
 *   @brief This function handles writing to the attribute handle in the GATT
 *   database using the data passed from the BT stack. The value to write is
 *   stored in a buffer whose starting address is passed as one of the
 *   function parameters
 *
 *   @param uint16_t attr_handle : GATT attribute handle
 *   @param uint8_t  p_val       : Pointer to LE GATT write request value
 *   @param uint16_t len         : length of GATT write request
 *
 *
 * @return wiced_bt_gatt_status_t: See possible status codes in
 *                                 wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t app_bt_set_value(uint16_t attr_handle,
                                              uint8_t *p_val,
                                              uint16_t len)
{
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_INVALID_HANDLE;

    /* Check for a matching handle entry */
    for (int i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Check if the buffer has space to store the data */
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);

            if (validLen)
            {
                /* Value fits within the supplied buffer; copy over the value */
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                gatt_status = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                /* Value to write does not meet size constraints */
                gatt_status = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
        else
        {
            /* Handle not found in local GATT DB */
            gatt_status = WICED_BT_GATT_INVALID_HANDLE;
        }
    }
    return gatt_status;
}

/**
 * Function Name : app_bt_gatt_req_read_by_type_handler
 *
 * Function Description :
 *   @brief Process read-by-type request from peer device
 *
 *   @param uint16_t conn_id                       : Connection ID
 *   @param wiced_bt_gatt_opcode_t opcode          : LE GATT request type opcode
 *   @param wiced_bt_gatt_read_by_type_t p_read_req: Pointer to read request
 *          containing the handle to read
 *  @param uint16_t len_requested                  : Length of data requested
 *
 * Return:
 *  wiced_bt_gatt_status_t                         : LE GATT status
 */
wiced_bt_gatt_status_t
app_bt_gatt_req_read_by_type_handler(uint16_t conn_id,
                                           wiced_bt_gatt_opcode_t opcode,
                                           wiced_bt_gatt_read_by_type_t *p_read_req,
                                           uint16_t len_requested)
{
    gatt_db_lookup_table_t *puAttribute;
    uint16_t last_handle = 0;
    uint16_t attr_handle = p_read_req->s_handle;
    uint8_t *p_rsp = app_bt_alloc_buffer(len_requested);
    uint8_t pair_len = 0;
    int used_len = 0;

    if (NULL == p_rsp)
    {
        printf("No memory, len_requested: %d!!\n",len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INSUF_RESOURCE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (WICED_TRUE)
    {
        last_handle = attr_handle;
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle,
                                                        p_read_req->e_handle,
                                                        &p_read_req->uuid);
        if (0 == attr_handle)
            break;

        if ( NULL == (puAttribute = app_bt_find_by_handle(attr_handle)))
        {
            printf("found type but no attribute for %d \n",last_handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id,
                                                opcode,
                                                p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            app_bt_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        int filled =
        wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used_len,
                                                     len_requested - used_len,
                                                     &pair_len,
                                                     attr_handle,
                                                     puAttribute->cur_len,
                                                     puAttribute->p_data);
        if (0 == filled)
        {
            break;
        }
        used_len += filled;

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (0 == used_len)
    {
        printf("attr not found  start_handle: 0x%04x"
               "end_handle: 0x%04x  Type: 0x%04x\n", p_read_req->s_handle,
                                                       p_read_req->e_handle,
                                                       p_read_req->uuid.uu.uuid16);
        wiced_bt_gatt_server_send_error_rsp(conn_id,
                                            opcode,
                                            p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        app_bt_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    return wiced_bt_gatt_server_send_read_by_type_rsp(conn_id,
                                                      opcode,
                                                      pair_len,
                                                      used_len,
                                                      p_rsp,
                                                      (void *)app_bt_free_buffer);
}

/**
 * Function Name: app_bt_send_message
 *
 * Function Description:
 *   @brief Check if client has registered for notification/indication from any connected peripheral.
 *          Whenever peripheral sends notification/indication, store it in corresponding custom Hello
 *          Sensor service in local GATT DB. Forward the same value as notification to Central.
 *
 *   @param wiced_bt_gatt_operation_complete_t op_cplt: Refer wiced_bt_gatt_operation_complete_t
 *   @param uint16_t handle: handle of local GATT DB to store the notification/indication
 *                           from Peripheral and send it to Central.
 *
 *   @return None
 *
 */
void app_bt_send_message(wiced_bt_gatt_operation_complete_t* op_cplt, uint16_t handle)
{
    wiced_bt_gatt_status_t status;

    /*Handle should be a non zero valid value, if its zero return with error message*/
    if(handle==0)
    {
        printf("Invalid Handle Value \r\n");
        return;
    }
    /* Check if the central is connected or not*/
    if(central_connid == 0)
    {
        printf("Central is not connected, Not forwarding the notification \r\n");
        return;
    }

    /* Search through local GATT DB for the notification handle and corresponding array to store the notification value.
       This value is sent by one of the connected peripherals */
    for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if(app_gatt_db_ext_attr_tbl[i].handle == handle + NOTIFY_ATT_OFFSET)
        {
            /* Copy the data */
            memcpy(app_gatt_db_ext_attr_tbl[i].p_data, op_cplt->response_data.att_value.p_data, app_gatt_db_ext_attr_tbl[i].max_len);
            status = wiced_bt_gatt_server_send_notification(central_connid,
                                                                app_gatt_db_ext_attr_tbl[i].handle,
                                                                app_gatt_db_ext_attr_tbl[i].max_len,
                                                                app_gatt_db_ext_attr_tbl[i].p_data,
                                                                NULL);
                printf("Notification Status: %d \n", status);
            }
        }

    }
/*******************************************************************************
 * GATT Client
 ******************************************************************************/
/**
 * Function Name: app_bt_gatt_operation_cplt_handler
 *
 * Function Description:
 *   @brief This callback function handles GATT response data from server device
 *         for GATT requests from client.
 *
 *   @param wiced_bt_gatt_operation_complete_t *p_operation_cplt :
 *          Pointer to response data by server for client GATT requests
 *
 *   @return wiced_bt_gatt_status_t                          : See possible
 *    status codes in wiced_bt_gatt_status_e in wiced_bt_gatt.h
 *
 */
wiced_bt_gatt_status_t
app_bt_gatt_operation_cplt_handler(wiced_bt_gatt_operation_complete_t *p_operation_cplt)
{
    wiced_bt_gatt_status_t status;
    uint8_t peer_index;
    uint16_t handle;
    wiced_result_t result;
    uint8_t dummy[2] = {0x00, 0x00};
    uint16_t notify_cccd_value = 0x0001;
    uint16_t indicate_cccd_value = 0x0002;
    uint8_t dummy_keys[24] = {0};

    switch(p_operation_cplt->op)
    {
        /* Notification/Indication received from GATT Server */
        case GATTC_OPTYPE_NOTIFICATION:
        case GATTC_OPTYPE_INDICATION:
        {
            /* If the GATT Server has sent an indication, send a confirmation packet to GATT Server */
            if(p_operation_cplt->op == GATTC_OPTYPE_INDICATION)
            {
                status = wiced_bt_gatt_client_send_indication_confirm(p_operation_cplt->conn_id, p_operation_cplt->response_data.att_value.handle);
                printf("Indication cnf send status = %d\n", status);
            }
            /* This notification or Indication needs to be sent to Central.
               Get the handle of service in which the value from GATT server
               needs to be stored using the associated connection ID of peer GATT Server */
            handle = app_bt_get_service_handle_from_conn_id(p_operation_cplt->conn_id);
            uint16_t length = p_operation_cplt->response_data.att_value.len;
            p_operation_cplt->response_data.att_value.p_data[length] = '\0';
            printf("GATT Notification - %s \n", p_operation_cplt->response_data.att_value.p_data);
            printf("Connection ID - %d \n", p_operation_cplt->conn_id);
            printf("service handle = %d\n", handle);
            app_bt_send_message(p_operation_cplt, handle);
        }
        break;
        /* Write response received from GATT Server */
        case GATTC_OPTYPE_WRITE_WITH_RSP:
        {
            if(p_operation_cplt->status == WICED_BT_GATT_SUCCESS)
            {
                printf("Write Response success for handle = %d, connection ID: %d\n", p_operation_cplt->response_data.att_value.handle, p_operation_cplt->conn_id);
                /* Read remaining attributes */
                app_bt_read_peer_attributes(p_operation_cplt->conn_id);
            }
        }
        break;
        /* Read response received from GATT Server */
        case GATTC_OPTYPE_READ_HANDLE:
        {
            /* Get the handle of service in which the read response value from GATT server
               needs to be stored using the associated connection ID of peer GATT Server */
            handle = app_bt_get_service_handle_from_conn_id(p_operation_cplt->conn_id);
            // printf("in op complete read handle, status = %x \n", p_operation_cplt->status);
            /* Get the index of peer data stored to set flags to track completion of read */
            peer_index = app_bt_get_peer_index(p_operation_cplt->conn_id);
            if(peer_index >= MAX_NUM_CONNECTIONS)
            {
                printf("This app only supports upto 4 connections (including Central device)\n");
                return WICED_BT_GATT_SUCCESS;
            }
            /* Check the handle from which the read response is received. There are three readable attributes: Notify, CCCD and Blink characteristic */
            if((p_operation_cplt->status == WICED_BT_GATT_SUCCESS) && (p_operation_cplt->response_data.handle == hs_discovery_data.hs_char_val_notify_handle))
            {
                printf("Read response received for notify characteristic\n");
                /* Store the value in read response in local GATT DB */
                for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
                {
                    if(app_gatt_db_ext_attr_tbl[i].handle == (handle + NOTIFY_ATT_OFFSET))
                    {
                        memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_operation_cplt->response_data.att_value.p_data, app_gatt_db_ext_attr_tbl[i].max_len);
                        // printf("p_data[0] = %c, p_data[6] = %c\n", app_gatt_db_ext_attr_tbl[i].p_data[0], app_gatt_db_ext_attr_tbl[i].p_data[6]);
                        break;
                    }
                }
                /* Read request on Notify characteristic is successful, Set the flag */
                peer_info[peer_index].is_read_notify_char_cplt = TRUE;
                // printf("p_operation_cplt->p_data adress = %d\n", &p_operation_cplt->response_data.att_value.p_data[0]);
                // printf("p_operation_cplt->first member = %d\n", &p_operation_cplt->conn_id);
                // printf("p_operation_cplt->response data = %d\n", &p_operation_cplt->response_data);
                // printf("p_operation_cplt->att_val = %d\n", &p_operation_cplt->response_data.att_value);
                // printf("p_operation_cplt->att_val.handle = %d\n", &p_operation_cplt->response_data.att_value.handle);
                /* Read remaining attributes */
                app_bt_read_peer_attributes(p_operation_cplt->conn_id);
            }
            else if((p_operation_cplt->status == WICED_BT_GATT_SUCCESS) && (p_operation_cplt->response_data.handle == hs_discovery_data.hs_char_val_blink_handle))
            {
                printf("Read response received for blink characteristic\n");
                /* Store the value in read response in local GATT DB */
                for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
                {
                    if(app_gatt_db_ext_attr_tbl[i].handle == (handle + BLINK_ATT_OFFSET))
                    {
                        memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_operation_cplt->response_data.att_value.p_data, app_gatt_db_ext_attr_tbl[i].max_len);
                        break;
                    }
                }
                /* Read request on CCCD is successful, Set the flag */
                peer_info[peer_index].is_read_blink_char_cplt = TRUE;
                /* Read remaining attributes */
                app_bt_read_peer_attributes(p_operation_cplt->conn_id);
            }
            else if((p_operation_cplt->status == WICED_BT_GATT_SUCCESS) && (p_operation_cplt->response_data.handle == hs_discovery_data.hs_cccd_handle))
            {
                printf("Read response received for notify cccd\n");
                if(p_operation_cplt->response_data.att_value.len == 2 && memcmp(p_operation_cplt->response_data.att_value.p_data, dummy, sizeof(dummy)) == 0)
                {
                    app_bt_write_peer_cccd((uint8_t*)&notify_cccd_value, p_operation_cplt->conn_id);
                }
                for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
                {
                    if(app_gatt_db_ext_attr_tbl[i].handle == (handle + CCCD_ATT_OFFSET))
                    {
                        memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_operation_cplt->response_data.att_value.p_data, app_gatt_db_ext_attr_tbl[i].max_len);
                        break;
                    }
                }
                /* Read request on Blink characteristic is successful, Set the flag */
                peer_info[peer_index].is_read_cccd_cplt = TRUE;
                /* Read remaining attributes */
                app_bt_read_peer_attributes(p_operation_cplt->conn_id);
            }
            else if((p_operation_cplt->status == WICED_BT_GATT_SUCCESS) && (p_operation_cplt->response_data.handle == hs_discovery_data.hs_char_val_ead_key_handle))
            {
                printf("Read response received for EAD characteristic\n");
                for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
                {
                    if(app_gatt_db_ext_attr_tbl[i].handle == (handle + EAD_ATT_OFFSET))
                    {
                        memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_operation_cplt->response_data.att_value.p_data, app_gatt_db_ext_attr_tbl[i].max_len);
                        if(p_operation_cplt->response_data.att_value.len == sizeof(wiced_bt_ble_key_material_t) &&
                          memcmp(p_operation_cplt->response_data.att_value.p_data, dummy_keys, sizeof(wiced_bt_ble_key_material_t)))
                        {
                            app_bt_update_ead_keys((wiced_bt_ble_key_material_t*)p_operation_cplt->response_data.att_value.p_data);
                            wiced_start_timer(&ead_read_keys_timer, 3);
                        }
                        break;
                    }
                }
                /* Read request on Blink characteristic is successful, Set the flag */
                peer_info[peer_index].is_read_ead_keys_cplt = TRUE;
                /* Read remaining attributes */
                app_bt_read_peer_attributes(p_operation_cplt->conn_id);
            }
            else if((p_operation_cplt->status == WICED_BT_GATT_SUCCESS) && (p_operation_cplt->response_data.handle == hs_discovery_data.hs_ead_cccd_handle))
            {
                printf("Read response received for EAD cccd\n");
                if(p_operation_cplt->response_data.att_value.len == 2 && memcmp(p_operation_cplt->response_data.att_value.p_data, dummy, sizeof(dummy)) == 0)
                {
                    app_bt_write_peer_ead_cccd((uint8_t*)&indicate_cccd_value, p_operation_cplt->conn_id);
                }
                for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
                {
                    if(app_gatt_db_ext_attr_tbl[i].handle == (handle + EAD_CCCD_ATT_OFFSET))
                    {
                        memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_operation_cplt->response_data.att_value.p_data, app_gatt_db_ext_attr_tbl[i].max_len);
                        break;
                    }
                }

                /* Read request on Blink characteristic is successful, Set the flag */
                peer_info[peer_index].is_read_ead_cccd_cplt = TRUE;
                /* Read remaining attributes */
                app_bt_read_peer_attributes(p_operation_cplt->conn_id);
            }
            /* Check if the GATT Server sent the error code indicating authentication is required before accessing the attribute
               Start the pairing procedure or encrypt the link based on bond info for the peer */
            if(((p_operation_cplt->status == WICED_BT_GATT_INSUF_ENCRYPTION) || (p_operation_cplt->status == WICED_BT_GATT_INSUF_AUTHENTICATION))
                && (p_operation_cplt->response_data.handle == hs_discovery_data.hs_char_val_notify_handle))
            {
                /* Reset the flag for read request to notify characteristic, our device has to send read request once encryption is complete */
                peer_info[peer_index].is_read_notify_char_cplt = FALSE;
                printf("Response received from server for read request: ");
                if(WICED_BT_GATT_INSUF_ENCRYPTION == p_operation_cplt->status)
                {
                    printf("WICED_BT_GATT_INSUF_ENCRYPTION\n");
                }
                else if(WICED_BT_GATT_INSUF_AUTHENTICATION == p_operation_cplt->status)
                {
                    printf("WICED_BT_GATT_INSUF_AUTHENTICATION\n");
                }
                /* Check if the device is present in NVRAM, if yes, we already have the keys to encrypt the connection, if no, we need to initiate pairing */
                if(BOND_INDEX_MAX == app_bt_find_device_in_flash(peer_info->remote_addr))
                {
                    printf("Peer device was not bonded previously, initiate pairing and bonding\n ");
                    result = wiced_bt_dev_sec_bond(peer_info[peer_index].remote_addr,
                                                    peer_info[peer_index].addr_type,
                                                    peer_info[peer_index].transport,
                                                    0, NULL);
                    if(WICED_BT_PENDING == result)
                    {
                        printf("Bonding initiated successfully \n");
                    }
                    else if(WICED_BT_SUCCESS == result)
                    {
                        printf("Peer device already paired \n");
                    }
                    else
                    {
                        printf("wiced_bt_dev_sec_bond failed, error code = %d\n", result);
                    }
                }
                else
                {
                    printf("Peer device is already bonded, encrypt the link\n");
                    wiced_bt_ble_sec_action_type_t sec_type = BTM_BLE_SEC_ENCRYPT;
                    result = wiced_bt_dev_set_encryption(peer_info[peer_index].remote_addr,
                                                         peer_info[peer_index].transport,
                                                         (void*)&sec_type);
                    if(WICED_BT_BUSY == result || WICED_BT_PENDING ==result)
                    {
                        printf("Encryption in progress, Check BTM_ENCRYPTION_STATUS_EVT for status!\n");
                    }
                    else if(WICED_BT_SUCCESS == result)
                    {
                        printf("Link already encrypted\n");
                    }
                    else if(WICED_BT_WRONG_MODE == result)
                    {
                        printf("Peer device is not connected\n");
                    }
                    else
                    {
                        printf("wiced_bt_dev_set_encryption failed, error code - %d\n", result);
                    }
                }
            }
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/**
 * Function Name: app_bt_write_peer_cccd
 *
 * Function Description:
 *   @brief Write the CCCD attribute of Notify characteristic of Hello Sensor
 *          device to either enable or disable notifications.
 *
 *   @param uint16_t cccd_val: cccd value to be be written to peer
 *   @param uint16_t conn_id: connection ID of the peer to send write req
 *
 *   @return wiced_bt_gatt_status_t
 *
 */
void app_bt_write_peer_cccd(uint8_t* cccd_val, uint16_t conn_id)
{
    wiced_bt_gatt_write_hdr_t app_write_cccd = {0};
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t notif_val[CCCD_LEN];

    notif_val[0] = cccd_val[0];
    notif_val[1] = cccd_val[1];
    printf("notif val 0 = %d, val 2 = %d \n", notif_val[0], notif_val[1]);
    app_write_cccd.auth_req = GATT_AUTH_REQ_NONE;
    app_write_cccd.handle = hs_discovery_data.hs_cccd_handle;
    printf("cccd handle = %d\n", app_write_cccd.handle);
    app_write_cccd.len = CCCD_LEN;
    app_write_cccd.offset = 0;
    status = wiced_bt_gatt_client_send_write(conn_id, GATT_REQ_WRITE, &app_write_cccd, notif_val, NULL);
    if(WICED_BT_GATT_SUCCESS != status)
    {
        printf("GATT Write request to CCCD failed, error code = %x \n", status);
    }
    else
    {
        printf("Write request to CCCD is successful\n");
    }
}

/**
 * Function Name: app_bt_write_peer_ead_cccd
 *
 * Function Description:
 *   @brief Write the CCCD attribute of EAD characteristic of Hello Sensor
 *          device to either enable or disable indication.
 *
 *   @param uint16_t cccd_val: cccd value to be be written to peer
 *   @param uint16_t conn_id: connection ID of the peer to send write req
 *
 *   @return wiced_bt_gatt_status_t
 *
 */
void app_bt_write_peer_ead_cccd(uint8_t* cccd_val, uint16_t conn_id)
{
    wiced_bt_gatt_write_hdr_t app_write_cccd = {0};
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t notif_val[CCCD_LEN];

    notif_val[0] = cccd_val[0];
    notif_val[1] = cccd_val[1];
    printf("indicate val 0 = %d, val 2 = %d \n", notif_val[0], notif_val[1]);
    app_write_cccd.auth_req = GATT_AUTH_REQ_NONE;
    app_write_cccd.handle = hs_discovery_data.hs_ead_cccd_handle;
    printf("cccd handle = %d\n", app_write_cccd.handle);
    app_write_cccd.len = CCCD_LEN;
    app_write_cccd.offset = 0;
    status = wiced_bt_gatt_client_send_write(conn_id, GATT_REQ_WRITE, &app_write_cccd, notif_val, NULL);
    if(WICED_BT_GATT_SUCCESS != status)
    {
        printf("GATT Write request to CCCD failed, error code = %x \n", status);
    }
    else
    {
        printf("Write request to CCCD is successful\n");
    }
}

/*******************************************************************************
 * GATT Utility for both Server and Client
 ******************************************************************************/
/**
 * Function Name : app_bt_store_peer_info
 *
 * Function Description:
 *   @brief This function stores the connection details of a peer
 *
 *   @param wiced_bt_gatt_connection_status_t* peer_conn_info: Refer to wiced_bt_gatt_connection_status_t
 *
 *   @return void
 *
 */
void app_bt_store_peer_info(wiced_bt_gatt_connection_status_t* peer_conn_info)
{
    uint8_t index;
    for (index = 0; index < MAX_NUM_CONNECTIONS; index++)
    {
        if(0 == peer_info[index].conn_id)
        {
            // printf("enter\n");
            peer_info[index].conn_id = peer_conn_info->conn_id;
            peer_info[index].link_role = peer_conn_info->link_role;
            peer_info[index].transport = peer_conn_info->transport;
            peer_info[index].connected = peer_conn_info->connected;
            peer_info[index].addr_type = peer_conn_info->addr_type;
            memcpy(peer_info[index].remote_addr, peer_conn_info->bd_addr, BD_ADDR_LEN);
            if(HCI_ROLE_CENTRAL == peer_info[index].link_role)
            {
                // printf("Central\n");
                peer_info[index].are_we_central = TRUE;
            }
            else if(HCI_ROLE_PERIPHERAL == peer_info[index].link_role)
            {
                // printf("not Central\n");
                peer_info[index].are_we_central = FALSE;
            }
            return;
        }
    }
}

/**
 * Function Name : app_bt_get_peer_index
 *
 * Function Description:
 *   @brief This function gets the array index for the connection ID passed
 *
 *   @param uint16_t conn_id: pass the required connection ID
 *
 *   @return uint8_t: array index where the connection ID is stored
 *
 */
uint8_t app_bt_get_peer_index(uint16_t conn_id)
{
    uint8_t index;
    for (index = 0; index < MAX_NUM_CONNECTIONS; index++)
    {
        if(conn_id == peer_info[index].conn_id)
            return index;
    }
    return MAX_NUM_CONNECTIONS;
}

/**
 * Function Name: app_bt_free_buffer
 *
 * Function Description:
 *   @brief This function frees up the memory buffer
 *
 *   @param uint8_t *p_data: Pointer to the buffer to be free
 *
 *   @return None
 */
void app_bt_free_buffer(uint8_t *p_buf)
{
    vPortFree(p_buf);
}

/**
 * Function Name: app_bt_alloc_buffer
 *
 * Function Description:
 *   @brief This function allocates a memory buffer.
 *
 *   @param int len: Length to allocate
 *
 *   @return None
 */
void* app_bt_alloc_buffer(int len)
{
    return pvPortMalloc(len);
}

/*******************************************************************************
 * GATT Utility for Server
 ******************************************************************************/
/**
 * Function Name : app_bt_find_by_handle
 *
 * Function Description:
 *   @brief Find attribute description by handle
 *
 *   @param uint16_t handle          : handle to look up
 *
 *   @return gatt_db_lookup_table_t  : pointer containing handle data
 *
 */
gatt_db_lookup_table_t  *app_bt_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (handle == app_gatt_db_ext_attr_tbl[i].handle)
        {
            return (&app_gatt_db_ext_attr_tbl[i]);
        }
    }
    return NULL;
}

/*******************************************************************************
 * GATT Utility for Client
 ******************************************************************************/
/**
 * Function Name: app_bt_read_peer_attributes
 *
 * Function Description:
 *   @brief Read all the characteristics of peer Hello Sensor
 *
 *   @param uint16_t conn_id: connection ID of the peer to send write req
 *
 *   @return void
 *
 */
void app_bt_read_peer_attributes(uint16_t conn_id)
{
    wiced_bt_gatt_status_t gatt_status;
    uint8_t peer_index;
    // printf("read attribute peer\n");
    peer_index = app_bt_get_peer_index(conn_id);

    if(peer_index >= MAX_NUM_CONNECTIONS)
    {
        printf("This app only supports upto 4 connections (including Central device)\n");
        return;
    }

    /* Use the flags to check if read request needs to be sent to GATT Server */
    if(!peer_info[peer_index].is_read_notify_char_cplt)
    {
        // printf("conn id = %d, i = %d, app_gatt_db_ext_attr_tbl[i].max_len = %d\n", conn_id, i, app_gatt_db_ext_attr_tbl[i].max_len);
        // printf("server HS notif handle = %d\n",hs_discovery_data.hs_char_val_notify_handle);
        /* Send read request to Notify Value Attribute */
        gatt_status = wiced_bt_gatt_client_send_read_handle(conn_id, hs_discovery_data.hs_char_val_notify_handle, 0, read_resp_data, sizeof(wiced_bt_gatt_operation_complete_t), GATT_AUTH_REQ_NONE);
        printf("gatt)status notif = 0x%02x\n", gatt_status);
        // printf("Address of p_data = %d\n", &app_gatt_db_ext_attr_tbl[i].p_data);
    }
    else if(!peer_info[peer_index].is_read_cccd_cplt)
    {
        /* Send read request to CCCD Attribute */
        gatt_status = wiced_bt_gatt_client_send_read_handle(conn_id, hs_discovery_data.hs_cccd_handle, 0, read_resp_data, sizeof(wiced_bt_gatt_operation_complete_t), GATT_AUTH_REQ_NONE);
        printf("gatt)status cccd = 0x%02x\n", gatt_status);
    }
    else if(!peer_info[peer_index].is_read_blink_char_cplt)
    {
        /* Send read request to Blink Attribute */
        gatt_status = wiced_bt_gatt_client_send_read_handle(conn_id, hs_discovery_data.hs_char_val_blink_handle, 0, read_resp_data, sizeof(wiced_bt_gatt_operation_complete_t), GATT_AUTH_REQ_NONE);
        printf("gatt)status read Blink = 0x%02x\n", gatt_status);
    }
    else if(!peer_info[peer_index].is_read_ead_keys_cplt)
    {
        /* Send read request to BLE EAD Keys Attribute */
        gatt_status = wiced_bt_gatt_client_send_read_handle(conn_id, hs_discovery_data.hs_char_val_ead_key_handle, 0, read_resp_data, sizeof(wiced_bt_gatt_operation_complete_t), GATT_AUTH_REQ_NONE);
        printf("gatt)status read ead = 0x%02x\n", gatt_status);
    }
    else if(!peer_info[peer_index].is_read_ead_cccd_cplt)
    {
        /* Send read request to BLE EAD CCCD Attribute */
        gatt_status = wiced_bt_gatt_client_send_read_handle(conn_id, hs_discovery_data.hs_ead_cccd_handle, 0, read_resp_data, sizeof(wiced_bt_gatt_operation_complete_t), GATT_AUTH_REQ_NONE);
        printf("gatt)status read ead cccd = 0x%02x\n", gatt_status);
    }
}

/**
 * Function Name: app_bt_get_peripheral_conn_id
 *
 * Function Description:
 *   @brief This function gets the connection ID of the peripheral device
 *          to which GATT write from Central needs to be passed on.
 *
 *   @param uint16_t handle: Handle in the local attribute which the Central has sent a GATT request
 *
 *   @return uint16_t: connection ID of the peripheral
 *
 */
uint16_t app_bt_get_peripheral_conn_id(uint16_t handle)
{
    for(uint8_t i = 0; i < MAX_PERIPHERALS; i++)
    {
        /* Check if the input handle is within start and end handles of the service */
        if((local_gatt_db_info[i].service_handle + DEVICE_NAME_ATT_OFFSET) >= handle && (local_gatt_db_info[i].service_handle) <= handle)
        {
            return local_gatt_db_info[i].conn_id;
        }
    }
/*    if(0x09 == handle){
        printf("Service changed Indications request received by central but not handled in this example \n");
    }*/
    printf("Connection ID corresponding to the Handle not found! \n");
    return 0;
}

/**
 * Function Name: app_bt_update_gatt_db
 *
 * Function Description:
 *   @brief Update the Local GATT data base, 3 fields can get updates:
 *          ATT_CCCD: If peer GATT Client writes to the CCCD
 *          ATT_BLINK: If peer GATT Client writes to the blink characteristic.
 *          ATT_DEVICE_NAME: Peer GAP Peripheral name stored in the attribute so
 *                           that the Central can see which peripheral it is interacting with.
 *
 *   @param uint8_t att_val_id: Identity number to the exact attribute to be written.
 *   @param uint16_t conn_id: connection ID of peripheral device
 *   @param uin16_t handle: handle of the local GATT DB to which central performed the write operation
 *
 *
 *   @return wiced_bt_gatt_status_t
 *
 */
void app_bt_update_gatt_db( uint8_t att_val_id, uint16_t conn_id, uint16_t handle)
{
    uint16_t peripheral_conn_id;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_write_hdr_t app_write_blink = {0};
    switch(att_val_id)
    {
        case ATT_CCCD:
        {
            peripheral_conn_id = app_bt_get_peripheral_conn_id(handle);
            for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
            {
                if(handle == app_gatt_db_ext_attr_tbl[i].handle)
                {
                    app_bt_write_peer_cccd(app_gatt_db_ext_attr_tbl[i].p_data, peripheral_conn_id);
                    break;
                }
            }
        }
        break;
        case ATT_BLINK:
        {
            // printf("ATT blink enter\n");
            peripheral_conn_id = app_bt_get_peripheral_conn_id(handle);
            printf("Peripheral conn id = %d\n", peripheral_conn_id);
            app_write_blink.handle = hs_discovery_data.hs_char_val_blink_handle;
            app_write_blink.len = app_hello_sensor_1_blink_len;
            app_write_blink.offset = 0;
            app_write_blink.auth_req = GATT_AUTH_REQ_NONE;
            for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
            {
                if(handle == app_gatt_db_ext_attr_tbl[i].handle)
                {
                    // printf("app_gatt_db_ext_attr_tbl->p_data[0] = %d\n", app_gatt_db_ext_attr_tbl[i].p_data[0]);
                if((app_gatt_db_ext_attr_tbl[i].p_data[0] <= 0x09 && app_gatt_db_ext_attr_tbl[i].p_data[0] > 0x0) && (peripheral_conn_id != 0x0))
                {
                    gatt_status = wiced_bt_gatt_client_send_write(peripheral_conn_id , GATT_REQ_WRITE, &app_write_blink, app_gatt_db_ext_attr_tbl[i].p_data, NULL);
                    if(WICED_BT_GATT_SUCCESS != gatt_status)
                    {
                        printf("GATT Write request failed, error code = %x \n", gatt_status);
                    }
                    else
                    {
                        printf("write success blink\n");
                    }
                }
                else
                {
                    printf("Write value entered is not in range or Hello Sensor device is not yet connected\n");
                }
                    break;
                }
            }
        }
        break;
        case ATT_DEVICE_NAME:
        {
            uint8_t att_handle;
            /* Search for a free custom Hello Sensor service. So that, the new connection can be allocated to the free service */
            for(uint8_t index = 0; index < MAX_PERIPHERALS; index++)
            {
                if(TRUE == local_gatt_db_info[index].is_service_free)
                {
                    /* Update the flag in local gatt db info to indicate that the service will be occupied by current connection */
                    local_gatt_db_info[index].is_service_free = FALSE;
                    att_handle = local_gatt_db_info[index].service_handle + DEVICE_NAME_ATT_OFFSET;
                    local_gatt_db_info[index].conn_id = conn_id;
                    /* Populate the GATT DB with peer device name, so that a user can identify the service associated with peripheral */
                    for(uint8_t i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
                    {
                        if(app_gatt_db_ext_attr_tbl[i].handle == att_handle)
                        {
                            app_gatt_db_ext_attr_tbl[i].p_data = adv_device_name;
                            break;
                        }
                    }
                    break;
                }
            }
        }
        break;
    }
}

/**
 * Function Name: app_bt_local_gatt_db_info_init
 *
 * Function Description:
 *   @brief This function populates the structure that holds local gatt db
 *          information with custom hello sensor service handles
 *
 *   @param None
 *
 *   @return None
 *
 */
void app_bt_local_gatt_db_info_init(void)
{
    local_gatt_db_info[0].service_handle = HDLS_HELLO_SENSOR_1;
    local_gatt_db_info[0].is_service_free = TRUE;
}

/**
 * Function Name: app_bt_get_service_handle_from_conn_id
 *
 * Function Description:
 *   @brief This function finds the service handle of local GATT DB value for the associated peer Hello Sensor
 *
 *   @param uint16_t conn_id: connection ID of peer Hello Sensor
 *
 *   @return None
 *
 */
uint16_t app_bt_get_service_handle_from_conn_id(uint16_t conn_id)
{
    for(uint8_t i = 0; i < MAX_PERIPHERALS; i++)
    {
        if(local_gatt_db_info[i].conn_id == conn_id)
        {
            return local_gatt_db_info[i].service_handle;
        }
    }

    printf("Service Handle not found for given connection ID\n");
    return 0;
}
