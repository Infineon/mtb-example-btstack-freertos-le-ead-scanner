#ifndef BLE_EAD_H
#define BLE_EAD_H

#include "wiced_bt_dev.h"

void ble_ead_advertiser_init(void);
wiced_result_t ble_ead_advertiser_state_changed(wiced_bt_management_evt_t event,
                                                wiced_bt_management_evt_data_t *p_event_data);
void ble_ead_scanner_init(void);

#endif
