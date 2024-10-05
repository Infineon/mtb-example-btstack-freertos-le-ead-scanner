#include "wiced_bt_ble.h"
#include "ble_ead.h"
#include "wiced_bt_trace.h"
#include "app_bt_utils.h"
#include "cy_cryptolite_trng.h"

uint32_t randomizer = 0; // incrementing counter

uint8_t adv_plaintext[20] = {0}; // User ADV data to encrypt
uint8_t adv_to_encrypt[5 + 20 + 4] = {0};   // No exceed 31 bytes for legacy ble adv
extern wiced_bt_ble_key_material_t ble_ead_key_material;

static void ble_ead_advertiser_fill_adv_data(void)
{
    uint8_t* p = adv_plaintext;
    uint8_t idx = 0;
    static uint8_t manuf_data = 0;

    /* BLE ADV payload is LTV type */
    p[0] = 10; // Length
    p[1] = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;   //Type
    memcpy(&p[2], "B_EAD_ENC", 9); //Value
    idx += 11;

    p[idx] = 3;
    p[idx + 1] = BTM_BLE_ADVERT_TYPE_SOLICITATION_SRV_UUID;
    p[idx + 2] = 0x88;
    p[idx + 3] = 0x2B;
    idx += 4;

    p[idx] = 4;
    p[idx + 1] = BTM_BLE_ADVERT_TYPE_MANUFACTURER;
    p[idx + 2] = 0x09;
    p[idx + 3] = 0x00;
    p[idx + 4] = manuf_data ++;
}

void ble_ead_advertiser_init(void)
{
    uint8_t *p = NULL;
    uint32_t mic = 0;
    wiced_bt_ble_advert_elem_t adv_elem[3], *p_adv_elem;
    uint8_t num_elem = 0;
    cy_en_cryptolite_status_t cry_rslt = CY_CRYPTOLITE_SUCCESS;

    cry_rslt = Cy_Cryptolite_Trng_Init(CRYPTOLITE, NULL);
    if(cry_rslt == CY_CRYPTOLITE_SUCCESS)
    {
        Cy_Cryptolite_Trng(CRYPTOLITE, &randomizer);
        Cy_Cryptolite_Trng_DeInit(CRYPTOLITE);
    }
    else
    {
        printf("[%s] trng generate failed, status:0x%x\n", __FUNCTION__, cry_rslt);
        return;
    }

    ble_ead_advertiser_fill_adv_data();

    p = adv_to_encrypt;
    UINT32_TO_STREAM(p, randomizer); // copy 4 bytes of the count as randomizer

    mic = wiced_bt_ble_encrypt_adv_packet(ble_ead_key_material.session_key, // session key
                                         ble_ead_key_material.iv,          // iv
                                         adv_to_encrypt,               // address of randomizer
                                         adv_plaintext,                // plaintext to be encrypted
                                         adv_to_encrypt+ 5,            // address to copy encrypted data, same length as of plaintext
                                         sizeof(adv_plaintext));       // length of the plaintext

    printf("\n[%s] mic:0x%lx, encypted:", __FUNCTION__, mic);
    print_array(adv_to_encrypt, sizeof(adv_to_encrypt) - 4);

    p = adv_to_encrypt + 5 + sizeof(adv_plaintext);
    UINT32_TO_BE_STREAM(p, mic);

    adv_elem[0].advert_type = BTM_BLE_ADVERT_TYPE_EAD;
    adv_elem[0].len = 5 + sizeof(adv_plaintext) + 4;
    adv_elem[0].p_data = adv_to_encrypt;
    num_elem = 1;
    p_adv_elem = &adv_elem[0];

    wiced_bt_ble_set_raw_advertisement_data(num_elem, p_adv_elem);

    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, BLE_ADDR_PUBLIC, NULL);
}

wiced_result_t ble_ead_advertiser_state_changed(wiced_bt_management_evt_t event,
                                                wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t rst = WICED_BT_SUCCESS;
    switch(event)
    {
        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            printf("[%s] ble_advert_state_changed:%d \n", __FUNCTION__, p_event_data->ble_advert_state_changed);
            if(p_event_data->ble_advert_state_changed == BTM_BLE_ADVERT_OFF)
            {
                ble_ead_advertiser_init();
            }
            break;
    }

    return rst;
}
