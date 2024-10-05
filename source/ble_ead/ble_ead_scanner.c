#include "wiced_bt_ble.h"
#include "ble_ead.h"
#include "wiced_bt_trace.h"
#include "app_bt_utils.h"

extern wiced_bt_ble_key_material_t ble_ead_key_material;

void ble_ead_scan_result_cback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    uint8_t *p_randomizer;
    uint8_t plaintext_len = 0;

    p_randomizer = wiced_bt_ble_check_advertising_data(p_adv_data, BTM_BLE_ADVERT_TYPE_EAD, &plaintext_len);
    if (p_randomizer)
    {
        wiced_bt_ble_key_material_t *p_mat = &ble_ead_key_material; // get the key material for the bd_addr
        if (p_mat)
        {
            uint8_t *p_plaintext;
            uint8_t *p_encrypted ;
            uint8_t *p_mic;

            printf("\n[%s] rcv encrypted:", __FUNCTION__);
            print_array(p_randomizer, plaintext_len - 4);

            p_encrypted = p_randomizer + 5;
            p_plaintext = p_randomizer + 5; // write the plaintext output to the same addr
            plaintext_len -= 5 + 4; // sizeof(randomizer) + sizeof(mic)

            // get the plaintext and mic from the encrypted data
            uint32_t mic = wiced_bt_ble_decrypt_adv_packet(p_mat->session_key,  // session key
                                                            p_mat->iv,           // iv
                                                            p_randomizer,        // randomizer from the incoming packet
                                                            p_encrypted,         // address of the encrypted data
                                                            p_plaintext,         // address of the decrypted plaintext
                                                            plaintext_len);      // length of the encrypted payload
            // get the mic from the packet
            uint32_t packet_mic;
            p_mic = p_plaintext + plaintext_len;
            BE_STREAM_TO_UINT32(packet_mic, p_mic);

            printf("\n[%s] mic:0x%lx pkt_mic:0x%lx, plaintext_len:%d\n", __FUNCTION__, mic, packet_mic, plaintext_len);

            if(packet_mic == mic)
            {
                // Packet MIC matches the calculated MIC hence plaintext is valid
                // set the adv and length pointer
                printf("\n[%s] decrypted data:", __FUNCTION__);
                print_array(p_plaintext, plaintext_len);
            }
        }
    }
}

void ble_ead_scanner_init(void)
{
    printf("[%s]", __FUNCTION__);
    wiced_bt_ble_observe(1, 0, ble_ead_scan_result_cback);
}