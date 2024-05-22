/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"
#include "../include/hid_usage_keyboard.h"


//mod by A open
typedef struct {
    uint8_t key_code;
    uint8_t counter;
    bool initial_delay_passed;
} key_state_t;

#define MAX_KEYS 6
static key_state_t key_states[MAX_KEYS] = {0};
static uint8_t num_keys = 0;
typedef struct {
    enum key_state {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

//mod by A close

#define KEYBOARD_ENTER_MAIN_CHAR    '\r'

const uint8_t keycode2ascii [113][2] = {
    {0, 0}, /* HID_KEY_NO_PRESS        */
    {0, 0}, /* HID_KEY_ROLLOVER        */
    {0, 0}, /* HID_KEY_POST_FAIL       */
    {0, 0}, /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER           */
    {0, 0}, /* HID_KEY_ESC             */
    {'\b', 0}, /* HID_KEY_DEL             */
    {0, 0}, /* HID_KEY_TAB             */
    {' ', ' '}, /* HID_KEY_SPACE           */
    {'-', '_'}, /* HID_KEY_MINUS           */
    {'=', '+'}, /* HID_KEY_EQUAL           */
    {'[', '{'}, /* HID_KEY_OPEN_BRACKET    */
    {']', '}'}, /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'}, /* HID_KEY_BACK_SLASH      */
    {'\\', '|'}, /* HID_KEY_SHARP           */  // HOTFIX: for NonUS Keyboards repeat HID_KEY_BACK_SLASH
    {';', ':'}, /* HID_KEY_COLON           */
    {'\'', '"'}, /* HID_KEY_QUOTE           */
    {'`', '~'}, /* HID_KEY_TILDE           */
    {',', '<'}, /* HID_KEY_LESS            */
    {'.', '>'}, /* HID_KEY_GREATER         */
    {'/', '?'}, /* HID_KEY_SLASH           */
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {'\b', 0}, /*HID_KEY_DELETE*/
    {0, 0},
    {0, 0},
    {'R', 'R'}, /*HID_KEY_RIGHT*/
    {'L', 'L'}, /*HID_KEY_LEFT*/
    {'D', 'D'}, /*HID_KEY_DOWN*/
    {'U', 'U'}, /*HID_KEY_UP*/
    {0, 0},
    {'/', '/'}, /*HID_KEY_KEYPAD_DIV*/
    {'*', '*'}, /*HID_KEY_KEYPAD_MUL*/
    {'-', '-'}, /*HID_KEY_KEYPAD_SUB*/
    {'+', '+'}, /*HID_KEY_KEYPAD_ADD*/
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /*HID_KEY_KEYPAD_ENTER*/
    {'1', '1'},
    {'2', '2'},
    {'3', '3'},
    {'4', '4'},
    {'5', '5'},
    {'6', '6'},
    {'7', '7'},
    {'8', '8'},
    {'9', '9'},
    {'0', '0'},
    {'\b', '.'},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}      /* HID_KEY_F21*/
};


static const char *TAG = "ESP_HIDH_DEMO";


/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src,
                             uint8_t key,
                             unsigned int length)
{
    for (unsigned int i = 0; i < length; i++) {
        if (src[i] == key) {
            return true;
        }
    }
    return false;
}



/**
 * @brief HID Keyboard modifier verification for capitalization application (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier)
{
    if ((modifier && HID_LEFT_SHIFT) ||
            (modifier && HID_RIGHT_SHIFT)) {
        return true;
    }
    return false;
}


/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char)
{
    if (!!key_char) {
        //edited code
        switch (key_char) {
            default:
                putchar(key_char); 
        }
        //putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
        if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
            putchar('\n');
        }
#endif // KEYBOARD_ENTER_LF_EXTEND
        fflush(stdout);
    }
}


/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier,
        uint8_t key_code,
        unsigned char *key_char)
{
    uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

    if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_F21)) {       //modified by as
        *key_char = keycode2ascii[key_code][mod];
    } else {
        // All other key pressed

        return false;
    }

    return true;
}



/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event)
{
    unsigned char key_char;

   // hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

    if (KEY_STATE_PRESSED == key_event->state) {
        if (hid_keyboard_get_char(key_event->modifier,
                                  key_event->key_code, &key_char)) {

            hid_keyboard_print_char(key_char);

        }
    }
}



void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
        key_event_t key_event;
        num_keys = 0;

        for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {
            if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
                !key_found((&param->input.data[i+1]), prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
                key_event.key_code = prev_keys[i];
                key_event.modifier = 0;
                key_event.state = KEY_STATE_RELEASED;
                key_event_callback(&key_event);
                key_states[i].key_code = 0;
                key_states[i].counter = 0;
                key_states[i].initial_delay_passed = false;
            }

            if (param->input.data[i+1] > HID_KEY_ERROR_UNDEFINED) {
                key_states[num_keys].key_code = param->input.data[i+1];
                key_states[num_keys].counter = 0;
                key_states[num_keys].initial_delay_passed = false;
                num_keys++;
                if (!key_found(prev_keys, param->input.data[i+1], HID_KEYBOARD_KEY_MAX)) {
                    key_event.key_code = param->input.data[i+1];
                    key_event.modifier = param->input.data[0];
                    key_event.state = KEY_STATE_PRESSED;
                    key_event_callback(&key_event);
                }
            }
        }

        memcpy(prev_keys, &param->input.data[1], HID_KEYBOARD_KEY_MAX);

    /*
        int length_id = param->input.length;
        int mod = param->input.data[0];
        int key = param->input.data[1];
        unsigned int key_char; */
        //ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        //ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        //printf("Input event\n");
       /* for(int i = 0; i<length_id; i++){
            printf("%d: ", i);
            printf(" %d ", param->input.data[i]);
        }*/
        //printf("\n");
        /*
        if(param->input.data[0] > 0 && param->input.data[1] > 0){
            //printf(" %d :", param->input.data[1]);
            //printf(" %c \n", param->input.data[1]);
            key_char = keycode2ascii[param->input.data[1]][1];
            //printf("mod %d\n", key_char);
            putchar((unsigned int)key_char);
        }
        else if(param->input.data[1] > 0){
            key_char = keycode2ascii[param->input.data[1]][0];
            //printf("no mod %d\n", key_char);
            putchar((unsigned int)key_char);
        } */
        //printf("")
        //printf("\n");
        //fflush(stdout); 
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}


/**
 * @brief Handle repeated keys for keyboard
 */
static void handle_repeated_keys(void *arg)
{
    const TickType_t xInitialDelay = pdMS_TO_TICKS(300);  // Initial delay of 300 ms
    const TickType_t xRepeatDelay = pdMS_TO_TICKS(50);    // Repeat delay of 50 ms

    while (1) {
        for (int i = 0; i < num_keys; i++) {
            if (key_states[i].key_code > HID_KEY_ERROR_UNDEFINED) {
                if (!key_states[i].initial_delay_passed) {
                    // Wait for the initial delay before starting repeated key presses
                    vTaskDelay(xInitialDelay);
                    key_states[i].initial_delay_passed = true;
                } else {
                    key_states[i].counter++;
                    if (key_states[i].counter >= 5) {  // Adjust the counter threshold as needed
                        key_event_t key_event;
                        key_event.key_code = key_states[i].key_code;
                        key_event.modifier = 0;
                        key_event.state = KEY_STATE_PRESSED;
                        key_event_callback(&key_event);
                        key_states[i].counter = 0;  // Reset counter after printing
                    }
                }
            }
        }
        vTaskDelay(xRepeatDelay);
    }
}




#define SCAN_DURATION_SECONDS 5

void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr) {
            //open the last result
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
        
        //free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}





void app_main(void)
{
    TaskHandle_t repeated_keys_task_handle;
    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);
    xTaskCreate(handle_repeated_keys, "repeated_keys", 2048, NULL, 2, &repeated_keys_task_handle);
}