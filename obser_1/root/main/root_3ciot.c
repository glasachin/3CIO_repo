/**
 * @file root_3ciot.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-04
 * 
 * @copyright Copyright (c) 2022
 * 
 *  1. This file reads the data in JSON format from serial monitor and send ping to the specified node
 */

#include "mdf_common.h"
#include "mwifi.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "mupgrade.h"
#include <stdlib.h>
#include "stdio.h"
#include "esp_timer.h"
#include "cJSON.h"

// #define MEMORY_DEBUG
static const char *TAG = "3CIOT";

//--------Serial Port Config Start----------
static const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_1) //for UART0
#define RXD_PIN (GPIO_NUM_3) // for UART0

void serial_port_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

//-------------END: Serial Port Config---------

//-------------Other Variables---------------
char *MAC_add, *cmd; 
//---------------------------------------------

static void root_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                      = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0};

    MDF_LOGI("Root is running");

    for (int i = 0;; ++i) {
        if (!mwifi_is_started()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        MDF_LOGI("Root receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

        size = sprintf(data, "(%d) Hello node!", i);
        ret = mwifi_root_write(src_addr, 1, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
        MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
    }

    MDF_LOGW("Root is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}


//--------Writing From Root to the specific Node---------
static void write_task(uint8_t *dest_addr, char *data_sent)
{
    size_t size                     = 0;
    int count                       = 0;
    char *data                      = NULL;
    mdf_err_t ret                   = MDF_OK;
    mwifi_data_type_t data_type     = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
	//uint8_t dest_addr[MWIFI_ADDR_LEN] = {0x3C,0x61,0x05,0x14,0xC7,0xE8};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    //size = asprintf(&data, "{\"src_addr\": \"" MACSTR "\",\"data\": \"Hello From Root from JCBRO\",\"count\": %d}",
    //                    MAC2STR(sta_mac), count++);

    size = asprintf(&data,"%s",data_sent);
	ret = mwifi_write(dest_addr, &data_type, data, size, true);        

    MDF_FREE(data);
}


/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(void *timer)
{
    uint8_t primary                 = 0;
    wifi_second_chan_t second       = 0;
    mesh_addr_t parent_bssid        = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u", primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            break;

        default:
            break;
    }

    return MDF_OK;
}



void init_all()
{
    //---Initializing Serial Port-----
    serial_port_init();

}

void app_main()
{
    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config   = {
        .channel   = CONFIG_MESH_CHANNEL,
        .mesh_id   = CONFIG_MESH_ID,
        .mesh_type = CONFIG_DEVICE_TYPE,
    };

    //-----variable area--------------
    // uint8_t --> aka unsigned char
    unsigned char* data = (unsigned char*) malloc(RX_BUF_SIZE+1);
    uint8_t MAC_addr[6] = { 0 }; // To store the received MAC address

    //----------Initialization-------
    init_all();

    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    /**
     * @brief Initializing root_task
     */
    xTaskCreate(root_task, "root_task", 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    

    // To Print system info at regualr interval
    TimerHandle_t timer = xTimerCreate("print_system_info", 100000 / portTICK_RATE_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);


    //---------loop in main app to receive data from serial port-------
    while(1)
    {
        const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 100 / portTICK_RATE_MS);

        if(rxBytes > 0)
        {
            //printf("Received Bytes: %d\n", rxBytes);
            // Printing received data back on the serial port
            // for(int i = 0; i < rxBytes; i++)
            // {
            //     printf("%c",data[i]);
            // }
            // printf("\n");
            // Write data back to the UART
        // uart_write_bytes(UART_NUM_0, (const char *) data, rxBytes);
        // print("\n");

        MAC_add = strtok((char *)data, ";");
        cmd = strtok(NULL, ";");
        printf("MAC: %s\n", MAC_add);
        printf("Command: %s\n", cmd);

        // convert MAC string to the compatible MAC address array
        sscanf(MAC_add, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",&MAC_addr[0], &MAC_addr[1], &MAC_addr[2],&MAC_addr[3], &MAC_addr[4], &MAC_addr[5] );
        
        
        }


        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

}

