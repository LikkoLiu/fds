#include "uart_gps.h"

#define UART_GPS_TAG "uart-gps"
#define RX_BUF_SIZE 512

static char *cut_substr(char *dest, char *src, char start, int n)
{
    char *p = dest;
    char *q = src;
 
    q += start;
    while(n--) *(p++ )= *(q++);
    *(p++) = '\0';
    return dest;
}

esp_err_t uartgpsdevInit(void)
{
    esp_err_t err = ESP_OK;
    esp_log_level_set(UART_GPS_TAG, ESP_LOG_INFO);

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    err = uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    err = uart_param_config(UART_NUM_2, &uart_config);
    err = uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    return err;
}

void GpsTask(void *arg)
{
    char *data = NULL;
    char *dest = NULL;
    data = (char *)malloc(RX_BUF_SIZE+1);
    dest = (char *)malloc(16);

    char *row;
    char *pos1;
    char *pos2;

    gps_data_t gps_data;
    /* 防止首次复位时，由于不明原因二次复位后，才能正常运行 */
    const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
    while (1)
    {
        bzero(&gps_data, sizeof(gps_data));
        bzero(&gps_data.lat, sizeof(gps_data.lat));
        bzero(&gps_data.lon, sizeof(gps_data.lat));

        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGD(UART_GPS_TAG, "Read %d bytes: '%s'", rxBytes, data);

            // 取经纬度
            row = strstr(data, "$GNGGA");
            pos1 = strchr(row, ',');      // UTC时间...
            pos2 = strchr(pos1 + 1, ','); // 纬度...
            pos1 = strchr(pos2 + 1, ','); // 纬度方向...
            cut_substr(gps_data.lat, pos2, 1, pos1 - pos2); // 纬度
            pos2 = strchr(pos1 + 1, ',');                   // 经度...
            pos1 = strchr(pos2 + 1, ',');                   // 经度方向...
            cut_substr(gps_data.lon, pos2, 1, pos1 - pos2); // 经度

            if ((gps_data.lat[0] == ',')||(gps_data.lon[0] == ','))
            {
                ESP_LOGI(UART_GPS_TAG, "failed to obtain GPS data");
            }
            else
            {
                ESP_LOGI(UART_GPS_TAG, "lat=%s lon=%s", gps_data.lat, gps_data.lon);
            }

        }
    }

    // Never reach here
    free(data);
    vTaskDelete(NULL);
}