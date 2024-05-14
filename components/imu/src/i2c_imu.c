#include "i2c_imu.h"

#define I2C_IMU_TAG "i2c-imu"

TaskHandle_t xImuHandle = NULL;

// Madgwick madgwick;
struct bmi160_dev sensor;

static float accel_sensitivity;
static float gyro_sensitivity;

static void vSaveImuData(float *fDataRoll, float *fDataPitch, float *fDataYaw)
{
    fArrRoll[usPtrArrImu] = *fDataRoll;
    fArrPitch[usPtrArrImu] = *fDataPitch;
    fArrYaw[usPtrArrImu] = *fDataYaw;

    usPtrArrImu++;
    if (N_SAMPLES == usPtrArrImu)
    {
        usPtrArrImu = 0;
    }
}
/**
 * IMU 依赖的 I2C 外设初始化
 */
static esp_err_t i2cimudevInit(void)
{
    esp_log_level_set(I2C_IMU_TAG, I2C_IMU_LOG);

    int i2c_imu_port = I2C_IMU_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_IMU_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_IMU_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_IMU_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_imu_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    err = i2c_driver_install(i2c_imu_port, conf.mode, I2C_IMU_RX_BUF_DISABLE, I2C_IMU_TX_BUF_DISABLE, 0);

    ESP_LOGI(I2C_IMU_TAG, "i2c(num %d) initialized %s", i2c_imu_port, (err == ESP_OK) ? "successfully" : "fail");
    return err;
}

/**
 * 选择一个8位设备寄存器。
 * @param devAddr I2C从设备地址
 * @param regAddr 首先注册要读取的regAddr
 */
static void SelectRegister(uint8_t devAddr, uint8_t regAddr)
{
    ESP_LOGD(__FUNCTION__, "devAddr=0x%x regAddr=0x%x", devAddr, regAddr);
    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

/**
 * 从 8 位设备寄存器读取多个字节。
 * @param devAddr I2C从设备地址
 * @param regAddr 首先注册要读取的regAddr
 * @param length 要读取的字节数
 * @param data 存储读取数据的缓冲区
 * @param timeout 可选的读取超时（以毫秒为单位）（0 表示禁用，不使用 I2Cdev::readTimeout 中的默认类值）
 * @return I2C_TransferReturn_TypeDef
 */
static uint8_t user_i2c_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length)
{
    i2c_cmd_handle_t cmd;
    SelectRegister(devAddr, regAddr);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, 1));

    if (length > 1)
        ESP_ERROR_CHECK(i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

#if __DEBUG__
    ESP_LOGI(__FUNCTION__, "regAddr=0x%x length=%d", regAddr, length);
    for (int i = 0; i < length; i++)
    {
        ESP_LOGI(__FUNCTION__, "data[%d]=0x%x", i, data[i]);
    }
#endif

    return 0;
}

/**
 * 将单个字节写入 8 位设备寄存器。
 * @param devAddr I2C从设备地址
 * @param regAddr 要写入的寄存器地址
 * @param length 要写入的字节数
 * @param data 要写入的字节数组
 * @return 操作状态（true=成功）
 */
static uint8_t user_i2c_write(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length)
{
#if __DEBUG__
    ESP_LOGI(__FUNCTION__, "regAddr=0x%x length=%d", regAddr, length);
    for (int i = 0; i < length; i++)
    {
        ESP_LOGI(__FUNCTION__, "data[%d]=0x%x", i, data[i]);
    }
#endif

    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, 1));
    if (length > 1)
        ESP_ERROR_CHECK(i2c_master_write(cmd, data, length - 1, 0));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length - 1], 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    return 0;
}

static void user_delay_ms(uint32_t period)
{
#if __DEBUG__
    ESP_LOGI(__FUNCTION__, "period=%" PRIu32, period);
#endif
    esp_rom_delay_us(period * 1000);
}

/**
 * 获取开机后的时间（以秒为单位）
 * 兼容ROS的time.toSec()函数
 */
static double TimeToSec()
{
    int64_t _time = esp_timer_get_time(); // Get time in microseconds since boot
    double __time = (double)_time / 1000000;
    return __time;
}

esp_err_t bmi160Init(void)
{
    sensor.id = IMU_SENSOR_ADDR;
    sensor.intf = BMI160_I2C_INTF;
    sensor.read = user_i2c_read;
    sensor.write = user_i2c_write;
    sensor.delay_ms = user_delay_ms;

    uint8_t ret = bmi160_init(&sensor);
    if (ret != BMI160_OK)
    {
        ESP_LOGE(I2C_IMU_TAG, "BMI160 initialization fail %d", ret);
    }
    ESP_LOGI(I2C_IMU_TAG, "Chip ID 0x%X", sensor.chip_id);

    // Config Accel
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G; // -2 --> +2[g]
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    accel_sensitivity = 16384.0; // g

    // Config Gyro
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    // sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS; // -250 --> +250[Deg/Sec]
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
    gyro_sensitivity = 131.2; // Deg/Sec

    ret = bmi160_set_sens_conf(&sensor);
    ESP_LOGI(I2C_IMU_TAG, "bmi160 set_sens_conf %s", (ret == ESP_OK) ? "successfully" : "fail");

    return ret;
}

esp_err_t I2cImuInit(void)
{
    esp_err_t err = ESP_OK;
    esp_log_level_set(I2C_IMU_TAG, I2C_IMU_LOG);

    err = i2cimudevInit();

    err = bmi160Init();
    ESP_LOGI(I2C_IMU_TAG, "imu initialization %s \r\n", (err == ESP_OK) ? "successfully" : "fail");

    return err;
}

void ImuTask(void *pvParameters)
{
    esp_err_t err = ESP_OK;
    double last_time_ = TimeToSec();

    usPtrArrImu = 0;

    // Madgwick 算法初始化
    MadgwickInit();

    for (uint8_t elasped = 0;; elasped++)
    {
        struct bmi160_sensor_data accel;
        struct bmi160_sensor_data gyro;
        err = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
        if (err != BMI160_OK)
        {
            ESP_LOGE(I2C_IMU_TAG, "BMI160 get_sensor_data fail %d", err);
            vTaskDelete(NULL);
        }
        ESP_LOGD(I2C_IMU_TAG, "accel=%d %d %d gyro=%d %d %d", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

        // Convert relative to absolute
        float ax = (float)accel.x / accel_sensitivity;
        float ay = (float)accel.y / accel_sensitivity;
        float az = (float)accel.z / accel_sensitivity;
        float gx = (float)gyro.x / gyro_sensitivity;
        float gy = (float)gyro.y / gyro_sensitivity;
        float gz = (float)gyro.z / gyro_sensitivity;
        float task_roll = 0.0, task_pitch = 0.0, task_yaw = 0.0;

        // Get the elapsed time from the previous
        float dt = (TimeToSec() - last_time_);
        last_time_ = TimeToSec();

        updateIMU(gx, gy, gz, ax, ay, az, dt);
        eulerAngles(&task_roll, &task_pitch, &task_yaw);

        vSaveImuData(&task_roll, &task_pitch, &task_yaw);
        ESP_LOGI(I2C_IMU_TAG, "roll=%f pitch=%f yaw=%f dt=%f", task_roll, task_pitch, task_yaw, dt);

        vTaskDelay(20 / portTICK_PERIOD_MS);
    } // end while

    // Never reach here
    vTaskDelete(NULL);
}
