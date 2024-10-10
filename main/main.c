#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

#define UART_ID uart0

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

// Estrutura para armazenar os dados de aceleração
typedef struct {
    int8_t x;
    int8_t y;
} data_mouse;


QueueHandle_t xQueueMouse;

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);


    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        FusionVector gyroscope_converted;
        gyroscope_converted.axis.x = gyro[0] / 131.0f;
        gyroscope_converted.axis.y = gyro[1] / 131.0f;
        gyroscope_converted.axis.z = gyro[2] / 131.0f;
        FusionVector accelerometer_converted;
        accelerometer_converted.axis.x = acceleration[0] / 16384.0f;
        accelerometer_converted.axis.y = acceleration[1] / 16384.0f;
        accelerometer_converted.axis.z = acceleration[2] / 16384.0f;
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope_converted, accelerometer_converted, 0.01f);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        int x_data =  euler.angle.pitch;
        uart_putc_raw(UART_ID, 0);
        uart_putc_raw(UART_ID, x_data & 0xFF);
        uart_putc_raw(UART_ID, ( x_data >> 8) & 0xFF);
        uart_putc_raw(UART_ID, -1);

        int y = euler.angle.roll;
        uart_putc_raw(UART_ID, 1);
        uart_putc_raw(UART_ID, y & 0xFF);
        uart_putc_raw(UART_ID, (y >> 8) & 0xFF);
        uart_putc_raw(UART_ID, -1);

        int click = acceleration[1];
        if (click > 10000){
            uart_putc_raw(UART_ID, 2);
            uart_putc_raw(UART_ID, 0);
            uart_putc_raw(UART_ID, ((click >> 8) & 0xFF));
            uart_putc_raw(UART_ID, -1);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void main() {
    stdio_init_all();

    xQueueMouse = xQueueCreate(10, sizeof(data_mouse));

    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}