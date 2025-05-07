#include "main.h"
#include "zephyr/drivers/i2c.h"

atomic_t sampled_angle_degree;

K_THREAD_STACK_DEFINE(sensor_stack_area, TASK_STACK_SIZE);
struct k_thread sensor_thread_data;

static const struct device *i2c_bus_dev = DEVICE_DT_GET(DT_ALIAS(as5600_bus)); // NOLINT(*-interfaces-global-init)
#define AS5600_I2C_ADDRESS (0x36)

#if !DT_NODE_HAS_STATUS(DT_ALIAS(as5600_bus), okay)
#error "as5600_sensor device alias not found or not enabled"
#endif


#define AS5600_STATUS_REG (0x0B)
#define AS5600_STATUS_MAGNET_DETECTED_FLAG (0x20)
#define AS5600_ANGLE_REGISTER_HIGH (0x0E)

static _Noreturn void sensor_task_entry(__unused void* p1,__unused void* p2,__unused void* p3)
{
    //todo low power
    while (true) {
        uint8_t status;
        int ret = i2c_reg_read_byte(i2c_bus_dev, AS5600_I2C_ADDRESS,AS5600_STATUS_REG, &status);
        if (ret != 0) {
            alarm("Failed to fetch status from AS5600: %d", ret);
        }
        if (!(status & AS5600_STATUS_MAGNET_DETECTED_FLAG)) {
            atomic_set(&sampled_angle_degree, -1);
            atomic_set(&led_status, ALARM);
            printk("Magnet not detected, skipping sample\n");
            k_sleep(K_MSEC(500));
        } else {
            uint8_t read_data[2] = {0, 0};
            static const uint8_t angle_reg = AS5600_ANGLE_REGISTER_HIGH;

            ret = i2c_write_read(i2c_bus_dev, AS5600_I2C_ADDRESS, &angle_reg, 1, &read_data, sizeof(read_data));
            if (ret != 0) {
                printk("Failed to fetch sample from AS5600: %d\n", ret);
                atomic_set(&sampled_angle_degree, -1);
            } else {
                uint16_t angle_val = (read_data[0] << 8 | read_data[1]) * 360 /4096;
                atomic_set(&sampled_angle_degree, angle_val);
            }
        }
        k_sleep(K_MSEC(50));
    }
}

void startSensorThread()
{
    startup_device(i2c_bus_dev);

    k_tid_t sensor_tid = k_thread_create(&sensor_thread_data, sensor_stack_area,
                                         K_THREAD_STACK_SIZEOF(sensor_stack_area),
                                         sensor_task_entry, NULL, NULL, NULL,
                                         TASK_PRIORITY, 0, K_NO_WAIT);
    if (!sensor_tid) {
        alarm("Failed to create sensor thread!\n");
    } else {
        k_thread_name_set(sensor_tid, "sensor_task");
    }
}
