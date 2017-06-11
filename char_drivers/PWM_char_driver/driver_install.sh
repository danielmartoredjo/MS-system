#!/bin/bash
# Install pwm driver with 0666 priority

insmod PWM_device_driver.ko
chmod 0666 /dev/pwm_drv