
#!/bin/sh

# used GPIOs
# 5, 6, 18, 19, 22, 23, 24, 25, 26, 27



#GPIO for motor_drive

#PWM_motor
# Set up GPIO 19 and set to output
echo "19" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio19/direction

#PWM_axis
# Set up GPIO 26 and set to output
echo "26" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio26/direction



#GPIO for ky-033

#IR_0
# Set up GPIO 21  and set to input
echo "21" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio21/direction


#GPIO for ky-033

#Hunt_0
# Set up GPIO 5  and set to input
echo "5" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio5/direction

#Hunt_1
# Set up GPIO 6 input
echo "6" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio6/direction



#GPIO for hc-sr04

#Sonar_0
# Set up GPIO 24 and set to output
echo "24" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio24/direction

# Set up GPIO 25  and set to input
echo "25" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio25/direction

#Sonar_1
# Set up GPIO 22 output
echo "22" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio22/direction

# Set up GPIO 23 input
echo "23" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio23/direction

#Sonar_2
# Set up GPIO 18 output
echo "18" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio18/direction

# Set up GPIO 27 input
echo "27" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio27/direction
