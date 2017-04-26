
#!/bin/sh

# GPIO numbers should be from this list
# 18, 22, 23, 24, 25, 27

# Note that the GPIO numbers that you program here refer to the pins
# of the BCM2835 and *not* the numbers on the pin header.
# So, if you want to activate GPIO7 on the header you should be
# using GPIO4 in this script. Likewise if you want to activate GPIO0
# on the header you should be using GPIO17 here.

# Set up GPIO 24 and set to output
echo "24" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio24/direction

# Set up GPIO 25  and set to input
echo "25" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio25/direction

# Set up GPIO 22 output
echo "22" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio22/direction

# Set up GPIO 23 input
echo "23" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio23/direction

# Set up GPIO 18 output
echo "18" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio18/direction

# Set up GPIO 27 input
echo "27" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio27/direction
