
#!/bin/sh

# GPIO numbers should be from this list
# 5, 6

# Set up GPIO 5  and set to input
echo "5" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio5/direction

# Set up GPIO 6 input
echo "6" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio6/direction

