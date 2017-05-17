
#!/bin/sh

# GPIO numbers should be from this list
# 5, 6

# Set up GPIO 5  and set to input
echo "5" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio5/direction

# Set up GPIO 6 input
echo "17" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio17/direction

echo "18" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio18/direction
