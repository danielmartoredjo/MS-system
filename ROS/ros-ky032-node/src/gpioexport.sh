
#!/bin/sh

# GPIO numbers should be from this list
# 5, 6

# Set up GPIO 20  and set to input
echo "21" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio21/direction

echo "20" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio20/direction






