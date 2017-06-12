#!/bin/sh

# GPIO numbers should be from this list
# pin 19 GPIO10 SPI_MOSI
# pin 21 GPIO09 SPI_MISO
# pin 23 GPIO11 SPI_CLK
# pin 33 GPIO13 SPI_CS

# Set up GPIO 10 and set to output
echo "10" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio10/direction

# Set up GPIO 09 and set to input
echo "09" > /sys/class/gpio/export
echo "in" > /sys/class/gpio/gpio09/direction

# Set up GPIO 11 and set to output
echo "11" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio11/direction

# Set up GPIO 13 and set to output
echo "13" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio13/direction

