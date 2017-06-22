#!/bin/sh

cd /home/pi/catkin_ws
screen -dm -S roscore
screen -S roscore -X stuff 'roscore\n'
screen -dm -S hc_sr04
screen -S hc_sr04 -X stuff 'rosrun hc_sr04 hc_sr04_node\n'
screen -dm -S ky_024
screen -S ky_024 -X stuff 'rosrun ky_024 ky_024_node\n'
screen -dm -S motor_drive
screen -S motor_drive -X stuff 'rosrun motor_drive motor_drive_node\n'
screen -dm -S controlmotor
screen -S controlmotor -X stuff 'rosrun controlmotor controlmotor_node\n'
