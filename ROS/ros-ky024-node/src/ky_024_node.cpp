/*
 *  File:     ky_024_node.cpp
 *  Version:  1.2
 *  Date:     12-06-2017
 */

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ky_024/hall_sensor.h> //change to hunt msg
#include <wiringPi.h>
#include <std_msgs/Float32.h> 

#define GPIO_HS_0 5
#define GPIO_HS_1 6

#define TRAVEL_TIME_MAX_US 1*1000000
#define POLES 5
#define TRAVEL_TIME_MIN_US 0.023748*1000000

// max speed = 30 km/h
// max speed = 8.33 m/s
// max speed = 833 cm/s
// max speed = 0.023748 s

// min speed = 1 s
// min speed = 19.79 cm/s

using namespace std;

namespace ky_024_node {
//V0.1

 class Spd {
  public:
   Spd(int u) : signal_(u){
     pinMode(signal_, INPUT);
     startTime = 0;
     travelTime = 0;
     SensorSpd = 0;
     speed = 0;
     delay(1000);
     }



    float value(bool* error) {
    int bail = 1000;
    do {
      ros::Duration(0.0001).sleep();
      if (--bail == 0) 
      {
        *error = true;
        return 0;
      }
    } while(digitalRead(signal_) == LOW);

    bail = 1000;
    startTime = micros();

    do {
      ros::Duration(0.0001).sleep();
      if (--bail == 0) 
      {
        *error = true;
        return 0;
      }
    } while(digitalRead(signal_) == HIGH);

    
    do {
      ros::Duration(0.001).sleep();
      if (--bail == 0) 
      {
        *error = true;
        return 0;
      }
    } while(digitalRead(signal_) == LOW);
    

    travelTime = micros() - startTime;
    travelTime = travelTime * POLES;
    *error = false; 
    if (travelTime >  TRAVEL_TIME_MAX_US){
      travelTime = TRAVEL_TIME_MAX_US;
    }
    if (travelTime < TRAVEL_TIME_MIN_US){
      travelTime = TRAVEL_TIME_MIN_US;
    }
    speed = (19.79 / travelTime)*10000; //snelheid in m/s   
    return speed*3.6;   //snelheid in km/h

    //speed = 28.03 - (travelTime * 2.2595 / 1000); //snelheid in km/h
    // speed = 28.03 - (travelTime * 6.676 / 1000); //snelheid in km/h
    // if (speed < 0 || speed > 30) 
    // {
    //   speed = 0;
    // }
    //return speed;
  }

 private:
  int signal_;
  float speed;
  long startTime;
  long travelTime;
  float SensorSpd;
 }; 
}

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "ky_024s");
  ros::NodeHandle node;
  ros::Rate rate(60);  // 10 hz

  // Build N ky_024.
  wiringPiSetupSys();  // uses gpio pin numbering
  // TODO: config these
  vector<ky_024_node::Spd> Spds;
  Spds.push_back(ky_024_node::Spd(GPIO_HS_0));
  Spds.push_back(ky_024_node::Spd(GPIO_HS_1));

  // Build a publisher for each Hunt sensor.
  vector<ros::Publisher>Spd_pubs;
  for (int i = 0; i < Spds.size(); ++i) {
    stringstream ss;
    ss << "Hall_" << i;
    Spd_pubs.push_back(node.advertise<ky_024::hall_sensor>(ss.str(), 10)); //replace sensor_msgs::Range for costum msg
  }
  
  ros::Publisher spdMsr_pubs = node.advertise<std_msgs::Float32>("spdMsr", 100);
  // Build base range message that will be used for
  // each time a msg is published.
  ky_024::hall_sensor hall; //replace for Hunt
  hall.sensor_id = "ky_024";

float value;
bool error;
std_msgs::Float32 spdMsr_send;
  while(ros::ok()) {
    // for (int i = 0; i < Spds.size(); ++i) {
    for (int i = 0; i < 1; ++i) {
      hall.header.stamp = ros::Time::now();
      value = Spds[i].value(&error);
      hall.Speed = value;
      if (i == 0){
      spdMsr_send.data = value;
      }
      if (error){
        ROS_INFO("Not move hall_sensor %d", i);
        Spd_pubs[i].publish(hall);
        spdMsr_pubs.publish(spdMsr_send);
      }
      else{
      Spd_pubs[i].publish(hall);
      spdMsr_pubs.publish(spdMsr_send);
      }
    }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}