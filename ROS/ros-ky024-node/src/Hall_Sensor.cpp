#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ky_024/hall_sensor.h> //change to hunt msg
#include <wiringPi.h>

#define GPIO_HS_0 17
#define GPIO_HS_1 4

using namespace std;

namespace ky_024_node {
//V0.1

 class Spd {
  public:
   Spd(int u) : signal_(u){
     pinMode(signal_, INPUT);
     delay(1000);
     }

static long starttime = 0;
static long endtime = 0;
long time = 0;
int lock = 1;

     float value() {
      float SensorSpd;
	while(digitalRead(signal_) && lock){
		starttime = micros();
		while(!digitalRead(signal_) && lock){
			while(digitalRead(signal_) && lock){			
			endtime = micros();
			time = starttime - endtime;
			SensorSpd = 60000 / time; //( in RPM)
			lock = 0;
			}
		}
	}
      if(digitalRead(signal_) == LOW) {
	SensorSpd = 0;
        }
    return SensorSpd;
    }


 private:
  int signal_;
 }; 
}

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "ky_024s");
  ros::NodeHandle node;
  ros::Rate rate(10);  // 10 hz

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
    ss << "IR_" << i;
    Spd_pubs.push_back(node.advertise<ky_024::hall_sensor>(ss.str(), 10)); //replace sensor_msgs::Range for costum msg
  }
  
  // Build base range message that will be used for
  // each time a msg is published.
  ky_024::hall_sensor hall; //replace for Hunt
  hall.sensor_id = "ky_024";

float value;
  while(ros::ok()) {
    for (int i = 0; i < Spds.size(); ++i) {
      hall.header.stamp = ros::Time::now();
      hall.Speed = Spds[i].value();
      hall_pubs[i].publish(hall);
    }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}