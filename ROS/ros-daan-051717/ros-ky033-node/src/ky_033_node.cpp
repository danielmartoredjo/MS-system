#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ky_033/hunt_sensor.h> //change to hunt msg
#include <wiringPi.h>

using namespace std;

namespace ky_033_node {
//V0.1

 class Hunt {
  public:
   Hunt(int s) : signal_(s) {
     pinMode(signal_, INPUT);
     delay(1000);
     }
     float value() {
      float sensorValue;
      if(digitalRead(signal_) == HIGH) {
        sensorValue = 0;
        }
      if(digitalRead(signal_) == LOW) {
        sensorValue = 1;
        }
    return sensorValue;
    }


 private:
  int signal_;
 }; 
}

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "ky_033s");
  ros::NodeHandle node;
  ros::Rate rate(10);  // 10 hz

  // Build N ky_033.
  wiringPiSetupSys();  // uses gpio pin numbering
  // TODO: config these
  vector<ky_033_node::Hunt> hunts;
  hunts.push_back(ky_033_node::Hunt(5));
  hunts.push_back(ky_033_node::Hunt(6));

  // Build a publisher for each Hunt sensor.
  vector<ros::Publisher> hunt_pubs;
  for (int i = 0; i < hunts.size(); ++i) {
    stringstream ss;
    ss << "hunt_" << i;
    hunt_pubs.push_back(node.advertise<ky_033::hunt_sensor>(ss.str(), 10)); //replace sensor_msgs::Range for costum msg
  }
  
  // Build base range message that will be used for
  // each time a msg is published.
  ky_033::hunt_sensor hunt; //replace for Hunt
  hunt.sensor_id = "ky_033";

float value;
  while(ros::ok()) {
    for (int i = 0; i < hunts.size(); ++i) {
      hunt.header.stamp = ros::Time::now();
      hunt.sensorValue = hunts[i].value();
      hunt_pubs[i].publish(hunt);
    }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

