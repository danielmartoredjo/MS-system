#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ky_032/dist_sensor.h> //change to hunt msg
#include <wiringPi.h>

#define GPIO_IR_0 21

using namespace std;

namespace ky_032_node {
//V0.1

 class Dist {
  public:
   Dist(int u) : signal_(u){
     pinMode(signal_, INPUT);
     delay(1000);
     }
     float value() {
      float SensorDist;
      if(digitalRead(signal_) == HIGH) {
        SensorDist = 0;
        }
      if(digitalRead(signal_) == LOW) {
        SensorDist = 1;
        }
    return SensorDist;
    }


 private:
  int signal_;
 }; 
}

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "ky_032s");
  ros::NodeHandle node;
  ros::Rate rate(10);  // 10 hz

  // Build N ky_033.
  wiringPiSetupSys();  // uses gpio pin numbering
  // TODO: config these
  vector<ky_032_node::Dist> dists;
  dists.push_back(ky_032_node::Dist(GPIO_IR_0));

  // Build a publisher for each Hunt sensor.
  vector<ros::Publisher>dist_pubs;
  for (int i = 0; i < dists.size(); ++i) {
    stringstream ss;
    ss << "IR_" << i;
    dist_pubs.push_back(node.advertise<ky_032::dist_sensor>(ss.str(), 10)); //replace sensor_msgs::Range for costum msg
  }
  
  // Build base range message that will be used for
  // each time a msg is published.
  ky_032::dist_sensor dist; //replace for Hunt
  dist.sensor_id = "ky_032";

float value;
  while(ros::ok()) {
    for (int i = 0; i < dists.size(); ++i) {
      dist.header.stamp = ros::Time::now();
      dist.SensorDist = dists[i].value();
      dist_pubs[i].publish(dist);
    }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

