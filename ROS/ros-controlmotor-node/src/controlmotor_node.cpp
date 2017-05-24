#include <vector>
#include <std_msgs/Float32.h>
#include <stdio.h>
//#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <controlmotor/control_sensor.h> //change to hunt msg
#include <wiringPi.h>

using namespace std;

float hc_range_0;
float hc_range_1;
float hc_range_2;

namespace controlmotor_node {
//V0.2

 class Control {
  public:
   Control(int d) : signal_(d) {
     pinMode(signal_, INPUT);
     }
 private:
  int signal_;
 }; 
}

float npwM = 1520;
float npwA = 1380;
static int state = 0;
static int kickstart = 0;
static int een = 0;
static int twee = 0;
static int gem = 0;
float pwm(void)
{
    gem = twee;
    twee = een;
    een = hc_range_1;
    gem = een + twee + gem;
    gem = gem / 3;
if (gem <= 10 && state == 0)
{
  npwM = 1520;
}
else if(gem <= 10.0 && state == 1)
{
    npwM = 1520;
}
else if(gem > 13.0 && gem <= 28.0)
{
  npwM = 1600;
  state = 1;
}
else if(gem > 28 && gem < 45.0 )
{
    npwM = 1610;
    state = 1;
} 
else if(gem >= 52.0)
{
    npwM = 1520;
}
return 0;
}


/*
us   | angle
-----------
1000 |  -20
1100 |  -15
1180 |  -10
1380 |  0
1560 |  10
1660 |  15

*/
static int step = 40;
float axis(void)
{
  if(hc_range_0 >= 50.0 && hc_range_1 <= 50.0 && hc_range_2 < hc_range_0)
  {
   npwA = npwA + step;
   if (npwA >= 1660){
    npwA = 1660;
   }
  }
  else if(hc_range_0 < hc_range_2 && hc_range_1 <= 50.0 && hc_range_2 >= 50.0)
  {
   npwA = npwA - step;
   if (npwA <= 1100){
    npwA = 1100;
   }
  }
  else{
    npwA = 1380;
  }
}

void hc_sr04_0Cb(const std_msgs::Float32::ConstPtr& msg){
hc_range_0  = msg->data;
}

void hc_sr04_1Cb(const std_msgs::Float32::ConstPtr& msg){
hc_range_1  = msg->data;
}

void hc_sr04_2Cb(const std_msgs::Float32::ConstPtr& msg){
hc_range_2  = msg->data;
}



int main(int argc, char **argv) {
  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "controlmotor");
  ros::NodeHandle node;
  ros::Rate rate(60);  // 10 hz
  

  // Build N motor_drive.
  wiringPiSetupSys();  // uses gpio pin numbering
  vector<controlmotor_node::Control> controlmotors;
  controlmotors.push_back(controlmotor_node::Control(18));

  // Build a publisher for each Hunt sensor.
  ros::Publisher speed_pubs = node.advertise<std_msgs::Float32>("motor_speed", 100);
  ros::Publisher axis_pubs = node.advertise<std_msgs::Float32>("motor_axis", 100);


  ros::Subscriber sub_0 = node.subscribe("hc_sr04_range_0", 10, hc_sr04_0Cb);
  ros::Subscriber sub_1 = node.subscribe("hc_sr04_range_1", 10, hc_sr04_1Cb);
  ros::Subscriber sub_2 = node.subscribe("hc_sr04_range_2", 10, hc_sr04_2Cb);


  std_msgs::Float32 npwA_send;
  std_msgs::Float32 npwM_send;
  while(ros::ok()) {
    pwm();
    axis();
    npwA_send.data = npwA;
    npwM_send.data = npwM;
    axis_pubs.publish(npwA_send);
    speed_pubs.publish(npwM_send);
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

