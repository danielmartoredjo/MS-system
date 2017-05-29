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
static int brake = 0;
static int drive = 0;
float pwm(void)
{
if (hc_range_1 > 0 && hc_range_1 <= 20 && state == 0)
{
  npwM = 1520;
}
/*else if ( hc_range_1 > 0 && hc_range_1 <= 10.0 && state == 1)
{
  drive = 0;
  brake = brake + 1;
  if (brake < 60 ){
    npwM = 1000;
  }
}*/
else if(hc_range_1 > 22.0 && hc_range_1 <= 28.0)
{
  npwM = 1600;
//  drive = drive + 1;
//  brake = 0;
//  state = 0;
 // if (drive >= 20){
//    state = 1;
}
  
}
else if(hc_range_1 > 28 && hc_range_1 < 45.0 )
{
  npwM = 1610;
//  drive = drive + 1;
//  brake = 0;
//  state = 0;
//  if (drive >=20){
//    state = 1;
  }
} 
else if(hc_range_1 >= 52.0)
{
    npwM = 1520;
//    state = 0;
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
enum states { rechts, midden, links } current_state;
static int step = 40;
static int M = 50;
float axis(void)
{
  current_state = midden;
  if(hc_range_2 < (hc_range_0 - 15) && hc_range_1 >= 15 && current_state == midden)
  {
   current_state = rechts;
   npwA = npwA + step;
   if (npwA >= 1660){
    npwA = 1660;

   }
  }
  else if(hc_range_0 < (hc_range_2 - 15) && hc_range_1 >= 15 && current_state == midden)
  {
   current_state = links;
   npwA = npwA - step;
   if (npwA <= 1100){
    npwA = 1100;

   }
  }
  else if(hc_range_1 <= M && hc_range_0 >= (hc_range_2 - 10) && hc_range_0 <= (hc_range_2 + 10) && hc_range_2 >= (hc_range_0 - 10) && hc_range_2 <= (hc_range_0 + 10)){
    npwA = 1380;
    current_state = midden;
  }
  else if (hc_range_1 >= M && hc_range_0 >= M && hc_range_0 >= M ) {
    npwA = 1380;
  }
  else {
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
  ros::Rate rate(200);  // 10 hz
  

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

