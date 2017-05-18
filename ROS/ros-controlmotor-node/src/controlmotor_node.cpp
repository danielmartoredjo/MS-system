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

float hc_range;

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
static int kickstart = 0;
static int een = 0;
static int twee = 0;
static int gem = 0;
float pwm(void)
{
    gem = twee;
    twee = een;
    een = hc_range;
    gem = een + twee + gem;
    gem = gem / 3;

    if(gem <= 7.0)
    {
        npwM = 1520;
        kickstart = 0;
    }
    else if(gem > 13.0 && gem <= 28.0)
      {
        npwM = 1610;
        /*if(kickstart <= 6)
        {
          kickstart++;
          npwM = 1610;
        }
        else if(kickstart > 6)
        {
          npwM = 1590;
        }
        */
  }
  else if(gem > 28 && gem < 45.0 )
  {
      npwM = 1625;
  } 
  else if(gem >= 52.0)
  {
      npwM = 1520;
  }
 return 0;
}


void hc_sr04Cb(const std_msgs::Float32::ConstPtr& msg){
hc_range  = msg->data;
}



int main(int argc, char **argv) {
  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "controlmotor");
  ros::NodeHandle node;
  ros::Rate rate(10);  // 10 hz
  

  // Build N motor_drive.
  wiringPiSetupSys();  // uses gpio pin numbering
  vector<controlmotor_node::Control> controlmotors;
  controlmotors.push_back(controlmotor_node::Control(18));

  // Build a publisher for each Hunt sensor.
  ros::Publisher speed_pubs = node.advertise<std_msgs::Float32>("motor_speed", 100);
  ros::Publisher axis_pubs = node.advertise<std_msgs::Float32>("motor_axis", 100);


  ros::Subscriber sub = node.subscribe("hc_sr04_range", 1000, hc_sr04Cb);


  std_msgs::Float32 npwA_send;
  std_msgs::Float32 npwM_send;
  while(ros::ok()) {
    pwm();
    npwA_send.data = npwA;
    npwM_send.data = npwM;
    axis_pubs.publish(npwA_send);
    speed_pubs.publish(npwM_send);
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}
