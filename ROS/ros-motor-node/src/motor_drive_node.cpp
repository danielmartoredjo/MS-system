#include <vector>
#include <std_msgs/Float32.h>
#include <stdio.h>
//#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <motor_drive/drive_sensor.h> //change to hunt msg
#include <wiringPi.h>

using namespace std;

float pwA;
float pwM;
FILE *pwmp;

namespace motor_drive_node {
//V0.2

 class Drive {
  public:
   Drive(int s, int a) : speed_(s), axis_(a) {
     pinMode(speed_, OUTPUT);
     pinMode(axis_, OUTPUT);
     pwmp = fopen("/dev/pwm_drv","w");
     fprintf(pwmp,"S");
     fclose(pwmp);
     }

     unsigned int pw_iA;
     char pwmaxis[6];
     float pwA;
     float motor_axis(void){
      if (pwA > 900 && pwA <2200){
       pw_iA = (unsigned int) pwA;
       pwmp = fopen("/dev/pwm_drv","w");
       pwmaxis[0] = 'A';
       pwmaxis[1] = ((pw_iA /1000) % 10) + '0';
       pwmaxis[2] = ((pw_iA /100) % 10) + '0';
       pwmaxis[3] = ((pw_iA /10) % 10) + '0';
       pwmaxis[4] = ((pw_iA /1) % 10) + '0';
       fprintf(pwmp,"%s",pwmaxis);
       fclose(pwmp);

       }
      else{ 
      }
      return pwA; 
    }
     unsigned int pw_iM;
     char pwmspeed[6];
     float pwM;
     float motor_speed(void){
      if (pwM > 900 && pwM <2200)
      {
       pw_iM = (unsigned int) pwM;
       pwmp = fopen("/dev/pwm_drv","w");
       pwmspeed[0] = 'M';
       pwmspeed[1] = ((pw_iM /1000) % 10) + '0';
       pwmspeed[2] = ((pw_iM /100) % 10) + '0';
       pwmspeed[3] = ((pw_iM /10) % 10) + '0';
       pwmspeed[4] = ((pw_iM /1) % 10) + '0';
       fprintf(pwmp,"%s",pwmspeed);
       fclose(pwmp);
       }
      else
      { 
      }
      return pwM;
    }


 private:
  int speed_;
  int axis_;
 }; 
}

void motor_speedCb(const std_msgs::Float32::ConstPtr& msg){
pwM = msg->data;

}

void motor_axisCb(const std_msgs::Float32::ConstPtr& msg){
pwA = msg->data;

}

int main(int argc, char **argv) {
  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "motor_drives");
  ros::NodeHandle node;
  ros::Rate rate(60);  // 10 hz
  

  // Build N motor_drive.
  wiringPiSetupSys();  // uses gpio pin numbering
  // TODO: config these
  vector<motor_drive_node::Drive> motor_drives;
  motor_drives.push_back(motor_drive_node::Drive(18 , 17));

  // Build a publisher for each Hunt sensor.
  vector<ros::Publisher> drive_pubs;
  for (int i = 0; i < motor_drives.size(); ++i) {
    stringstream ss;
    ss << "drive_" << i;
    drive_pubs.push_back(node.advertise<motor_drive::drive_sensor>(ss.str(), 10)); //replace sensor_msgs::Range for costum msg
  }

  ros::Subscriber speed_sub = node.subscribe("motor_speed", 1000, motor_speedCb);
  ros::Subscriber axis_sub = node.subscribe("motor_axis", 1000, motor_axisCb);  
  
  // Build base range message that will be used for
  // each time a msg is published.
  motor_drive::drive_sensor drive; //replace for Hunt
  drive.sensor_id = "motor_drive";

  while(ros::ok()) {
    for (int i = 0; i < motor_drives.size(); ++i) {
      drive.header.stamp = ros::Time::now();
      motor_drives[i].pwA = pwA;
      motor_drives[i].pwM = pwM;
      drive.pwA = motor_drives[i].motor_axis();
      drive.pwM = motor_drives[i].motor_speed();
      
      drive_pubs[i].publish(drive);
    }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

