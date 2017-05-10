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

namespace motor_drive_node {
//V0.1

 class Drive {
  public:
   Drive(int d) : signal_(d) {
//     wiringPiSetupGpio ();
     pinMode(signal_, INPUT);
     float pwinit = 1520; //calibration  width in microseconds
     for (x= 0; x<100; x++)
      {
      digitalWrite(signal_, HIGH);
      delayMicroseconds (pwinit);
      digitalWrite(signal_, LOW);
      delayMicroseconds (16500-pwinit);
      }
     }
     float pw;
     float value(){
      if (pw > 0 && pw <16500){
       digitalWrite(signal_, HIGH) ;
//       for (int j = 0; j < pw; j++)
//       {
//         usleep(1);
//       }
       delayMicroseconds (pw) ;
       digitalWrite(signal_, LOW) ;
//       for (int j = 0; j < (16500-pw); j++)
//       {
//         usleep(1);
//       }
       delayMicroseconds (16500-pw) ;

      }
      else{ 
       //pw = 1520;
      }
      return pw;

     }


 private:
  int signal_;
  int x;
 }; 
}

float pw;
void motor_speedCb(const std_msgs::Float32::ConstPtr& msg){
pw = msg->data;

}

int main(int argc, char **argv) {
  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "motor_drives");
  ros::NodeHandle node;
  ros::Rate rate(60);  // 10 hz
  

  // Build N motor_drive.
  //  wiringPiSetupGpio ();
  wiringPiSetupSys();  // uses gpio pin numbering
  // TODO: config these
  vector<motor_drive_node::Drive> motor_drives;
  motor_drives.push_back(motor_drive_node::Drive(18));

  // Build a publisher for each Hunt sensor.
  vector<ros::Publisher> drive_pubs;
  for (int i = 0; i < motor_drives.size(); ++i) {
    stringstream ss;
    ss << "drive_" << i;
    drive_pubs.push_back(node.advertise<motor_drive::drive_sensor>(ss.str(), 10)); //replace sensor_msgs::Range for costum msg
  }

  ros::Subscriber sub = node.subscribe("motor_speed", 1000, motor_speedCb);  
  
  // Build base range message that will be used for
  // each time a msg is published.
  motor_drive::drive_sensor drive; //replace for Hunt
  drive.sensor_id = "motor_drive";

float value;
  while(ros::ok()) {
    for (int i = 0; i < motor_drives.size(); ++i) {
      drive.header.stamp = ros::Time::now();
      //temp_pw = motor_drives[i].value();
      motor_drives[i].pw = pw;
      drive.pw = motor_drives[i].value();
      //motor_drives[i].pw = temp_pw;
      //drive.pw = temp_pw;
      drive_pubs[i].publish(drive);
    }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

