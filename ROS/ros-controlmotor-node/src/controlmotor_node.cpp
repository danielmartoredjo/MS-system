/*
 * file: controlmotor_node.cpp
 * version: 1.5
 */
 
#include <vector>
#include <std_msgs/Float32.h>
#include <stdio.h>
//#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <ros/ros.h>
//#include <controlmotor/control_sensor.h> //change to hunt msg
#include <wiringPi.h>
 
using namespace std;
 
float hc_range_0;
float hc_range_1;
float hc_range_2;
float speedMeasure;
float masterSpeed;
int targetSpeed;
 
#define SCHEDULING_RATE 200 // hz
#define STOP_CYCLES SCHEDULING_RATE / 15 // stop cycles for the break states
#define STOP_REV_CYCLES SCHEDULING_RATE / 10
#define GPIO_BRAKE_0 6
#define GPIO_BRAKE_1 21
 
namespace controlmotor_node {
//V0.2
 
 class Control {
    public:
     Control(int d) : signal_(d) {
         pinMode(signal_, INPUT);
         pinMode(GPIO_BRAKE_0, OUTPUT);
         pinMode(GPIO_BRAKE_1, OUTPUT);
         }
 private:
    int signal_;
 }; 
}
 
float npwM = 1520;
float npwA = 1380;
enum motor_states { idle, driveStart, driveMin, driveMid, drivePlus, brake_init, brake_stationary, brake_reverse, brake_idle } ;
static enum motor_states state = idle;
static enum motor_states ostate = brake_stationary;
static int stop = 0;
static float spacing = 40;
static int drive = 0;
float pwm(void)
{
    // determine state
    switch(state)
    {
        case idle:
            digitalWrite(GPIO_BRAKE_0, HIGH);
            digitalWrite(GPIO_BRAKE_1, HIGH);
            if (hc_range_1 <= 20)
            {
                state = idle;
            } 
            else if (hc_range_0 > 25 && hc_range_1 > 25 && hc_range_2 > 25) 
            {
                // state = drivePlus;
                state = driveStart;
            }
            break;
 
        case driveStart:
            if (hc_range_1 <= 15 && speedMeasure > 0)
            {
                state = brake_init;
            }
            else if (hc_range_1 > (spacing - 5))
            {
                state = driveMin;
            }
            else
            {
                state = driveStart;
            }
            break;
 
 
        case driveMin:
            if (hc_range_1 <= 20 && speedMeasure > 0)
            {
                state = brake_init;
            } 
            else if (hc_range_1 > (spacing + 5)) 
            {
                state = drivePlus;
            }
            else if ((hc_range_1 > (spacing - 5)) && (hc_range_1 < (spacing + 5)))
            {
                state = driveMid;
            }
            else
            {
                state = driveMin;
            }
            break;
 
        case driveMid:
            if (hc_range_1 <= 20 && speedMeasure > 0)
            {
                state = brake_init;
            } 
            else if (hc_range_1 > (spacing + 5))
            {
                state = drivePlus;
            }
            else if (hc_range_1 < (spacing - 5))
            {
                state = driveMin;
            }
            else
            {
                state = driveMid;
            }
            break;
 
        case drivePlus:
            if (hc_range_1 <= 20 && speedMeasure > 0)
            {
                state = brake_init;
            } 
            else if (hc_range_1 < (spacing - 5)) 
            {
                state = driveMin;
            }
            else if ((hc_range_1 > (spacing - 5)) && (hc_range_1 < (spacing + 5)))
            {
                state = driveMid;
            }
            else
            {
                state = drivePlus;
            }
            break;
 
        case brake_init:
            digitalWrite(GPIO_BRAKE_0, LOW);
            digitalWrite(GPIO_BRAKE_1, LOW);
            if (stop >= STOP_CYCLES)
            {
                state = brake_stationary;
                stop = 0;
            }
            else
            {
                state = brake_init;
            }
            break;
 
        case brake_stationary:
            if (stop >= STOP_CYCLES)
            {
                state = brake_reverse;
                stop = 0;
            }
            else
            {
                state = brake_stationary;
            }
            break;
 
        case brake_reverse:
            if (stop >= STOP_REV_CYCLES)
            {
 
                state = brake_idle;
                stop = 0;
            }
            else
            {
                state = brake_reverse;
            }
            break;
 
        case brake_idle:
            if (speedMeasure <= 0)
            { 
                state = idle;
            }
            else
            {
                state = brake_idle;
            }
            break;
 
        default:
            state = idle;
    }
 
    // out of range emergency state
    //if (hc_range_1 > 58.0)
    //{
    //    state = drive1;
    //}
 
    // act on state
    switch (state)
    {
        case idle:
            npwM = 1520;
            //targetSpeed = 0;
            stop = 0;    
            break;
 
        case driveStart:
            npwM = 1600;
            stop = 0;
            break;
 
        case driveMin:
            //npwM--;
            //if (npwM <= 1595)
            //{
                npwM = 1593;
            //}
            //targetSpeed = 8;
            stop = 0;
            break;
 
        case driveMid:
            npwM = 1603;
            //targetSpeed = 10; 
            stop = 0;
            break;
 
        case drivePlus:
            //npwM++;
            //if (npwM >= 1605)
            //{
                npwM = 1605;
            //}
            //targetSpeed = 8;
            stop = 0;
            break;
 
 
        case brake_init:
            npwM = 1000;
            //targetSpeed = -10;
            stop++;
            break;
 
        case brake_stationary:
            npwM = 1520;
            //targetSpeed = 0;
            stop++;
            break;
 
        case brake_reverse:
            npwM = 1320;
            //targetSpeed = -5;
            stop++;
            break;
 
        case brake_idle:
            npwM = 1520;
            break;
 
        default:
            npwM = 1520;
            //targetSpeed = 0;
            stop = 0;
    }
 
    if (ostate != state)
    {
        ostate = state;
        ROS_INFO("Motor state: %d", state);
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
static int step = 2;
static int M = 50;
float axis(void)
{
    if (hc_range_0 > 3 && hc_range_1 > 3 && hc_range_2 > 3 ){
        current_state = midden;
        if(hc_range_2 < (hc_range_0 - 10) && hc_range_1 >= 10 && current_state == midden)
        {
         current_state = rechts;
         npwA = npwA + step;
         if (npwA >= 1660){
            npwA = 1660;
 
         }
        }
        else if(hc_range_0 < (hc_range_2 - 10) && hc_range_1 >= 10 && current_state == midden)
        {
         current_state = links;
         npwA = npwA - step;
         if (npwA <= 1100){
            npwA = 1100;
 
         }
        }
        else if(hc_range_1 <= M && hc_range_0 >= (hc_range_2 - 8) && hc_range_0 <= (hc_range_2 + 8) && hc_range_2 >= (hc_range_0 - 8) && hc_range_2 <= (hc_range_0 + 8)){
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
 
void spdMsr_0Cb(const std_msgs::Float32::ConstPtr& msg){
speedMeasure = msg->data;
}
 
void master_speedCb(const std_msgs::Float32::ConstPtr& msg){
masterSpeed = msg->data;
}
 
 
 
 
int main(int argc, char **argv) {
    // Start ROS node.
    ROS_INFO("Starting node");
    ros::init(argc, argv, "controlmotor");
    ros::NodeHandle node;
    ros::Rate rate(SCHEDULING_RATE);  
     
 
    // Build N motor_drive.
    wiringPiSetupSys();  // uses gpio pin numbering
    vector<controlmotor_node::Control> controlmotors;
    //controlmotors.push_back(controlmotor_node::Control(18));
 
    // Build a publisher for each Hunt sensor.
    ros::Publisher speed_pubs = node.advertise<std_msgs::Float32>("motor_speed", 100);
    ros::Publisher axis_pubs = node.advertise<std_msgs::Float32>("motor_axis", 100);
 
 
    ros::Subscriber sub_0 = node.subscribe("hc_sr04_range_0", 10, hc_sr04_0Cb);
    ros::Subscriber sub_1 = node.subscribe("hc_sr04_range_1", 10, hc_sr04_1Cb);
    ros::Subscriber sub_2 = node.subscribe("hc_sr04_range_2", 10, hc_sr04_2Cb);
 
     ros::Subscriber spdMsr_sub_0 = node.subscribe("spdMsr", 10, spdMsr_0Cb);
     ros::Subscriber master_speed_sub = node.subscribe("master_speed", 10, master_speedCb);
 
 
    std_msgs::Float32 npwA_send;
    std_msgs::Float32 npwM_send;
    while(ros::ok()) {
        pwm();
        axis();
        //speedctrl();
        npwA_send.data = npwA;
        npwM_send.data = npwM;
        axis_pubs.publish(npwA_send);
        speed_pubs.publish(npwM_send);
 
        ros::spinOnce();
        rate.sleep();    
    }
    return 0;
}