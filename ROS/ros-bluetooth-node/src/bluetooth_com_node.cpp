/*
 *  File: bluetooth_com.cpp
 *  version: 1.2
 */

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <bluetooth_com/bluetooth_com.h>
#include <string.h>

// libraries for BlueZ
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

// #define BDADDR_MSPI_1   (&(bdaddr_t) {{ 184, 39, 235, 93, 123, 201}}) // B8:27:EB:5D:7B:C9

using namespace std;

namespace bluetooth_com_node 
{
};

//static bdaddr_t bd_adrr_mspi_1;

int main(int argc, char **argv) 
{
    /*
    bd_adrr_mspi_1.b[0] = 184;
    bd_adrr_mspi_1.b[1] = 39;
    bd_adrr_mspi_1.b[2] = 235;
    bd_adrr_mspi_1.b[3] = 93;
    bd_adrr_mspi_1.b[4] = 123;
    bd_adrr_mspi_1.b[5] = 201;
    // bdaddr_t *MSPI = (&(bdaddr_t) {{ 184, 39, 235, 93, 123, 201}}); // B8:27:EB:5D:7B:C9
    */
    // Start ROS node.
    ROS_INFO("Starting node");
    ros::init(argc, argv, "bluetooth_com");
    ros::NodeHandle node;
    ros::Rate rate(60);  // 60 hz

    // Build a publisher for the bluetooth messages

    ros::Publisher master_speed_pub     = node.advertise<std_msgs::Float32>("master_speed", 10);
    ros::Publisher master_acce_x_pub    = node.advertise<std_msgs::Float32>("master_acce_x", 10);
    ros::Publisher master_acce_y_pub    = node.advertise<std_msgs::Float32>("master_acce_y", 10);
    ros::Publisher master_acce_z_pub    = node.advertise<std_msgs::Float32>("master_acce_z", 10);

    std_msgs::Float32 master_speed_send;
    std_msgs::Float32 master_acce_x_send;
    std_msgs::Float32 master_acce_y_send;
    std_msgs::Float32 master_acce_z_send;

    // Bluetooth stuff
    struct sockaddr_rc loc_addr = { 0 }, rem_addr = { 0 };
    char buf[1024] = { 0 };
    int s, client, bytes_read;
    socklen_t opt = sizeof(rem_addr);

    // allocate socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // bind socket to port 1 of the first available 
    // local bluetooth adapter
    loc_addr.rc_family = AF_BLUETOOTH;
    // loc_addr.rc_bdaddr = *BDADDR_ANY;
    // loc_addr.rc_bdaddr = *BDADDR_MSPI_1;
    bdaddr_t tmp = { };
    loc_addr.rc_bdaddr = tmp;
    loc_addr.rc_channel = (uint8_t) 1;
    bind(s, (struct sockaddr *)&loc_addr, sizeof(loc_addr));

    // put socket into listening mode
    ROS_INFO("Socket opened. Looking \n");
    listen(s, 1);

    // accept one connection
    client = accept(s, (struct sockaddr *)&rem_addr, &opt);

    ba2str( &rem_addr.rc_bdaddr, buf );
    ROS_INFO("Accepted connection from %s\n", buf);
    memset(buf, 0, sizeof(buf));

    while(ros::ok()) 
    {    
        bytes_read = read(client, buf, sizeof(buf));
        if( bytes_read > 0 ) 
        {
            // read data
            //ROS_INFO("Received [%s]\n", buf);
            // do stuff to receive the data

            char Sspeed[30];
            float Fspeed;
            char Sxacc[30];
            float Fxacc;
            char Syacc[30];
            float Fyacc;
            char Szacc[30];
            float Fzacc;

            strncpy(Sspeed, buf+1, 4);
            Sspeed[4]= '\0';
            Fspeed = strtof(Sspeed, NULL);

            strncpy(Sxacc, buf+8, 4);
            Sxacc[4]= '\0';
            Fxacc = strtof(Sxacc, NULL);

            strncpy(Syacc, buf+15, 4);
            Syacc[4]= '\0';
            Fyacc = strtof(Syacc, NULL);

            strncpy(Szacc, buf+22, 4);
            Szacc[4]= '\0';
            Fzacc = strtof(Szacc, NULL);


            // do stuff with the data
            master_speed_send.data = Fspeed;
            master_acce_x_send.data = Fxacc;
            master_acce_y_send.data = Fyacc;
            master_acce_z_send.data = Fzacc;

            // Publish the values
            master_speed_pub.publish(master_speed_send);
            master_acce_x_pub.publish(master_acce_x_send);
            master_acce_y_pub.publish(master_acce_y_send);
            master_acce_z_pub.publish(master_acce_z_send);

        } else {
            ROS_WARN("Connection lost. Retry listening\n");
            // put socket into listening mode
            listen(s, 1);

            // accept one connection
            client = accept(s, (struct sockaddr *)&rem_addr, &opt);

            ba2str( &rem_addr.rc_bdaddr, buf );
            ROS_INFO("Accepted connection from %s\n", buf);
            memset(buf, 0, sizeof(buf));
        }
        ros::spinOnce();
        rate.sleep();    
    }
    // close connection
    close(client);
    close(s);
    
    return 0;
}