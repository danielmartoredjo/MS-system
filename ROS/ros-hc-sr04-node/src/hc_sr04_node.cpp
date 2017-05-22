#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <hc_sr04/obj_sensor.h>
#include <wiringPi.h>

#define GPIO_SONAR_0_TRIG    24
#define GPIO_SONAR_0_ECHO    25
#define GPIO_SONAR_1_TRIG    22
#define GPIO_SONAR_1_ECHO    23
#define GPIO_SONAR_2_TRIG    18
#define GPIO_SONAR_2_ECHO    27

#define CSV_TEST
#define CSV_FILE_NAME "Ultrasonic_measure.csv"

#define FILTER_ARRAY_SIZE 20
#define SAMPLE_STANDARD_DEVIATION 0.2

float filterSample(float array[], int size);
float filterSpikeAverage(float array[], int size);

using namespace std;

namespace hc_sr04_node {

// Maximum distance reported. Values over this distance
// report MAX_DISTANCE. TODO make this a property.
const static float MAX_DISTANCE = 60;
const static float DIST_SCALE = 58.0;
const static float TRAVEL_TIME_MAX = MAX_DISTANCE * DIST_SCALE;

class Sonar {
 public:
  Sonar(int t, int e) : trigger_(t), echo_(e) {
    pinMode(trigger_, OUTPUT);
    pinMode(echo_, INPUT);
    digitalWrite(trigger_, LOW);
    delay(1000);
  }
  
  float distance(bool* error) {
    // Send trig pulse
    digitalWrite(trigger_, HIGH);
    delayMicroseconds(20);
    digitalWrite(trigger_, LOW);

    // Wait for echo. Very rarely (2 of 12K at 20Hz)
    // see ECHO never go HIGH so we include a way to
    // bail. 
    int bail = 1000;
    while(digitalRead(echo_) == LOW) {
      if (--bail == 0) {
	*error = true;
	return 0;
      }
    }

    // Measure time for echo. Return early if the
    // pulse is appearing to take too long. Note:
    // error case of never going LOW results in
    // MAX reading :/
    long startTime = micros();
    long travelTime = 0;
    while(digitalRead(echo_) == HIGH) {
      travelTime = micros() - startTime;
      if (travelTime > TRAVEL_TIME_MAX) {
	travelTime = TRAVEL_TIME_MAX;
	break;
      }
      delayMicroseconds(10);
    }
    
    // Return distance in cm
    *error = false;    
    return travelTime / 58.0;
  }
 
private:
  int trigger_;
  int echo_;
};

} // namespace hc_sr04_node

int main(int argc, char **argv) {

  // Start ROS node.
  ROS_INFO("Starting node");
  ros::init(argc, argv, "hc_sr04s");
  ros::NodeHandle node;
  ros::Rate rate(60);  // 10 hz

  // Build N sonars.
  wiringPiSetupSys();  // uses gpio pin numbering
  // TODO: config these
  vector<hc_sr04_node::Sonar> sonars;
  sonars.push_back(hc_sr04_node::Sonar(GPIO_SONAR_0_TRIG, GPIO_SONAR_0_ECHO));
  sonars.push_back(hc_sr04_node::Sonar(GPIO_SONAR_1_TRIG, GPIO_SONAR_1_ECHO));
  sonars.push_back(hc_sr04_node::Sonar(GPIO_SONAR_2_TRIG, GPIO_SONAR_2_ECHO));

  // Build a publisher for each sonar.
  vector<ros::Publisher> sonar_pubs;
  for (int i = 0; i < sonars.size(); ++i) {
    stringstream ss;
    ss << "sonar_" << i;
    sonar_pubs.push_back(node.advertise<hc_sr04::obj_sensor>(ss.str(), 10));
  }
  
    ros::Publisher sonic_0_pubs = node.advertise<std_msgs::Float32>("hc_sr04_range_0", 10);
    ros::Publisher sonic_1_pubs = node.advertise<std_msgs::Float32>("hc_sr04_range_1", 10);
    ros::Publisher sonic_2_pubs = node.advertise<std_msgs::Float32>("hc_sr04_range_2", 10);
  
  // Build base range message that will be used for
  // each time a msg is published.
  hc_sr04::obj_sensor range;
  range.sensor_id = "hc_sr04";
  range.min_range = 0.0;
  range.max_range = hc_sr04_node::MAX_DISTANCE;
 
  float distance;
  bool error;
  std_msgs::Float32 hc_distance_send_0;
  std_msgs::Float32 hc_distance_send_1;
  std_msgs::Float32 hc_distance_send_2;

  #ifdef CSV_TEST
    FILE *fp;
    int i_measure = 0;
    fp = fopen(CSV_FILE_NAME, "w+"); // creates new/blanks the file
    fprintf(fp, "i, raw, filtered\n");
    fclose(fp);
  #endif

  float filterArray[FILTER_ARRAY_SIZE];
  int filterArrayIndex = 0;

  while(ros::ok()) {    
    for (int i = 0; i < sonars.size(); ++i) {
      range.header.stamp = ros::Time::now();
      range.range = sonars[i].distance(&error);
      if (i == 0)
        {
          hc_distance_send_0.data = range.range;
          sonic_0_pubs.publish(hc_distance_send_0);
        }
      if (i == 1)
        {
          hc_distance_send_1.data = range.range;
          sonic_1_pubs.publish(hc_distance_send_1);
        }
      if (i == 2)
        {
          hc_distance_send_2.data = range.range;
          sonic_2_pubs.publish(hc_distance_send_2);
        }
      if (error)
	    ROS_WARN("Error on sonar %d", i);
      else
        //hc_distance_send.data = sonars[1].distance(&error);;
        //sonic_pubs.publish(hc_distance_send);
    	  sonar_pubs[i].publish(range);

      filterArray[filterArrayIndex] = hc_distance_send_0.data;
      filterArrayIndex++;
      filterArrayIndex %= FILTER_ARRAY_SIZE; // keep in range

      #ifdef CSV_TEST
        if (i_measure < 1000)
        {
          fp = fopen(CSV_FILE_NAME, "a"); // open for appending
          fprintf(fp, "%d, %.4f, %.4f\n", i_measure, hc_distance_send_0.data, 
            // filterSample(filterArray, FILTER_ARRAY_SIZE));
            filterSpikeAverage(filterArray, FILTER_ARRAY_SIZE));
          i_measure++;
          fclose(fp);
        }
      #endif

    }
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

float filterSpikeAverage(float array[], int size)
{
  float average = 0;
  for (int i = 0; i < size; i++)
  {
    average += array[i];
  }
  average /= size;

  float averageNoSpike = 0;
  int count = 0;
  float standardDeviation = SAMPLE_STANDARD_DEVIATION;
  for (int i = 0; i < size; i++)
  {
    if ( (array[i] < average * (1 + standardDeviation)) && (array[i] > average * (1 - standardDeviation)) )
    {
      averageNoSpike += array[i];
      count++;
    } else if (array[i] > 0 && array[0] < 60){
      // fix the deviated value to average
      array[i] = average;
    }
  }
  return averageNoSpike/count;
}

float filterSample(float array[], int size)
/*
 *  Written of. too much cpu in use
 */
{
  float sortArray[size];
  for (int i = 0; i < size; i++)
  {
    sortArray[i] = array[i];
  }

  /*
  * Array sorting code
  */
  int changed = 0;
  float temp;
  do
  {
    // keep repeating until no changes occured
    changed = 0;
    for(int i = 0; i<size; i++)
    {
      for(int j = i + 1; j<size; j++)
      {
        /*
        * If there is a smaller element towards right of the array then swap it.
        */
        if(sortArray[j] < sortArray[i])
        {
          temp = sortArray[i];
          sortArray[i] = sortArray[j];
          sortArray[j] = temp;
          changed = 1;
        }
      }
    }
  } while (changed);

  // use center average
  float avg;
  int count;
  for (int i = 4; i < size-5; i++)
  {
    avg += sortArray[i];
    count++;
  }
  avg /= count;
  return avg;
}
