/*
 *  file: hc_sr04_node.cpp
 *  version: 1.3
 *  date: 12-06-2017
 */

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Float32.h>
#include <hc_sr04/obj_sensor.h>
#include <wiringPi.h>


#define SCHEDULING_RATE 60  // Hz

#define GPIO_SONAR_0_TRIG    24
#define GPIO_SONAR_0_ECHO    25
#define GPIO_SONAR_1_TRIG    22
#define GPIO_SONAR_1_ECHO    23
#define GPIO_SONAR_2_TRIG    18
#define GPIO_SONAR_2_ECHO    27

//#define CSV_TEST
//#define CSV_FILE_NAME "Ultrasonic_measure.csv"

#define FILTER_ARRAY_SIZE 5
#define SAMPLE_STANDARD_DEVIATION 0.2

float filterSample(float array[], int size);
float filterSpikeAverage(float array[], int size);
float distanceLeftRight(bool* error, float &leftValue, float &rightValue);

struct ultrasonicData
{
  float bufferArray[FILTER_ARRAY_SIZE];
  int index;
} ultrasoon0, ultrasoon1, ultrasoon2;

static struct timespec udelay;

using namespace std;

namespace hc_sr04_node 
{

  // Maximum distance reported. Values over this distance
  // report MAX_DISTANCE. TODO make this a property.
  const static float MAX_DISTANCE = 100;
  const static float DIST_SCALE = 58.0;
  const static float TRAVEL_TIME_MAX = MAX_DISTANCE * DIST_SCALE;

  class Sonar 
  {
    public:
      Sonar(int t, int e) : trigger_(t), echo_(e) 
      {
        pinMode(trigger_, OUTPUT);
        pinMode(echo_, INPUT);
        digitalWrite(trigger_, LOW);
        delay(1000);
      }
      
      float distance(bool* error) 
      {
        // Send trig pulse
        digitalWrite(trigger_, HIGH);
        delayMicroseconds(20);
        digitalWrite(trigger_, LOW);

        // Wait for echo. Very rarely (2 of 12K at 20Hz)
        // see ECHO never go HIGH so we include a way to
        // bail. 
        int bail = 1000;
        while(digitalRead(echo_) == LOW) 
        {
          if (--bail == 0) 
          {
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
        while(digitalRead(echo_) == HIGH) 
        {
          travelTime = micros() - startTime;
          if (travelTime > TRAVEL_TIME_MAX) 
          {
          	travelTime = TRAVEL_TIME_MAX;
          	break;
          }
          delayMicroseconds(1);
          // ros::Duration(0.0000001).sleep(); // sleep for around 80 micro sec
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
  ros::Rate rate(SCHEDULING_RATE);  

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
 
  // float distance;
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

  float testfilterArray[FILTER_ARRAY_SIZE];
  // float filterArray[3][FILTER_ARRAY_SIZE];
  int testfilterArrayIndex = 0;
  int bufferClear = 0;
  // int filterArrayIndex[3] = {0, 0, 0};

  long node_start_time = 0;
  long node_stop_time = 0;
  float leftRightValueRead = 0;
  float leftValueRead = 0;
  float rightValueRead = 0;

  while(ros::ok()) {
    node_start_time = micros();    
    for (int i = 0; i < sonars.size(); ++i) {
    // for (int i = 0; i < 0; ++i) {
      range.header.stamp = ros::Time::now();
      
      // Read the sensors
      if (i == 0)
      {
        // measure left and right
        distanceLeftRight(&error, leftValueRead, rightValueRead);
        range.range = leftValueRead;
      }
      else if (i == 1)
      {
        range.range = sonars[i].distance(&error);  
      }
      else if (i == 2)
      {
        range.range = rightValueRead;
      }
      

      if (i == 0)
      {
        ultrasoon0.bufferArray[ultrasoon0.index] = range.range;
        ultrasoon0.index++;
        ultrasoon0.index %= FILTER_ARRAY_SIZE;

        if (bufferClear)
        {
          hc_distance_send_0.data = filterSpikeAverage(ultrasoon0.bufferArray, FILTER_ARRAY_SIZE);
        } else {
          hc_distance_send_0.data = ultrasoon0.bufferArray[ultrasoon0.index];
        }
        
        sonic_0_pubs.publish(hc_distance_send_0);
      }
      
      else if (i == 1)
      {
        ultrasoon1.bufferArray[ultrasoon1.index] = range.range;
        ultrasoon1.index++;
        ultrasoon1.index %= FILTER_ARRAY_SIZE;
        if (bufferClear)
        {
          hc_distance_send_1.data = filterSpikeAverage(ultrasoon1.bufferArray, FILTER_ARRAY_SIZE);
        } else {
          hc_distance_send_1.data = ultrasoon1.bufferArray[ultrasoon1.index];
        }
        sonic_1_pubs.publish(hc_distance_send_1);
      }
      else if (i == 2)
      {
        ultrasoon2.bufferArray[ultrasoon2.index] = range.range;
        ultrasoon2.index++;
        ultrasoon2.index %= FILTER_ARRAY_SIZE;
        if (bufferClear)
        {
          hc_distance_send_2.data = filterSpikeAverage(ultrasoon2.bufferArray, FILTER_ARRAY_SIZE);
        } else {
          hc_distance_send_2.data = ultrasoon2.bufferArray[ultrasoon2.index];
        }
        sonic_2_pubs.publish(hc_distance_send_2);
      }

      if (ultrasoon0.index >= 11)
      {
        bufferClear = 1;
      }

      if (error)
	    ROS_WARN("Error on sonar %d", i);
      else
    	  sonar_pubs[i].publish(range);
//      #ifdef CSV_TEST
//
//        if (i_measure < 1000)
//        {
//          fp = fopen(CSV_FILE_NAME, "a"); // open for appending
//          fprintf(fp, "%d, %.4f\n", i_measure, hc_distance_send_0.data
//            // filterSample(filterArray, FILTER_ARRAY_SIZE));
//            /*,filterSpikeAverage(testfilterArray, FILTER_ARRAY_SIZE)*/);
//          i_measure++;
//          fclose(fp);
//        }
//      #endif
//
    }

    // leftRightValueRead = distanceLeftRight(&error, leftValueRead, rightValueRead);
    // ROS_INFO("LR: %f L: %f R: %f", leftRightValueRead, leftValueRead, rightValueRead);    
    // if (error)
    //   ROS_WARN("Error on sonars");
    // node_stop_time = micros();
    // ROS_INFO("i:%ld, s:%ld, df:%ld", node_start_time, node_stop_time, node_stop_time - node_start_time);
    ros::spinOnce();
    rate.sleep();    
  }
  return 0;
}

float filterSpikeAverage(float array[], int size)
{
  float average = 0;
  int total = 0;
  for (int i = 0; i < size; i++)
  {
    if (array[i] != 0)
    {
      average += array[i];  
      total++;
    }
    
  }
  if (total != 0)
  {
    average /= size;  
  }
  else
  {
    average = 0;
  }
  

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
  if (count != 0)
  {
    return averageNoSpike/count;  
  } 
  return 0;
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

float distanceLeftRight(bool* error, float &leftValue, float &rightValue)
/*
 *  Read left and right ultrasonic sensor at the same time for cpu optimization.
 *  Modifies the leftValue and rightValue on exit with the distances.
 */
{
  const static float MAX_DISTANCE = 60;
  const static float DIST_SCALE = 58.0;
  const static float TRAVEL_TIME_MAX = MAX_DISTANCE * DIST_SCALE;
  // Send trigger pulses
  digitalWrite(GPIO_SONAR_0_TRIG, HIGH);
  digitalWrite(GPIO_SONAR_2_TRIG, HIGH);
  delayMicroseconds(20);
  digitalWrite(GPIO_SONAR_0_TRIG, LOW);
  digitalWrite(GPIO_SONAR_2_TRIG, LOW);

  // Wait for echo. Very rarely (2 of 12K at 20Hz)
  // see ECHO never go HIGH so we include a way to
  // bail. 
  int bail = 1000;
  while((digitalRead(GPIO_SONAR_0_ECHO) == LOW) && (digitalRead(GPIO_SONAR_0_ECHO) == LOW)) 
  {
    if (--bail == 0) 
    {
      *error = true;
      return 0;
    }
  }

  // Measure time for echo. Return early if the
  // pulse is appearing to take too long. Note:
  // error case of never going LOW results in
  // MAX reading :/
  long startTime = micros();
  long travelTime = 0; // check for timeouts
  long travelTimeLeft = 0;
  long travelTimeRight = 0;
  bool readComplete = false;
  bool readLeftComplete = false;
  bool readRightComplete = false;

  while(!readComplete) 
  {
    travelTime = micros() - startTime;
    
    if ((digitalRead(GPIO_SONAR_0_ECHO) == HIGH) && !readLeftComplete)
    {
      travelTimeLeft = travelTime;
    } else {
      readLeftComplete = true;
    }

    if ((digitalRead(GPIO_SONAR_2_ECHO) == HIGH) && !readRightComplete)
    {
      travelTimeRight = travelTime;
    } else {
      readRightComplete = true;
    }

    if (travelTime > TRAVEL_TIME_MAX) 
    {
      travelTime = TRAVEL_TIME_MAX;
      break;
    }

    if (readLeftComplete && readRightComplete)
    {
      readComplete = true;
    } else {
      delayMicroseconds(5);  
    }
  }
  
  // Return distance in cm
  *error = false;
  leftValue = travelTimeLeft / 58.0;
  rightValue = travelTimeRight / 58.0;
  return (leftValue * 100) + rightValue;
}