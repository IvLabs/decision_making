#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "apriltags/AprilTagDetections.h"

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"

#include "std_msgs/String.h"
#include "localization_fira/posi.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>
#include "std_msgs/Int8.h"

#include <SerialPort.h>
#include <iostream>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>


/*
flags(from dmn to parser)
1=initial_forward
2=forward walk
3=left turn from initial position
4=right turn from initial position
5=back walk
6=leg from back to front (to obtain initial position)
7=initial_back
8=leg from front to back (to obtain initial position)
status (from parser) on completing the gait 
11
21
31.. indicates completion of the respective gaits
*/

int stat=0; int k=1; int back=0; int loop=0;
std_msgs::Int8 flag;
float z_threshold; float yaw_threshold; //to be set according to robot behavior
// if it crosses a particular yaw_threshold it should start turning 
// if it travelles more than z_threshold it should start walking back
  
float error_yaw; // allowable yaw error
float z_reference; float yaw_reference; //taken at the start line in ready state
float z_current; float yaw_current; 
int start_back_walk = 0; int start_front_walk = 0; int switch_val=0;
float tag_endline = 0.3; //distance between april tag and endline(in meters) //reduce it (0.25)


void pose_callback(const visualization_msgs::Marker::ConstPtr& marker) //reduce the speed at which aprilTag node is publishing data. Callback should not be busy
//const apriltags::AprilTagDetections::ConstPtr& detections)
{
  
  if (switch_val == 1){ // if ready switch is pressed(say switch 1). This will be true till start switch is not pressed
    z_reference = marker->pose.position.z; //threshold before start switch(to activate initial_step node) is not pressed
    yaw_reference = marker->pose.orientation.z;    
   }

  else if (switch_val==2) //&& if start switch is pressed(say switch 2)
   {
    z_current=marker->pose.position.z;
    yaw_current=marker->pose.orientation.z;

    if (back==0)
      start_front_walk = 1;
    else if(z_current < tag_endline)
      start_back_walk = 1; back = 1;
   }
}


void status_callback(const std_msgs::Int8::ConstPtr& msg) //status of completion of parser
{ 
  stat = msg->data; //at a time status is received by one parsernode only.
  loop=1;
}

void orient(int init_step_f, int left_turn_f, int right_turn_f)
{
        if (yaw_current < abs(yaw_reference - yaw_threshold ))
          flag.data=init_step_f; //take front_initial_step or back_initial_step
        else if (yaw_current > abs(yaw_reference - yaw_threshold ))
          {
           if (yaw_current > (yaw_reference - yaw_threshold ))//left turn
             flag.data=left_turn_f;
           else if (yaw_current < (yaw_reference - yaw_threshold ))//right turn
             flag.data=right_turn_f;
           }
} 

void turn()
{
        if (yaw_current > (yaw_reference - yaw_threshold ))//left turn
             flag.data=3;
        else if (yaw_current < (yaw_reference - yaw_threshold ))//right turn
             flag.data=4;
}

void correct_dev(int walk_back_f, int front_to_back_f)
{
        if (yaw_current < abs(yaw_reference - yaw_threshold ))
          flag.data=walk_back_f; //continue to walk back or front
        else if (yaw_current > abs(yaw_reference - yaw_threshold ))
          {
           flag.data = front_to_back_f; //get foot from front_to_back or back to front
          }        
}


/////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
   ros::init(argc, argv, "dmn");//node name=dmn
   ros::NodeHandle n; ros::NodeHandle f; ros::NodeHandle c;

   ros::Subscriber sub_status = n.subscribe("flag_to_dmn",1, status_callback);
   ros::Subscriber sub_pose = c.subscribe("/apriltags/marker", 1, pose_callback);
   ros::Publisher pub_flag = f.advertise<std_msgs::Int8>("flag_to_nodes", 10); usleep(1000*1000);

   std_msgs::Int8 msg;
   std_msgs::Int8 status;
   // INCLUDE SERIAL READ FROM TIVA 
   
   SerialPort s("/dev/ttyACM0");
   SerialPort::BaudRate b;
   b=s.BAUD_9600;
   s.Open(b);
ros::Rate loop_rate(10);
while (ros::ok())
{
 ros::spinOnce();
 msg.data =s.ReadByte();
 switch_val=msg.data;

 if (switch_val == 1){
        flag.data = 0; //start conveyor (ready position) store flag.data in a variable in conveyor and let it be 0 or true always.
        loop=1;
   }
 

if(start_front_walk == 1) 
{
   if(k>0){ 
        flag.data=1; //front_initial_step 
        loop=1;
     } 
   if (stat==11) //initial_step_complete
     flag.data=2; //initiate front_walk
   else if (stat==21)// 'x' walking_steps_complete
     correct_dev(2,6); // correct deviation after 'x' steps
   else if (stat==61) //foot at initial position
     turn(); //turn
   else if (stat==31 || stat==41) //one step turn complete
      orient(1,3,4);
}  

////////////////////////////////////////////////////////////////////////

if(start_back_walk == 1) 
{
  if (stat==21 || stat==11) //one leg in front and other at back
   flag.data = 6; //back to front
  else if (stat==61) //foot at initial position
   flag.data = 7; //back_initial
  else if (stat==71) //initial_back_step complete
   flag.data = 5; //start back_walk
  else if (stat==51)//'x' back_walking_steps_complete
     correct_dev(5,8);
  else if (stat==81) //foot at initial position
     turn();
  else if (stat==31 || stat==41)
     orient(7,3,4);
}

////////////////////////////////////////////////////////////////////////
 k--;

 if(loop>0)
       {
        pub_flag.publish(flag); 
        loop--;
       }
}
 return 0;
}

  


