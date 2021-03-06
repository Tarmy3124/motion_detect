#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//for ros
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "motion.h"

using namespace std;
using namespace cv;

bool new_yolomsg=false;
float det_rxL,det_rxR,det_rxT,det_rxB;

    // yolo define
    nav_msgs::Path gui_path;    
    geometry_msgs::PoseStamped this_pose_stamped;
    //maker for rviz 
    visualization_msgs::Marker marker;
    ros::Publisher vis_pub;
    ros::Subscriber dets_sub,mynt_depth;
    //yolo
    //ros::Publisher yolo_point_pub;
    ros::Publisher yolo_person_pub;
    //process mynt
    ros::Publisher points_publisher_;

//yolo dets function
void detsCallback(
const yolofast::DetsPersonPositonPtr &msg){
det_rxL = msg->det_L;
det_rxR = msg->det_R;
det_rxT = msg->det_T;
det_rxB = msg->det_B;
//ROS_INFO("DetsPersonPositon recieve ok");
new_yolomsg =true;
}
//papera -----------------------------------------------
void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Get a pointer to the depth values casting the data
    // pointer to floating point
    cv_bridge::CvImagePtr depth_ptr;
    //from ros to opencv
     depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
   
    //float* depths = (float*)(&msg->data[0]);

 
    // Image coordinates of the center pixel
   if(new_yolomsg ==true){

    int u = (det_rxL+det_rxR)  / 2;
    int v = (det_rxT+det_rxB)/ 2;
    //int u1=(det_rxL+det_rxR+10)  / 2;
    //int v1=(det_rxT+det_rxB+10)/ 2;
   // std::cout<<"u----"<<u<<"v----"<<v;
    std::cout<<"depth_ptr->imageat<uint16_t>(v, u);\n"<<depth_ptr->image.at<float>(v, u)<<std::endl;
   // int centerIdx = u + msg->width * v;
 //   float depth,depthL1,depthL2,center_x=387.029,center_y=241.294,constant_x=0.0027087,constant_y=0.00270966;
static float depth_last,depthL1,depthL2,center_x=317.808,center_y=211.492,constant_x=0.0020104397,constant_y=0.00201100853;
  float depth=depth_ptr->image.at<float>(v, u); //problem

    ROS_INFO("Center distance : %f mm", depth);
    

    cv::Vec3f point;

      point[0] = -(u - center_x) * depth * constant_x ;
      point[1] = -(v - center_y) * depth * constant_y ;
      point[2] = -depth;
     //cout<<"external"<<point<<endl;
     //yolo publish the person trajactory

     //visualization with maker
     //http://wiki.ros.org/rviz/DisplayTypes/Marker#Sphere_.28SPHERE.3D2.29
     //1.1Example Usage
	marker.header.frame_id = "camera_depth_optical_frame";
	marker.header.stamp = msg->header.stamp;
        gui_path.header.seq++;
	marker.ns = "my_yolo_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = point[2]*0.001;
	marker.pose.position.y = point[0]*0.001;
	marker.pose.position.z = point[1]*0.001;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 0.4;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;

    //visualization with path
    //nav_msgs::Path path;
    gui_path.header.stamp=msg->header.stamp;
    gui_path.header.frame_id="camera_depth_optical_frame";
    gui_path.header.seq++;

    this_pose_stamped.header.stamp=msg->header.stamp;
    this_pose_stamped.header.seq++;
    this_pose_stamped.header.frame_id="camera_depth_optical_ frame";
    //this_pose_stamped.header.frame_id=msg->header.frame_id;
    this_pose_stamped.pose.position.x =point[2]*0.001;
      this_pose_stamped.pose.position.y =point[0]*0.001; 
      this_pose_stamped.pose.position.z =point[1]*0.001;   
    
   if (gui_path.poses.size()>5)gui_path.poses.erase(gui_path.poses.begin());
   gui_path.poses.push_back(this_pose_stamped);
     
  yolo_person_pub.publish(gui_path);
  // publishPoints(point, msg);
   vis_pub.publish(marker);
   
 }
   new_yolomsg =false;//姣忔寰幆浠诲姟缁撳熬娑堟伅鏍囧織娓呴櫎
    //ROS_INFO("Center distance : %g m", depths[centerIdx]);
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"motionyolo");   
    ros::NodeHandle nh;

  vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
 //yolo
  yolo_person_pub = nh.advertise<nav_msgs::Path>("personTrajectory",10, true);
//yolo sub
   dets_sub = nh.subscribe("/yoloDetsTx", 10,detsCallback);
   mynt_depth = nh.subscribe("/camera/depth/image_rect_raw",1000,depthCallback);
   ros::spin();

}


