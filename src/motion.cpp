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

//for ncnn
#include "benchmark.h"
#include "cpu.h"
#include "datareader.h"
#include "net.h"
#include "gpu.h"


#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

bool new_yolomsg=false;
float det_rxL,det_rxR,det_rxT,det_rxB;
cv::Mat frame_G;
//tarmy data
//ncnn-----------------------------------------
int demo(cv::Mat& image, ncnn::Net &detector, int detector_size_width, int detector_size_height)
{
/*
    static const char* class_names[] = {"background",
                                        "aeroplane", "bicycle", "bird", "boat",
                                        "bottle", "bus", "car", "cat", "chair",
                                        "cow", "diningtable", "dog", "horse",
                                        "motorbike", "person", "pottedplant",
                                        "sheep", "sofa", "train", "tvmonitor","basketball"
                                       };
*/
    //basketball
    static const char* class_names[] = {"background",
                                        "basketball"
                                       };
    

    cv::Mat bgr = image.clone();
    int img_w = bgr.cols;
    int img_h = bgr.rows;

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB,\
                                                 bgr.cols, bgr.rows, detector_size_width, detector_size_height);

    //数据预处理
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    in.substract_mean_normalize(mean_vals, norm_vals);

    ncnn::Extractor ex = detector.create_extractor();
    ex.set_num_threads(2);
    ex.input("data", in);
    ncnn::Mat out;
    ex.extract("output", out);

    for (int i = 0; i < out.h; i++)
    {
        int label;
        float x1, y1, x2, y2, score;
        float pw,ph,cx,cy;
        const float* values = out.row(i);

        
        x1 = values[2] * img_w;
        y1 = values[3] * img_h;
        x2 = values[4] * img_w;
        y2 = values[5] * img_h;

        score = values[1];
        label = values[0];
        //if(strcmp(class_names[label],"person")!=0)continue; //only select person label
        //basketbal
        if(strcmp(class_names[label],"basketball")!=0)continue; //only select basketball
        //处理坐标越界问题
        if(x1<0) x1=0;
        if(y1<0) y1=0;
        if(x2<0) x2=0;
        if(y2<0) y2=0;

        if(x1>img_w) x1=img_w;
        if(y1>img_h) y1=img_h;
        if(x2>img_w) x2=img_w;
        if(y2>img_h) y2=img_h;
        
        det_rxL=x1;
        det_rxR=x2;
        det_rxB=y1;
        det_rxT=y2;
        new_yolomsg=true; //we got the person positon !

        cv::rectangle (image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 0), 1, 1, 0);
        //Debug
       // std::cout<<x1<<'\n'<<x2<<'\n'<<y1<<'\n'<<y2<<'\n';
        

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[label], score * 100);
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        cv::putText(image, text, cv::Point(x1, y1 + label_size.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }
    return 0;
}

int test_cam()
{
    //定义yolo-fastest VOC检测器
    ncnn::Net detector;  
    detector.load_param("/home/shunya/ros/papera/catkin_yolo/src/motin_detect/src/model/yolo-fastest.param");
    detector.load_model("/home/shunya/ros/papera/catkin_yolo/src/motin_detect/src/model/yolo-fastest.bin");
    int detector_size_width  = 320;
    int detector_size_height = 320;

    cv::Mat frame;
    cv::VideoCapture cap(0);

    while (true)
    {
        cap >> frame;
        double start = ncnn::get_current_time();
        demo(frame, detector, detector_size_width, detector_size_height);
        double end = ncnn::get_current_time();
        double time = end - start;
        printf("Time:%7.2f \n",time);
        cv::imshow("demo", frame);
        cv::waitKey(1);
    }
    return 0;
}
//ncnn-----------------------------------------






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
    int u1=(det_rxL+det_rxR+3)  / 2;
    int v1=(det_rxT+det_rxB+3)/ 2;
   // std::cout<<"u----"<<u<<"v----"<<v;
    std::cout<<"depth_ptr->imageat<uint16_t>(v, u);\n"<<depth_ptr->image.at<float>(v, u)<<std::endl;
   // int centerIdx = u + msg->width * v;
 //   float depth,depthL1,depthL2,center_x=387.029,center_y=241.294,constant_x=0.0027087,constant_y=0.00270966;
static float depth_last,depthL1,depthL2,center_x=317.808,center_y=211.492,constant_x=0.0020104397,constant_y=0.00201100853;
  float depth_sort[3];
   depth_sort[0]=depth_ptr->image.at<float>(v, u); //center
   depth_sort[1]=depth_ptr->image.at<float>(v1, u); //center+5
   depth_sort[2]=depth_ptr->image.at<float>(v, u1); //center
  float sort_temp=0.0f;  
for (int i_sort=0;i_sort<2;i_sort++){
  if(depth_sort[i_sort]>depth_sort[i_sort+1]){
  sort_temp=depth_sort[i_sort];
  depth_sort[i_sort]=depth_sort[i_sort+1];
  depth_sort[i_sort+1]=sort_temp;
  }
}
  float depth = depth_sort[1];
  std::cout<<"depth_sort is \n"<<depth_sort<<std::endl;

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
    
   if (gui_path.poses.size()>200)gui_path.poses.erase(gui_path.poses.begin());
   gui_path.poses.push_back(this_pose_stamped);
     
  yolo_person_pub.publish(gui_path);
  // publishPoints(point, msg);
   vis_pub.publish(marker);
   
 }
   new_yolomsg =false;//姣忔寰幆浠诲姟缁撳熬娑堟伅鏍囧織娓呴櫎
    //ROS_INFO("Center distance : %g m", depths[centerIdx]);
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
     cv_bridge::CvImagePtr cv_ptr;
     cv::Mat frame;
     try
       {
        cout<<"ok";
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      //frame= cv_bridge::toCvCopy(msg, msg->encoding)->image;
      frame_G= cv_bridge::toCvCopy(msg, msg->encoding)->image;
      
   


   // cv::VideoCapture cap(0);

   // while (true)
   // {
       // cap >> frame;
       // frame = cv_ptr->image;
    /*    double start = ncnn::get_current_time();
        demo(frame, detector, detector_size_width, detector_size_height);

        cv::imshow("demo", frame);
        cv::waitKey(1);
        double end = ncnn::get_current_time();
        double time = end - start;
        printf("Time:%7.2f \n",time);
    */ 
        // emit Show_image(0,im);
       }
       catch (cv_bridge::Exception& e)
       {

        // log(Error,("video frame0 exception: "+QString(e.what())).toStdString());
        cout<<"something wrong with video input"; 
        return;
       }
 }
int main(int argc,char** argv)
{
    //test_cam(); //_ncnn
    //return 0;
    ros::init(argc,argv,"motionyolo");   
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);
    image_transport::ImageTransport it_(nh);
    image_transport::Subscriber image_sub;
    image_sub=it_.subscribe("/camera/color/image_raw",100,imageCallback,image_transport::TransportHints("raw"));
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
 //yolo
  yolo_person_pub = nh.advertise<nav_msgs::Path>("personTrajectory",10, true);
//yolo sub
   dets_sub = nh.subscribe("/yoloDetsTx", 10,detsCallback);
   mynt_depth = nh.subscribe("/camera/depth/image_rect_raw",1000,depthCallback);

     ncnn::Net detector;
    detector.load_param("/home/shunya/ros/papera/catkin_yolo/src/motin_detect/src/model/yolo-fastest.param");
    detector.load_model("/home/shunya/ros/papera/catkin_yolo/src/motin_detect/src/model/yolo-fastest.bin");
    int detector_size_width  = 320;
    int detector_size_height = 320;
      while (ros::ok())
    {
       // cap >> frame;
        double start = ncnn::get_current_time();
        demo(frame_G, detector, detector_size_width, detector_size_height);
        //cv::imshow("demo", frame_G);
        double end = ncnn::get_current_time();
        double time = end - start;
        printf("Time:%7.2f \n",time);
        //cv::waitKey(1);  //not necessary? 
        ros::spinOnce();
        loop_rate.sleep();
     }
   

}


