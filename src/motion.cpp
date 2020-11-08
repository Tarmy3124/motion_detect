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

using namespace std;
using namespace cv;

Mat framepro,frame,dframe;
bool flag=false;
int value=25,core=120;
ros::Publisher points_publisher_;
void frameprocess(Mat);
void thres(int,void*){std::cout<<"thres is ok";}
//ROS系统下面订阅ZED摄像头的图像以及深度
//https://blog.csdn.net/yyd__/article/details/98776293
//https://www.jianshu.com/p/9fd15afcb3c8  ros(c++)之cv_bridge库简单使用
void publishPoints(cv::Vec3f &point , const sensor_msgs::Image::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 points_msg;
    points_msg.header.seq = msg->header.seq;
    points_msg.header.stamp = msg->header.stamp;
    //points_msg.header.frame_id = msg->header.frame_id;
    points_msg.header.frame_id = "odom";
    points_msg.width = 10;
    points_msg.height = 10;
    points_msg.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(points_msg);

    modifier.setPointCloud2Fields(
        4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
        sensor_msgs::PointField::FLOAT32, "z", 1,
        sensor_msgs::PointField::FLOAT32, "rgb", 1,
        sensor_msgs::PointField::FLOAT32);

    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    sensor_msgs::PointCloud2Iterator<float> iter_x(points_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(points_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(points_msg, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(points_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(points_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(points_msg, "b");
    
        *iter_x = point[2] * 0.001;
        *iter_y = 0.f - point[0] * 0.001;
        *iter_z = 0.f - point[1] * 0.001;
 //   cout<<"iternal"<<point<<endl;

        *iter_r = static_cast<uint8_t>(255);
        *iter_g = static_cast<uint8_t>(255);
        *iter_b = static_cast<uint8_t>(255);
 points_publisher_.publish(points_msg);
}
void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Get a pointer to the depth values casting the data
    // pointer to floating point
    cv_bridge::CvImagePtr depth_ptr;
     depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
   
    float* depths = (float*)(&msg->data[0]);

    
    // Image coordinates of the center pixel
   
    int u = msg->width / 2;
    int v = msg->height / 2;
    int u1 = msg->width / 2+5;
    int v1 = msg->height / 3-5;
    int u2 = msg->width / 2-5;
    int v2 = msg->height / 3+5;
    std::cout<<"depth_ptr->imageat<uint16_t>(v, u);\n"<<depth_ptr->image.at<float>(v, u)<<std::endl;
   // std::cout<<"depth_ptr->imageat<uint16_t>(v1, u1);\n"<<depth_ptr->image.at<uint16_t>(v1, u1)<<std::endl;
   // std::cout<<"depth_ptr->imageat<uint16_t>(v2, u2);\n"<<depth_ptr->image.at<uint16_t>(v2, u2)<<std::endl;
    int centerIdx = u + msg->width * v;
    float center_x=387.029,center_y=241.294,constant_x=0.0027087,constant_y=0.00270966;
 
    // Linear index of the center pixel

    cv::Vec3f point;
   auto depth=depth_ptr->image.at<float>(v, u);
    //cout<<"depth leixing "<<depth;
    // Output the measure
      point[0] = (u - center_x) * depth * constant_x ;
      point[1] = (v - center_y) * depth * constant_y ;
      point[2] = depth ;
     cout<<"external"<<point<<endl;
     publishPoints(point, msg);
    

    //ROS_INFO("Center distance : %g m", depths[centerIdx]);
}
//process depth from mynt
/*void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Get a pointer to the depth values casting the data
    // pointer to floating point
    cv_bridge::CvImagePtr depth_ptr;
     depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); 
   
    float* depths = (float*)(&msg->data[0]);

 
    // Image coordinates of the center pixel
   
    int u = msg->width / 2;
    int v = msg->height / 3;
    std::cout<<"depth_ptr->imageat<uint16_t>(v, u);\n"<<depth_ptr->image.at<uint16_t>(v, u)<<std::endl;
    int centerIdx = u + msg->width * v;
    float depth,center_x=387.029,center_y=241.294,constant_x=0.0027087,constant_y=0.00270966;
 
    // Linear index of the center pixel

    cv::Vec3f point;
    depth=depth_ptr->image.at<uint16_t>(v, u);
    // Output the measure
      point[0] = (u - center_x) * depth * constant_x ;
      point[1] = (v - center_y) * depth * constant_y ;
      point[2] = depth ;
     cout<<"external"<<point<<endl;
     publishPoints(point, msg);
    

    //ROS_INFO("Center distance : %g m", depths[centerIdx]);
}*/
void callback1(const sensor_msgs::ImageConstPtr &msg)//想了用两个回调函数，中间隔一段时间的办法，但是并不能产生间隔的效果
{
    cv_bridge::CvImagePtr cv_ptr;
    ROS_INFO("now is ok");
    try
    {
        cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        frame=cv_ptr->image;
        waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("error:%s",e.what());
        return ;
    }
    if (!cv_ptr->image.empty())
        {
        if (flag==false) {framepro=cv_ptr->image;flag=true;}
        else {frame=cv_ptr->image;flag=false;}
    absdiff(framepro,frame,dframe);
    frameprocess(dframe);
        }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"motiondet");   
    ros::NodeHandle nh;
   // image_transport::ImageTransport it(nh); 
   // image_transport::Subscriber sub1;
    //sub1=it.subscribe("/camera/rgb/image_raw",1,&callback1);
    ros::Subscriber sub = nh.subscribe("/cam0/image_raw", 1000, callback1);
//sub1=it.subscribe("/cam0/image_raw",1,&callback1);
ros::Subscriber leshi_depth = nh.subscribe("/camera/depth/image", 1000,depthCallback);
///mynteye/depth/image_raw
points_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/md_centerPeople", 1);
//ros::Subscriber leshi_depth = nh.subscribe("/mynteye/depth/image_raw", 1000,depthCallback);

    ros::spin();

}

void frameprocess(Mat dframe)
{
    Mat edge;
    vector<Point> points;
    vector<vector<Point> > contours;
    vector<Vec4i> hier;
    Mat element = getStructuringElement(MORPH_RECT,Size(core,core));
    int i=0;
    imshow("origin",framepro);
  //  cvCvtColor(dframe,dframe,CV_BGR2GRAY);//看了一下例程,知道图像是CV_8UC3的bgr8型，那就是用这个宏了
  //  cvCreateTrackbar("value","view",&value,255,thres);//动作阈值，调value就是调对动作幅度的敏感度
  //  cvCreateTrackbar("core","view",&core,180,thres);//没有用roslaunch而直接用opencv自带的Trackbar函数 感觉可能更方便直观一点？
                                                                                                        //core是腐蚀核，调core就是调目标精细度
    cvtColor(dframe,dframe,CV_BGR2GRAY);//看了一下例程,知道图像是CV_8UC3的bgr8型，那就是用这个宏了
    createTrackbar("value","origin",&value,255,thres);//动作阈值，调value就是调对动作幅度的敏感度
    createTrackbar("core","origin",&core,180,thres);//没有用roslaunch而直接用opencv自带的Trackbar函数 感觉可能更方便直观一点？
                    threshold(dframe,edge,value,255,0);
                    medianBlur(edge,edge,5);//用了一个中值去噪声 效果好像还行。如果对运动精细程度要求比较高就会考虑换用更小的核来滤波
                    dilate(edge,edge,element);//腐蚀函数。在测试的时候因为是拍自己的脑袋，所以我用了大核。核的大小调整直接影响对运动物体大小的探测
                    Canny(edge,edge,3,150);//后面是基本操作，canny取边缘然后在灰度图里画出来
                    findContours(edge,contours,hier,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,Point(0,0));//一个问题：opencv2的findContours函数貌似没有external的选项
                                                                                                                                                                                    //所以我的矩形框里经常有小矩形框
            drawContours(edge,contours,-1,Scalar(255),0,CV_AA);
            for (vector <vector<Point> >::iterator it=contours.begin();it!=contours.end();++it)
                for (vector<Point>::iterator inner=it->begin();inner!=it->end();++inner)
                    {
                        points.push_back(*inner);
                    }
            vector<Rect> boundrect(contours.size());
            for (vector <vector<Point> >::iterator it=contours.begin();it!=contours.end();++it)
            {
                boundrect[i]=boundingRect(Mat(contours[i]));  //boundingRect
                rectangle(framepro,boundrect[i],Scalar(255,255,255),2);
                i++;
            } //外接长方形
            imshow("view",framepro);
}
