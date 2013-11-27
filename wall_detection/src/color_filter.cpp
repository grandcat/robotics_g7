#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>

#include <string.h>
#include "std_msgs/String.h"
#include <sstream>      //std::stringstream
#include <stdio.h>      //getchar()
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

namespace enc = sensor_msgs::image_encodings;


static const char WINDOW[] = "Image window";


//initial min and max HSV filter values.
//these will be changed using trackbars
int CHANGE = 10;
int MAX = 256;
int H_MIN = 100;
int H_MAX = MAX;
int S_MIN = 50;
int S_MAX = MAX;
int V_MIN = 70;
int V_MAX = MAX;


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber keyboard_sub;

	public:
	ImageConverter(): it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::color_filter, this);
		keyboard_sub = nh_.subscribe("/keyboard/input", 1, &ImageConverter::ChangeThreshold, this);
		cv::namedWindow(WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(WINDOW);
	}

	void ChangeThreshold(const std_msgs::String::ConstPtr& msg)
	{
		//std::cout<<msg->data.c_str()<<std::endl;
		//Hue-part
		if (strcmp(msg->data.c_str(),"a")==0 || strcmp(msg->data.c_str(),"A")==0)
		{
			H_MIN -= CHANGE;
			if (H_MIN < 0)
				H_MIN = 0;
		}
		else if (strcmp(msg->data.c_str(),"q")==0 || strcmp(msg->data.c_str(),"Q")==0)
		{
			H_MIN += CHANGE;
			if (H_MIN > H_MAX)
				H_MIN = H_MAX;
		}
		else if (strcmp(msg->data.c_str(),"s")==0 || strcmp(msg->data.c_str(),"S")==0)
		{
			H_MAX -= CHANGE;
			if (H_MAX < H_MIN)
				H_MAX = H_MIN;
		}
		else if (strcmp(msg->data.c_str(),"w")==0 || strcmp(msg->data.c_str(),"W")==0)
		{
			H_MAX += CHANGE;
			if (H_MAX > MAX)
				H_MAX = MAX;
		}

		//Saturate-part
		else if (strcmp(msg->data.c_str(),"d")==0 || strcmp(msg->data.c_str(),"D")==0)
		{
			S_MIN -= CHANGE;
			if (S_MIN < 0)
				S_MIN = 0;
		}
		else if (strcmp(msg->data.c_str(),"e")==0 || strcmp(msg->data.c_str(),"E")==0)
		{
			S_MIN += CHANGE;
			if (S_MIN > S_MAX)
				S_MIN = S_MAX;
		}
		else if (strcmp(msg->data.c_str(),"f")==0 || strcmp(msg->data.c_str(),"F")==0)
		{
			S_MAX -= CHANGE;
			if (S_MAX < S_MIN)
				S_MAX = S_MIN;
		}
		else if (strcmp(msg->data.c_str(),"r")==0 || strcmp(msg->data.c_str(),"R")==0)
		{
			S_MAX += CHANGE;
			if (S_MAX > MAX)
				S_MAX = MAX;
		}

		//Value-part
		else if (strcmp(msg->data.c_str(),"g")==0 || strcmp(msg->data.c_str(),"G")==0)
		{
			V_MIN -= CHANGE;
			if (V_MIN < 0)
				V_MIN = 0;
		}
		else if (strcmp(msg->data.c_str(),"t")==0 || strcmp(msg->data.c_str(),"T")==0)
		{
			V_MIN += CHANGE;
			if (V_MIN > V_MAX)
				V_MIN = V_MAX;
		}
		else if (strcmp(msg->data.c_str(),"h")==0 || strcmp(msg->data.c_str(),"H")==0)
		{
			V_MAX -= CHANGE;
			if (V_MAX < V_MIN)
				V_MAX = V_MIN;
		}
		else if (strcmp(msg->data.c_str(),"y")==0 || strcmp(msg->data.c_str(),"Y")==0)
		{
			V_MAX += CHANGE;
			if (V_MAX > MAX)
				V_MAX = MAX;
		}
		std::cout<<"H_MIN: "<< H_MIN<<" H_MAX: "<<H_MAX<< " S_MIN: "<<S_MIN<<" S_MAX: "<<S_MAX<<" V_MIN: "<<V_MIN<<" V_MAX: "<<V_MAX<<std::endl;
	}

	void color_filter(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::Mat src_image, hsv_image, threshold_image;
		src_image = cv_ptr->image; //get the source image from the pointer
		cv::cvtColor(src_image, hsv_image,CV_BGR2HSV);
		cv::inRange(hsv_image,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),threshold_image);
		//cv::inRange(hsv_image,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),hsv_image);
		cv::imshow("Original", src_image);
		cv::imshow("HSV", hsv_image);
		cv::imshow("THRESHOLD", threshold_image);
		cv::waitKey(3);
	}




};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();

	return 0;
}

