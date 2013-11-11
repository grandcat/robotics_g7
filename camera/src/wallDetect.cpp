#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>

#include <string.h>
#include <math.h>		// M_PI
#include <algorithm>    // std::min
#include <camera/Line2D.h> //msg for a Line
#include <camera/Lines2D.h>//msg containing Line(s)


namespace enc = sensor_msgs::image_encodings;


static const char WINDOW[] = "Image window";


class ImageConverter
{
	ros::NodeHandle nh_;
	ros::Publisher vector_lines_pub_;
	ros::Subscriber vector_lines_sub_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	//image_transport::Publisher image_pub_;

	public:
	ImageConverter(): it_(nh_)
	{
		//image_pub_ = it_.advertise("/im_out", 1);
		vector_lines_pub_ = nh_.advertise<camera::Lines2D>("/camera/2Dlines", 1);
		vector_lines_sub_ = nh_.subscribe("/camera/2Dlines", 1, &ImageConverter::wall_checker, this);
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::HoughLinesDetector, this);
		//image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);

		//image_sub_ = it_.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb, this);

		cv::namedWindow(WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(WINDOW);
	}


	void HoughLinesDetector(const sensor_msgs::ImageConstPtr& msg)
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

		cv::Mat src_image, canny_image;
		src_image = cv_ptr->image; //get the source image from the pointer
		//http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
		cv::Canny(src_image, canny_image, 50, 200, 3);


		//http://docs.opencv.org/modules/imgproc/doc/feature_detection.html?highlight=houghlinesp#houghlinesp
		std::vector<cv::Vec4i> HoughLines;
		cv::HoughLinesP( canny_image, HoughLines, 1, CV_PI/180, 80, 30, 10 );


		camera::Lines2D msg_lines; //segmentation fault in for-loop when trying to allocate the array and assigning as <msg_lines[i] = line>?
		//cv::Scalar(Blue,Green,Red)

		//std::cout<<"\n\nFound "<<(int)HoughLines.size()<<" lines!\n"<<std::endl;
		bool isPathFree = true;
		for( size_t i = 0; i < HoughLines.size(); i++ )
		{
			double k = 0;
			int x0 = HoughLines[i][0];
			int y0 = HoughLines[i][1];
			int x1 = HoughLines[i][2];
			int y1 = HoughLines[i][3];

			//calculate the angle in degrees [-90,90) for the line
			k = atan2(y1 - y0 , x1 - x0)*(180/M_PI);

			//For sending the lines as a ROS-message
			camera::Line2D line;
			line.x0 = x0;
			line.y0 = y0;
			line.x1 = x1;
			line.y1 = y1;
			line.angle = k;


			if ( abs(k) <= 90 && abs(k) >= 80 ) //horizontal line |, Blue
			{
				//if there exist a line in the middle (too small passage)
				if (std::max(y0,y1) > 300 && std::max(x0,x1) >= 240 && std::min(x0,x1) <= 400)
				{
					cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,0,255), 3);
					line.description = "wall obstacle";
					isPathFree = false;
					msg_lines.indices_obstacle.push_back(i);
				}
				else{
					cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255,0,0), 3);
					line.description = "wall horizontal";
					msg_lines.indices_horizontal.push_back(i);
				}
			}
			else if ( abs(k) <= 5 && abs(k) >= 0) //vertical line -- , Green
			{
				cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,255,0), 3);
				line.description = "wall vertical";
				msg_lines.indices_vertical.push_back(i);
			}
			else //road, orange and yellow
			{
				if (k < 0 && std::max(y0,y1) > 240 ) //Left side off road (/) orange
				{
					cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,165,255), 3);
					line.description = "road left";
					msg_lines.indices_left.push_back(i);
				}
				else if (k >= 0 && std::max(y0,y1) > 240 ) //Right side of road (\) yellow
				{
					cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,255,255), 3);
					line.description = "road right";
					msg_lines.indices_right.push_back(i);
				}
				else //uninteresting lines, black
				{
					cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,0,0), 3);
					line.description = "uninteresting";
					msg_lines.indices_uninteresting.push_back(i);
				}
			}

			//save the line for the message
			msg_lines.lines.push_back(line);

		}

		if (isPathFree)
			msg_lines.isPathFree = true;
		else
			msg_lines.isPathFree = false;

		cv::imshow("Hough Lines Algorithm", src_image);
		cv::waitKey(3);
		vector_lines_pub_.publish(msg_lines);
		//image_pub_.publish(cv_ptr->toImageMsg());
	}

	void wall_checker(const camera::Lines2D::ConstPtr &msg)
	{
		std::cout<<"\n\nFound "<<(int)msg->lines.size()<<" lines!"<<std::endl;

		std::cout<<"# Left: "<<msg->indices_left.size()<<std::endl;
		for (int i=0; i<msg->indices_left.size(); i++)
		{
			int index = msg->indices_left[i];
			int x0 = msg->lines[index].x0;
			int y0 = msg->lines[index].y0;
			int x1 = msg->lines[index].x1;
			int y1 = msg->lines[index].y1;
			double k = msg->lines[index].angle;
			std::cout<<(int)i<<": start ("<<(int)x0<<","<<(int)y0<<"), end ("<<(int)x1<<","<<(int)y1<<"), slope: "<<k<<std::endl;
		}

		std::cout<<"# Right: "<<msg->indices_right.size()<<std::endl;
		for (int i=0; i<msg->indices_right.size(); i++)
		{
			int index = msg->indices_right[i];
			int x0 = msg->lines[index].x0;
			int y0 = msg->lines[index].y0;
			int x1 = msg->lines[index].x1;
			int y1 = msg->lines[index].y1;
			double k = msg->lines[index].angle;
			std::cout<<(int)i<<": start ("<<(int)x0<<","<<(int)y0<<"), end ("<<(int)x1<<","<<(int)y1<<"), slope: "<<k<<std::endl;
		}
		std::cout<<"# Horizontal: "<<msg->indices_horizontal.size()<<std::endl;
		for (int i=0; i<msg->indices_horizontal.size(); i++)
		{
			int index = msg->indices_horizontal[i];
			int x0 = msg->lines[index].x0;
			int y0 = msg->lines[index].y0;
			int x1 = msg->lines[index].x1;
			int y1 = msg->lines[index].y1;
			double k = msg->lines[index].angle;
			std::cout<<(int)i<<": start ("<<(int)x0<<","<<(int)y0<<"), end ("<<(int)x1<<","<<(int)y1<<"), slope: "<<k<<std::endl;
		}
		std::cout<<"# Vertical: "<<msg->indices_vertical.size()<<std::endl;
		for (int i=0; i<msg->indices_vertical.size(); i++)
		{
			int index = msg->indices_vertical[i];
			int x0 = msg->lines[index].x0;
			int y0 = msg->lines[index].y0;
			int x1 = msg->lines[index].x1;
			int y1 = msg->lines[index].y1;
			double k = msg->lines[index].angle;
			std::cout<<(int)i<<": start ("<<(int)x0<<","<<(int)y0<<"), end ("<<(int)x1<<","<<(int)y1<<"), slope: "<<k<<std::endl;
		}

		std::cout<<"# Uninteresting: "<<msg->indices_uninteresting.size()<<std::endl;

		if (msg->isPathFree == true)
			std::cout<<"The path is free!\n"<<std::endl;
		else
		{
			std::cout<<"There is something blocking the path!\n"<<std::endl;
			std::cout<<"# obstacle: "<<msg->indices_obstacle.size()<<std::endl;
			for (int i=0; i<msg->indices_obstacle.size(); i++)
			{
				int index = msg->indices_obstacle[i];
				int x0 = msg->lines[index].x0;
				int y0 = msg->lines[index].y0;
				int x1 = msg->lines[index].x1;
				int y1 = msg->lines[index].y1;
				double k = msg->lines[index].angle;
				std::cout<<(int)i<<": start ("<<(int)x0<<","<<(int)y0<<"), end ("<<(int)x1<<","<<(int)y1<<"), slope: "<<k<<std::endl;
			}
		}


	}

	/*
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
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

		cv::imshow("derp", cv_ptr->image);
		cv::waitKey(3);

		//image_pub_.publish(cv_ptr->toImageMsg());
	}
	*/
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}

