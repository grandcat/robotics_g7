#include "wall_detection.h"
struct Line2D; //forward declare the struct Line2D

namespace enc = sensor_msgs::image_encodings;



//Each object of Wall_Detection will contain the information about found lines in one image

Wall_Detection::Wall_Detection(): it_(nh_)
{
	lines_pub_ = nh_.advertise<wall_detection::Lines2D>("/image/2Dlines", 1);
	image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Wall_Detection::HoughLinesDetector, this);

	if (FLAG_SHOW_OUTPUT)
		lines_sub_ = nh_.subscribe("/image/2Dlines", 1, &Wall_Detection::wall_checker, this);

	cv::namedWindow(WINDOW);
}

Wall_Detection::~Wall_Detection()
{
	cv::destroyWindow(WINDOW);
}


void Wall_Detection::HoughLinesDetector(const sensor_msgs::ImageConstPtr& msg)
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


	wall_detection::Lines2D msg_lines; //segmentation fault in for-loop when trying to allocate the array and assigning as <msg_lines[i] = line>?


	lines_.clear();
	indices_left.clear();
	indices_right.clear();
	indices_horizontal.clear();
	indices_vertical.clear();
	indices_obstacle.clear();
	indices_uninteresting.clear();

	lines_.reserve(HoughLines.size()); //allocate the number of lines found with the HougLinesP algorithm

	//cv::Scalar(Blue,Green,Red)

	//std::cout<<"\n\nFound "<<(int)HoughLines.size()<<" lines!\n"<<std::endl;
	isPathFree = true;

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
		wall_detection::Line2D msg_line;
		msg_line.x0 = x0;
		msg_line.y0 = y0;
		msg_line.x1 = x1;
		msg_line.y1 = y1;
		msg_line.angle = k;

		//save the struct Line2D object
		Line2D line_;
		line_.x0=x0;
		line_.y0=y0;
		line_.x1=x1;
		line_.y1=y1;
		line_.angle=k;


		if ( abs(k) <= 90 && abs(k) >= 80 ) //horizontal line |, Blue
		{
			//if there exist a line in the middle (too small passage)
			if (std::max(y0,y1) > 300 && std::max(x0,x1) >= 240 && std::min(x0,x1) <= 400)
			{
				cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,0,255), 3);
				isPathFree = false;
				msg_line.description = "wall obstacle";
				msg_lines.indices_obstacle.push_back(i);
				line_.description = "wall obstacle";
				indices_obstacle.push_back(i);
			}
			else{
				cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255,0,0), 3);
				msg_line.description = "wall horizontal";
				msg_lines.indices_horizontal.push_back(i);
				line_.description = "wall horizontal";
				indices_horizontal.push_back(i);
			}
		}
		else if ( abs(k) <= 5 && abs(k) >= 0) //vertical line -- , Green
		{
			cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,255,0), 3);
			msg_line.description = "wall vertical";
			msg_lines.indices_vertical.push_back(i);
			line_.description = "wall vertical";
			indices_vertical.push_back(i);
		}
		else //road, orange and yellow
		{
			if (k < 0 && std::max(y0,y1) > 240 ) //Left side off road (/) orange
			{
				cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,165,255), 3);
				msg_line.description = "road left";
				msg_lines.indices_left.push_back(i);
				line_.description = "road left";
				indices_left.push_back(i);
			}
			else if (k >= 0 && std::max(y0,y1) > 240 ) //Right side of road (\) yellow
			{
				cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,255,255), 3);
				msg_line.description = "road right";
				msg_lines.indices_right.push_back(i);
				line_.description = "road right";
				indices_right.push_back(i);
			}
			else //uninteresting lines, black
			{
				cv::line( src_image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,0,0), 3);
				msg_line.description = "uninteresting";
				msg_lines.indices_uninteresting.push_back(i);
				line_.description = "uninteresting";
				indices_uninteresting.push_back(i);
			}
		}

		//save the line for the message
		msg_lines.lines.push_back(msg_line);
		//save the line in the lines vector
		//lines_.push_back(line_);

		//lines_[i] = line_;
		lines_.push_back(line_);


	}

	if (isPathFree)
		msg_lines.isPathFree = true;
	else
		msg_lines.isPathFree = false;

	if (FLAG_SHOW_IMAGE)
	{
		cv::imshow("Hough Lines Algorithm", src_image);
		cv::waitKey(3);
	}
	lines_pub_.publish(msg_lines);

}

void Wall_Detection::wall_checker(const wall_detection::Lines2D::ConstPtr &msg)
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

	//std::cout<<"FROM ATTRIBUTE "<<(bool)isPathFree<<std::endl;
	//std::cout<<"Found: "<<lines_.size()<<" lines!"<<std::endl;
}








int main(int argc, char* argv[])
{
	ros::init(argc, argv, "wall_detector");
	if (argc != 1)
	{
		if (atoi(argv[1]) == 1)
		{
			FLAG_SHOW_IMAGE = true;
		}
		else if (atoi(argv[1]) >= 2)
		{
			FLAG_SHOW_IMAGE = true;
			FLAG_SHOW_OUTPUT = true;
		}
	}
	Wall_Detection wd;
	std::cout<<"Package with line-information are being published to topic /image/2Dlines/"<<std::endl;
	ros::spin();
	return 0;
}

