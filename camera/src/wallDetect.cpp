#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/im_out", 1);
    //image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    //loadAndShowImage();
    //EdgeDetector();
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::HoughLinesDetector, this);
    //image_sub_ = it_.subscribe("/camera/depth/image", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void loadAndShowImage()
  {
	  cv::Mat image;
	  image = cv::imread("/home/findus/Bilder/nyancat.png", CV_LOAD_IMAGE_COLOR);   // Read the file
	  if(! image.data )                              // Check for invalid input
	  {
		  std::cout <<  "Could not open or find the image" << std::endl ;
	  }

	  cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
	  imshow( "Display window", image );                   // Show our image inside it.
	  cv::waitKey(0);                                          // Wait for a keystroke in the window
	  cv::destroyWindow(WINDOW);
  }

  void EdgeDetector()
  {
	  cv::Mat src, scr_small, dst, color_dst;
	  //src = cv::imread("/home/findus/Bilder/nyancat.png", CV_LOAD_IMAGE_COLOR);
	  src = cv::imread("/home/findus/Bilder/inside_maze.jpg", CV_LOAD_IMAGE_GRAYSCALE);   // Read the file
	  if(! src.data )                              // Check for invalid input
	  {
		  std::cout <<  "Could not open or find the image" << std::endl ;
	  }

	  cv::resize(src, scr_small, cv::Size(500,500),CV_INTER_AREA);

	  //http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
	  cv::Canny(scr_small, dst, 50, 200, 3);
	  cvtColor( dst, color_dst, CV_GRAY2BGR ); //single channel to 3 channel

	  //http://docs.opencv.org/modules/imgproc/doc/feature_detection.html?highlight=houghlinesp#houghlinesp
	  std::vector<cv::Vec4i> lines;
	  cv::HoughLinesP( dst, lines, 1, CV_PI/180, 80, 30, 10 );
	  for( size_t i = 0; i < lines.size(); i++ )
	  {
		  cv::line( scr_small, cv::Point(lines[i][0], lines[i][1]),
				  cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
	  }

	  //cv::namedWindow( "Display Hough Lines", CV_WINDOW_AUTOSIZE );// Create a window for display.
	  imshow( "Hough Lines", scr_small );                   // Show our image inside it.
	  cv::waitKey(0);                                          // Wait for a keystroke in the window
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

    cv::Mat src, dst, color_dst;
    src = cv_ptr->image; //get the source image from the pointer
	//http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/canny_detector/canny_detector.html
	cv::Canny(src, dst, 50, 200, 3);
	//cvtColor( dst, color_dst, CV_GRAY2BGR ); //single channel to 3 channel

	//http://docs.opencv.org/modules/imgproc/doc/feature_detection.html?highlight=houghlinesp#houghlinesp
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP( dst, lines, 1, CV_PI/180, 80, 30, 10 );
	for( size_t i = 0; i < lines.size(); i++ )
	{
	  cv::line( src, cv::Point(lines[i][0], lines[i][1]),
			  cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255,0,0), 3, 8 );
	}
    cv::imshow("Hough Lines Algorithm", src);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }

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

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
