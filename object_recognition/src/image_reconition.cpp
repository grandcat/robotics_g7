#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";
Mat trainingImg[];


/*
 * Try to identify the object by comparing edge detection to train images
 */
void identifyObject(Mat edges)
{

}


/*
 * Perform Canny Edge Detection on the image
 */
void edgeDetection(Mat img)
{
	//noise reduction with gaussian filter
	Mat edges;
	int ksize = 5;
	blur(img, edges, Size(ksize, ksize));

	// set tresholds
	int lowThreshold = 100;
	int upperThreshold = lowThreshold*3;

	// Canny Edge Detection
	Canny(edges, edges, lowThreshold, upperThreshold, ksize);
	Mat finalImg;
	finalImg.create( img.size(), img.type() );
	finalImg = Scalar::all(0);
	img.copyTo(finalImg, edges);

	imshow("Edge map", finalImg);

	identifyObject(edges);
}


/*
 * Train on some images
 */
void trainOnImages()
{

	Mat trainingImg[0] = imread("/trainingimages/1_1.jpg");


}




int main( int argc, char** argv )
{
	ros::init(argc, argv, "recognition");
	ros::NodeHandle n;

	// Load an image
	Mat src = imread( argv[1] );

	if( !src.data )
	{ return -1; }

	/// Convert the image to grayscale
	cvtColor( src, src, CV_BGR2GRAY );

	/// Show the image
	edgeDetection(src);

	/// Wait until user exit program by pressing a key
	waitKey(0);

	return 0;
}
