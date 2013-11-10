#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

struct ImgHist 
{
	Mat bHist;
	Mat gHist;
	Mat rHist;
};

struct Features
{
	vector<KeyPoint> keypoints;
	Mat descriptors;
};

struct IdImg 
{
	Mat img;
	int obj;
	ImgHist colorHist;
	Features feat;
};

const int trainSize = 6;
IdImg testImg[trainSize];


/*
 * Make a color histogram for the image
 */
ImgHist colorDetection(Mat matImg)
{
	//Split image into planes (B, G, R)
	vector<Mat> planes;
	split( matImg, planes );

	int histSize = 256;

	// Set ranges for B,G,R
	float range[] = { 0, 256 } ; //the upper boundary is exclusive
	const float* histRange = { range };
	
	bool uniform = true; 
	bool accumulate = false;

	// Calculate histograms for B,G,R
	Mat bHist, gHist, rHist;
	calcHist(&planes[0], 1, 0, Mat(), bHist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&planes[1], 1, 0, Mat(), gHist, 1, &histSize, &histRange, uniform, accumulate);
	calcHist(&planes[2], 1, 0, Mat(), rHist, 1, &histSize, &histRange, uniform, accumulate);

	ImgHist hist = {bHist, gHist, rHist};
	return hist;
}


/*
 * Detect features in image by finding keypoints
 */
Features featureDetector(Mat img)
{
  Ptr<FeatureDetector> featureDetector = FeatureDetector::create("SURF");
  vector<KeyPoint> keypoints;
  featureDetector->detect(img, keypoints);

	Mat descriptors;
  Ptr<DescriptorExtractor> featureExtractor = DescriptorExtractor::create("SURF");
  featureExtractor->compute(img, keypoints, descriptors);

	/*Mat outputImage;
  Scalar keypointColor = Scalar(255, 0, 0);
  drawKeypoints(img, keypoints, outputImage, keypointColor, DrawMatchesFlags::DEFAULT);

  namedWindow("Output");
  imshow("Output", outputImage);*/

	Features feat = { keypoints, descriptors };
	return feat;
}


/*
 * Try to identify the object by comparing edge detection to train images
 */
void identifyObject(ImgHist hist, Features feat)
{
	// matching color histogram
	int bestColorMatch = 0;
	double diff = 0.3;
	for(int i=0; i < trainSize; ++i) {
		double resB = compareHist(hist.bHist, testImg[i].colorHist.bHist, 3);
		double resG = compareHist(hist.gHist, testImg[i].colorHist.gHist, 3);
		double resR = compareHist(hist.rHist, testImg[i].colorHist.rHist, 3);

		if(resB < diff && resG < diff && resR < diff) {
			bestColorMatch = testImg[i].obj;
			std::cout << "Best match color: " << bestColorMatch << std::endl;
			std::cout << "Histogram compare B,G,R: " << resG << ", " << resB << ", " << resR << std::endl;
		}
	}
	if(bestColorMatch == 0) {
		std::cout << "No match" << std::endl;
	}


	// matching descriptors
	int bestDescriptMatch = 0;
	int nrKeypoints = 10;
	for(int i=0; i < trainSize; ++i) {
		BFMatcher matcher(cv::NORM_L2, true);
		vector<cv::DMatch> matches;
		matcher.match(feat.descriptors, testImg[i].feat.descriptors, matches);

		if(matches.size() > nrKeypoints) {
			bestDescriptMatch = testImg[i].obj;
			std::cout << "Best match descriptors: " << bestDescriptMatch << std::endl;
			std::cout << "Nr matches: " << matches.size() << std::endl;

			if(bestDescriptMatch == bestColorMatch) {
				std::cout << "Best match: " << bestDescriptMatch << std::endl;
				break;
			}
		}
	}

	if(bestDescriptMatch != bestColorMatch && bestColorMatch != 0) {
		std::cout << "Best match: " << bestColorMatch << std::endl;
	} else if(bestColorMatch == 0) {
		std::cout << "Best match: " << bestDescriptMatch << std::endl;
	}

}


/*
 * Train on some images
 */
void train()
{
	ImgHist dummyHist;
	Features edgeDummy;
	testImg[0] = { imread("src/testimages/1_1.jpg"), 1, dummyHist, edgeDummy };
	testImg[1] = { imread("src/testimages/2_1.jpg"), 2, dummyHist, edgeDummy };
	testImg[2] = { imread("src/testimages/3_1.jpg"), 3, dummyHist, edgeDummy };
	testImg[3] = { imread("src/testimages/3_2.jpg"), 3, dummyHist, edgeDummy };
	//testImg[4] = { imread("src/testimages/3_3.jpg"), 3, dummyHist, edgeDummy };
	testImg[4] = { imread("src/testimages/4_1.jpg"), 4, dummyHist, edgeDummy };
	testImg[5] = { imread("src/testimages/4_2.jpg"), 4, dummyHist, edgeDummy };
	//testImg[7] = { imread("src/testimages/4_3.jpg"), 4, dummyHist, edgeDummy };

	Mat img = imread( "src/testimage.jpg" );
	for(int i=0; i < trainSize; ++i) {
		resize(testImg[i].img, testImg[i].img, img.size());

		testImg[i].colorHist = colorDetection(testImg[i].img);

		testImg[i].feat = featureDetector(testImg[i].img);
	}
}




class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
	image_transport::Subscriber image_sub_depth_;

public:
  IplImage* img;
	IplImage* img_depth;
	IplImage* hsv_image;
	IplImage* hsv_mask;

  ImageConverter()
    : it_(nh_)
  {

	image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
	//image_sub_depth_ = it_.subscribe("/camera/depth/image_rect", 1, &ImageConverter::imageCb_depth, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);	//RGB image
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //cv_ptr has image now, identify object
		img = new IplImage(cv_ptr->image);
		Mat src(img); 
		
		Mat refSize = imread( "src/testimage.jpg" );	
		resize(src, src, refSize.size());

		// Detect if objects through feature detection
		if(featureDetector(src).keypoints.size() > 20) {
			// Color detection
			ImgHist hist = colorDetection(src);

			//Feature detection
			Features feat = featureDetector(src);

			// Identify object
			identifyObject(hist, feat);
		}

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);

    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char** argv)
{
  //ros::init(argc, argv, "image_converter");
	ros::init(argc, argv, "object_recognition");
	ros::NodeHandle n;

	initModule_nonfree();

	// Train on test images
	train();

  ImageConverter ic;
  ros::spin();
  return 0;
}

