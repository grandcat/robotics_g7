// Used for testing on images

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
	//Mat edges;
	Features feat;
};



//static const char WINDOW[] = "Image window";
const int trainSize = 6;
IdImg testImg[trainSize];


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
 * Perform Canny Edge Detection on the image (Not used)
 */
/*Mat edgeDetection(Mat img)
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

	//imshow("Edge map", finalImg);

	return finalImg;
}*/


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

	int bestDescriptMatch = 0;
	int nrKeypoints = 10;
	for(int i=0; i < trainSize; ++i) {
		// matching descriptors
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

		/*cvtColor( testImg[i].img, testImg[i].img, CV_BGR2GRAY );
		testImg[i].edges = edgeDetection(testImg[i].img);*/
	}
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "recognition");
	ros::NodeHandle n;

	initModule_nonfree();

	// Train on test images
	train();

	// Load an image
	Mat img = imread( "src/testimage.jpg" );
	Mat src = imread( argv[1] );
	resize(src, src, img.size());

	if( !src.data )
	{ return -1; }

	// Color detection
	ImgHist hist = colorDetection(src);

	//Feature detection
	Features feat = featureDetector(src);

	/// Convert the image to grayscale
	//cvtColor( src, src, CV_BGR2GRAY );

	// Canny Edge Detector
//	Mat edges = edgeDetection(src);

	// Identify object
	identifyObject(hist, feat);


	// matching descriptors
	/*BFMatcher matcher(cv::NORM_L2, true);
	vector<cv::DMatch> matches;
	matcher.match(feat.descriptors, testImg[0].feat.descriptors, matches);

	// drawing the results
	namedWindow("matches", 1);
	Mat img_matches;
	drawMatches(src, feat.keypoints, testImg[0].img, testImg[0].feat.keypoints, matches, img_matches);
	imshow("matches", img_matches);*/

	/// Wait until user exit program by pressing a key
	waitKey(0);

	return 0;
}
