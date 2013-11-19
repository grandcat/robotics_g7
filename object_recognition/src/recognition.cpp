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
#include <sys/types.h>
#include <dirent.h>

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
	int obj;
	ImgHist colorHist;
	Features feat;
};

static const std::string objects[] = {"pepper", "lemon", "pear", "carrot", "giraff", "tiger", "hippo"};

vector<IdImg> trainImg;


/*
 * Subtract background based on RGB color
 */
Mat subtractBackground(Mat img)
{
  int rh = 255;
	int gh = rh;
  int bh = rh;

  int rl = 70;
  int gl = rl;
  int bl = rl;

  // for dilation
  Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));

  Mat bgIsolation;
	inRange(img, Scalar(bl, gl, rl), Scalar(bh, gh, rh), bgIsolation);

  bitwise_not(bgIsolation, bgIsolation);

  erode(bgIsolation, bgIsolation, Mat());
  dilate(bgIsolation, bgIsolation, element);

	//namedWindow("background mask");
  //imshow(windowName, bgIsolation);

	return bgIsolation;
}


/*
 * Draw RGB histogram
 */
void drawHistograms(ImgHist hist)
{
	Mat bHist = hist.bHist;
	Mat gHist = hist.gHist;
	Mat rHist = hist.rHist;
	
	int histSize = 256;
  int hist_w = 512; 
  int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );
  Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(bHist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(bHist.at<float>(i)) ),
                       Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(gHist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(gHist.at<float>(i)) ),
                       Scalar( 0, 255, 0), 2, 8, 0  );
      line( histImage, Point( bin_w*(i-1), hist_h - cvRound(rHist.at<float>(i-1)) ) ,
                       Point( bin_w*(i), hist_h - cvRound(rHist.at<float>(i)) ),
                       Scalar( 0, 0, 255), 2, 8, 0  );
  }
  namedWindow("Histogram", CV_WINDOW_AUTOSIZE );
  imshow("Histogram", histImage );
}


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

	for(int i=0; i<10; ++i) {
		bHist.at<float>(i) = 0;
		gHist.at<float>(i) = 0;
		rHist.at<float>(i) = 0;
	}

  // Normalize the result
  int histH = 400;
  normalize(bHist, bHist, 0, histH, NORM_MINMAX, -1, Mat() );
  normalize(gHist, gHist, 0, histH, NORM_MINMAX, -1, Mat() );
  normalize(rHist, rHist, 0, histH, NORM_MINMAX, -1, Mat() );

	ImgHist hist = {bHist, gHist, rHist};
	return hist;
}

/*
 * Show keypoints in image
 */
void showKeypoints(Mat img, Features feat) 
{
	Mat outputImage;
  Scalar keypointColor = Scalar(255, 0, 0);
  drawKeypoints(img, feat.keypoints, outputImage, keypointColor, DrawMatchesFlags::DEFAULT);

  namedWindow("Output");
  imshow("Output", outputImage);
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

	Features feat = { keypoints, descriptors };
	return feat;
}


/*
 * Try to identify the object by comparing edge detection to train images
 */
int identifyObject(ImgHist hist, Features feat)
{
	// Match color histogram
	int bestColorMatch = 0;
	double bestRes = 0;
	for(int i=0; i < trainImg.size(); ++i) {
		double resB = compareHist(hist.bHist, trainImg[i].colorHist.bHist, CV_COMP_CORREL);
		double resG = compareHist(hist.gHist, trainImg[i].colorHist.gHist, CV_COMP_CORREL);
		double resR = compareHist(hist.rHist, trainImg[i].colorHist.rHist, CV_COMP_CORREL);
		std::cout << "Histogram compare B,G,R for " << trainImg[i].obj << ": " << resG << ", " << resB << ", " << resR << std::endl;
		
		// Not a match if negative correlation
		if(resB > 0 && resG > 0 && resR > 0) {
			double res = resB*resB + resG*resG + resR*resR;
			//std::cout << "Res: " << res << std::endl;
			if(res > bestRes) {
				bestRes = res;
				bestColorMatch = trainImg[i].obj;
			}
		}
	}
	std::cout << "Best match color: " << bestColorMatch << std::endl;

	// Match descriptors
	int bestDescriptMatch = 0;
	float matchPercent = 0.4;
	for(int i=0; i < trainImg.size(); ++i) {
		// Use BruteForceMatcher
		BFMatcher matcher(cv::NORM_L1, true);
		vector<cv::DMatch> matches;
		matcher.match(feat.descriptors, trainImg[i].feat.descriptors, matches);

		// Check percent of keypoints that match
		float res = (float)matches.size() / trainImg[i].feat.keypoints.size();
		std::cout << "Percent match for " << trainImg[i].obj << ": " << res << std::endl;

		if(res > matchPercent) {
			matchPercent = res;
			bestDescriptMatch = trainImg[i].obj;
			}
	}
	std::cout << "Best match descriptors: " << bestDescriptMatch << std::endl;

	if(bestDescriptMatch == bestColorMatch) {
		std::cout << "Best match: " << objects[bestDescriptMatch-1] << std::endl;
	} else if(bestDescriptMatch != bestColorMatch && bestColorMatch != 0) {
		std::cout << "Best match: " << objects[bestColorMatch-1] << std::endl;
	} else if(bestColorMatch == 0) {
		std::cout << "Best match: " << objects[bestDescriptMatch-1] << std::endl;
	} else {
		std::cout << "No match" << std::endl;
	}
	
	return 0;
}


/*
 * Train on some images
 */
void train()
{
	// Get all trainimages
	vector<std::string> files;
  class dirent *ent;
  std::string dirName = "src/trainimages";
  DIR *dir = opendir(dirName.c_str());
  while ((ent = readdir(dir)) != NULL) {
    const std::string fileName = ent->d_name;

    // get only image files (.jpg)
  	std::size_t isImg = fileName.find(".jpg");
    if (isImg != std::string::npos) {
      files.push_back(fileName);
  	}
  }
  closedir(dir);

  // Calculate color histogram and feature detection on train images
	for(int i=0; i < files.size(); ++i) {
    const std::string fullFileName = dirName + "/" + files[i];
		Mat img = imread(fullFileName);

		IdImg dummy;
		
		// Get object id from file name
		std::stringstream ss;
  	ss << files[i][0] << files[i][1];
  	int n;
  	std::istringstream(ss.str()) >> n;
		dummy.obj = n;
	
		dummy.colorHist = colorDetection(img);
		dummy.feat = featureDetector(img);
		trainImg.push_back(dummy);
	}
}


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
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

		Rect crop((img->width)/4, (img->height)/2, (img->width)/2, (img->height)*1/2);
		Mat src;
		Mat(img, crop).copyTo(src);

		//Mat src(img); 

		// Detect objects through feature detection
		int featSize = featureDetector(src).keypoints.size();

		// Mask background
		Mat bgMask = subtractBackground(src);
		Mat maskedImg;
		src.copyTo(maskedImg, bgMask);

		//std::cout << "Nr keypoints: " << featSize << std::endl;
		//if(featSize > 10) {
			//std::cout << "OBJECT!" << std::endl;

			// Color detection
			ImgHist hist = colorDetection(maskedImg);

			//Feature detection
			//Features feat = featureDetector(maskedImg);

			// Identify object
			/*int obj = identifyObject(hist, feat);

			image_pub_.publish(obj);*/
		//}

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "object_recognition");
	ros::NodeHandle n;

	initModule_nonfree();	// For using SURF

	// Train on test images
	train();

	// Testing on image
	if(argc > 1) {
		Mat src = imread( argv[1] );

		Mat bgMask = subtractBackground(src);
		Mat maskedImg;
		src.copyTo(maskedImg, bgMask);
		src = maskedImg;

		ImgHist hist = colorDetection(src);
		//drawHistograms(hist);
		Features feat = featureDetector(src);
		showKeypoints(src, feat);
		int obj = identifyObject(hist, feat);
		waitKey(0);
	} else {
	  ImageConverter ic;
	  ros::spin();
	}

  return 0;
}

