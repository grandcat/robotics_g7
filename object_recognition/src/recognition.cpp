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
#include "std_msgs/String.h"
#include <fstream>
#include <boost/algorithm/string.hpp>

#include <algorithm> //for find function
#include "recognition_constants.hpp"
#include <object_recognition/Recognized_objects.h> //msg for recognized object

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

struct RGBHist
{
  Mat bHist;
  Mat gHist;
  Mat rHist;
};

struct HSVHist
{
  Mat hHist;
  Mat sHist;
  Mat vHist;
};

struct Features
{
  vector<KeyPoint> keypoints;
  Mat descriptors;
};

struct IdImg
{
  int obj;
  std::string name;
  RGBHist rgb;
  HSVHist hsv;
  Features feat;
  double score_rgb;
  double score_hsv;
  double score_feat;
  double score_total;
  int nrOfFesturesFound;
  int nrOfFesturesTotal;

  bool operator<(const IdImg& a) const
  {
    return a.score_total < score_total;
  }

  friend std::ostream & operator<<(std::ostream & stream, const IdImg &a)
  {
    stream 	<< "Name: "<<a.name<<"\nTotal score: "<<a.score_total
            <<"\nRGB score: "<<a.score_rgb<<"\nHSV score: "<<a.score_hsv<<"\nFeat score: "<<a.score_feat
            <<"\nFound features: "<<a.nrOfFesturesFound<<"\nTotal features: "<<a.nrOfFesturesTotal;
    return stream;
  }
};


static const std::string dirName = "/home/robo/DD2425_2013/fuerte_workspace/robotics_g7/object_recognition/src/trainimages";
const std::string datafile = "/home/robo/DD2425_2013/fuerte_workspace/robotics_g7/object_recognition/src/traindata.txt";

static const std::string objects[] = {"pepper", "lemon", "pear", "carrot", "giraff", "tiger", "hippo"};

vector<IdImg> trainImg;
int keypThreshold = 30;
vector<int> identified;
vector<double> obj_scores;

bool FLAG_SHOW_OUTPUT = false;

/*
 * Draw RGB histogram
 */
void drawHistograms(RGBHist hist)
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
 * Make a HSV color histogram for the image
 */
HSVHist colorDetectionHSV(Mat img)
{
  Mat hsvImg;
  cvtColor( img, hsvImg, CV_BGR2HSV );

  //Split image into planes (HSV)
  vector<Mat> planes;
  split( hsvImg, planes );

  // Set bins for hue and saturation
  int histSize[] = { 180, 256, 256 };
  //int histSize = 256;

  // Set hue and saturation ranges
  float hRanges[] = { 0, 180 };
  float sRanges[] = { 0, 256 };
  float vRanges[] = { 0, 256 };
  const float* histRange[] = { hRanges, sRanges, vRanges };

  //Calculate HSV histogram and normalize
  Mat hHist, sHist, vHist;
  calcHist( &planes[0], 1, 0, Mat(), hHist, 1, &histSize[0], &histRange[0], true, false );	//hue
  calcHist( &planes[1], 1, 0, Mat(), sHist, 1, &histSize[1], &histRange[1], true, false );	//saturation
  calcHist( &planes[2], 1, 0, Mat(), vHist, 1, &histSize[2], &histRange[2], true, false );	//value


  //In the RGB case we needed to set the "black-color" bins to zero,
  //in HSV repr. darkness is located in vHist in the first bin.
  //hHist.at<float>(0) = 0;
  //sHist.at<float>(0) = 0;

  vHist.at<float>(0) = 0;

  normalize( hHist, hHist, 0, 1, NORM_MINMAX, -1, Mat() );
  normalize( sHist, sHist, 0, 1, NORM_MINMAX, -1, Mat() );
  normalize( vHist, vHist, 0, 1, NORM_MINMAX, -1, Mat() );

  HSVHist hist;
  hist.hHist = hHist;
  hist.sHist = sHist;
  hist.vHist = vHist;
  return hist;
}

/*
 * Make a RGB color histogram for the image
 */
RGBHist colorDetectionRGB(Mat img)
{
  //Split image into planes (B, G, R)
  vector<Mat> planes;
  split( img, planes );

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

  for(int i=0; i<1; ++i)
    {
      bHist.at<float>(i) = 0;
      gHist.at<float>(i) = 0;
      rHist.at<float>(i) = 0;
    }

  // Normalize the result
  int histH = 400;
  normalize(bHist, bHist, 0, histH, NORM_MINMAX, -1, Mat() );
  normalize(gHist, gHist, 0, histH, NORM_MINMAX, -1, Mat() );
  normalize(rHist, rHist, 0, histH, NORM_MINMAX, -1, Mat() );

  RGBHist hist = {bHist, gHist, rHist};
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
 * Try to identify the object by comparing color histogram, hue and feature detection to train images
 */
std::vector<IdImg> identifyObject(RGBHist& rgb, HSVHist& hsv, Features& feat)
{
  std::vector<IdImg> matched_objects(trainImg.size()); //vector that will hold information from all tested objects

  for(unsigned int i=0; i < trainImg.size(); ++i)
    {
      //RGB
      double resB = compareHist(rgb.bHist, trainImg[i].rgb.bHist, CV_COMP_CORREL);
      double resG = compareHist(rgb.gHist, trainImg[i].rgb.gHist, CV_COMP_CORREL);
      double resR = compareHist(rgb.rHist, trainImg[i].rgb.rHist, CV_COMP_CORREL);
      //HSV
      double resH = compareHist(hsv.hHist, trainImg[i].hsv.hHist, CV_COMP_CORREL);
      double resS = compareHist(hsv.sHist, trainImg[i].hsv.sHist, CV_COMP_CORREL);
      double resV = compareHist(hsv.vHist, trainImg[i].hsv.vHist, CV_COMP_CORREL);

      /////////////////////////////////////////////////////////
      matched_objects[i].obj = trainImg[i].obj;
      matched_objects[i].name = trainImg[i].name;

      //nomalize the score to be between [0,1] (divide the length of the vector
      //with the max lenght of the vector.
      if(resB > 0 && resG > 0 && resR > 0)
        matched_objects[i].score_rgb = sqrt( resB*resB + resG*resG + resR*resR)/sqrt(3);
      else
        matched_objects[i].score_rgb = 0;
      if (resH > 0 && resS > 0 && resV > 0)
        matched_objects[i].score_hsv = sqrt( resH*resH + resS*resS + resV*resV)/sqrt(3);
      else
        matched_objects[i].score_hsv = 0;
    }

  // Match descriptors
  for(unsigned int i=0; i < trainImg.size(); ++i)
    {
      // Use BruteForceMatcher
      BFMatcher matcher(cv::NORM_L1, true);
      vector<cv::DMatch> matches;
      matcher.match(feat.descriptors, trainImg[i].feat.descriptors, matches);

      matched_objects[i].nrOfFesturesFound = matches.size();
      matched_objects[i].nrOfFesturesTotal = trainImg[i].feat.keypoints.size();

      // Check percent of keypoints that match
      float res = (float)matches.size() / trainImg[i].feat.keypoints.size();

      ///////////////////////////////////////////////////////////
      matched_objects[i].score_feat = (double)res;
      matched_objects[i].score_total = matched_objects[i].score_rgb * matched_objects[i].score_hsv * matched_objects[i].score_feat;
    }

  std::sort(matched_objects.begin(), matched_objects.end());

  //take out the max_size best unique matches. If max_size is -1 (or a number that best_matches.size() never can be,
  //then create a rank list of all objects
  int max_size = -1;
  std::vector<int> saved_id;
  std::vector<IdImg> best_matches;

  for (unsigned int i=0; i < matched_objects.size(); i++)
    {
      if (best_matches.size() == max_size) break;

      else
        {
          //if the item does not exist in the vector
          if (std::find(saved_id.begin(), saved_id.end(), matched_objects[i].obj)==saved_id.end())
            {
              //std::cout<<"New id: "<<matched_objects[i].obj<<std::endl;
              saved_id.push_back(matched_objects[i].obj);
              best_matches.push_back(matched_objects[i]);

            }
          else
            continue;
        }
    }

  return best_matches;
}


/*
 * Train on some images
 */
void train()
{
  ROS_INFO("Training started...");
  // Get all folders for train images
  vector<std::string> imgDir;
  class dirent *ent;
  DIR *dir = opendir(dirName.c_str());
  while ((ent = readdir(dir)) != NULL)
    {
      const std::string imgDirName = ent->d_name;
      std::size_t isTest = imgDirName.find("test");
      //get image directories
      if (imgDirName[0] != '.' && isTest == std::string::npos)
        {
          //std::cout << imgDirName << std::endl;
          imgDir.push_back(imgDirName);
        }
    }
  closedir(dir);

  // Calculate color histogram and feature detection on train images
  for(unsigned int i=0; i < imgDir.size(); ++i)
    {

      // Get train images for each object
      const std::string imgPath = dirName + "/" + imgDir[i];
      dir = opendir(imgPath.c_str());

      while ((ent = readdir(dir)) != NULL)
        {
          const std::string filename = ent->d_name;

          // get only image files (.jpg)
          std::size_t isImg = filename.find(".jpg");
          if (isImg != std::string::npos)
            {
              const std::string fullFileName = imgPath + "/" + filename;
              //std::cout << "fullFileName: "<< fullFileName << std::endl;
              Mat img = imread(fullFileName);
              IdImg dummy;

              std::stringstream ss;
              ss << filename[0] << filename[1]; //ss will contain the first part of the filename
              int n;
              std::istringstream(ss.str()) >> n;

              dummy.obj = n;
              dummy.name = imgDir[i];
              dummy.rgb = colorDetectionRGB(img);
              dummy.hsv = colorDetectionHSV(img);
              dummy.feat = featureDetector(img);
              //std::cout<<"obj: "<<dummy.obj<<std::endl;
              std::cout<<"Trained "<<dummy.name<<std::endl;
              trainImg.push_back(dummy);
            }
        }
      closedir(dir);
    }
}


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber filter_sub_;
  ros::Publisher obj_pub_;


public:
  IplImage* img;
  IplImage* img_depth;
  IplImage* hsv_image;
  IplImage* hsv_mask;

  ImageConverter()
    : it_(nh_)
  {

    filter_sub_ = it_.subscribe("/color_filter/filtered_image", 1, &ImageConverter::imageCb, this);
    obj_pub_ = nh_.advertise<object_recognition::Recognized_objects>("/recognition/recognized", 1);
  }

  ~ImageConverter()
  {
    cv::destroyAllWindows();
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
    Mat src;
    Mat(img).copyTo(src);

    // Detect objects through feature detection
    int featSize = featureDetector(src).keypoints.size();

    if (FLAG_SHOW_OUTPUT)
      {
        //std::cout << "Nr keypoints: " << featSize << std::endl;
      }

    int type; //{"pepper", "lemon", "pear", "carrot", "giraff", "tiger", "hippo"};

    // Try to recognize object if enough keys are detected
    if(featSize > keypThreshold)
      {
        if (FLAG_SHOW_OUTPUT)
          std::cout << "Sees an object with nr of feature:\n"<<featSize << std::endl;

        // Color detection
        RGBHist rgb = colorDetectionRGB(src);
        //drawHistograms(hist);

        HSVHist hsv = colorDetectionHSV(src);

        //Feature detection
        Features feat = featureDetector(src);
        //showKeypoints(src, feat);

        // Identify object(s)
        std::vector<IdImg> objects = identifyObject(rgb, hsv, feat);

        type = objects[0].obj;

        if (FLAG_SHOW_OUTPUT)
          {
            int nr_to_show = 3;
            for (unsigned int i = 0; i < objects.size() && i < nr_to_show; ++i)
              //for (unsigned int i = 0; i < objects.size(); ++i)
              {
                std::cout<<i+1<<": "<<objects[i]<<std::endl;
                std::cout<<"---------------------------"<<std::endl;

              }
            std::cout<<"\n";
          }

        std::string name  = objects[0].name;
        //std::cout << "Recognized object: " << name << std::endl;

        //Publish message when object is detectected
        //image_pub_.publish(obj);
      }
    else //no object found
      {
        if (FLAG_SHOW_OUTPUT)
          {
            std::cout << "no object\n" << std::endl;
          }
        type = (int)objRecognition::OBJTYPE_NO_OBJECT;
      }


    //Create ros-message
    object_recognition::Recognized_objects obj_msg;
    obj_msg.obj_type.push_back(type);
    obj_pub_.publish(obj_msg);

    cv::imshow(WINDOW, src);
    cv::waitKey(3);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_recognition");
  ros::NodeHandle n;

  initModule_nonfree();	// For using SURF

  // Train on test images
  ROS_INFO("[Object features] Start training.");
  train();
  ROS_INFO("Training complete!");
  FLAG_SHOW_OUTPUT = true;


  // Train mode
  if(argc > 1)
    {

      /*train();
      Mat src = imread( argv[1] );
      RGBHist hist = colorDetectionRGB(src);
      drawHistograms(hist);
      Features feat = featureDetector(src);
      showKeypoints(src, feat);
      //std::vector<IdImg> obj = identifyObject(hist, feat);
      waitKey(0);*/
    }
  else
    {
      // Get train data
      /*std::string line;
      std::ifstream myfile(datafile.c_str());
      if (myfile.is_open())
      {
        while( getline(myfile,line)) {
        	//std::cout << line << std::endl;
        	std::vector<double> v;
        	std::vector<string> strs;
      		boost::split(strs, line, boost::is_any_of("[;"));

      		vector<string>::iterator i ;
       		for(i = strs.begin() ; i != strs.end(); i++) {
       			std::string s = *i;
          	std::cout << s;
          	double lol = atof(s.c_str());
          	v.push_back(lol);
       		}
          //Mat hist(v.size(), 1, DataType<float>::type);
         	Mat hist( v, true );
      	  std::cout << hist << " " << v.size() << std::endl;*/

      /*std::string buf;
      std::stringstream ss(line.c_str());
      std::vector<string> tokens;
      while (ss >> buf) {
      	tokens.push_back(buf);
      }
      std::cout << tokens[0] << " " << tokens[1] << std::endl;*/

      /*IdImg dummy;
      dummy.obj = n;
      dummy.name = imgDir[i];
      dummy.rgb = colorDetectionRGB(img);
      dummy.feat = featureDetector(img);
      trainImg.push_back(dummy);*/
      /*}
      myfile.close();
      }
      else std::cout << "Unable to open file"; */

      ImageConverter ic;
      ROS_INFO("object recognition is up and running!");
      ROS_INFO("Messages are being sent to /recognition/recognized");
      ros::spin();
    }
  return 0;
}

