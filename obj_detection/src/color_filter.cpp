#include "headers/color_filter.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//for the keyboard input (will remove later and only use the Trackbar)
#include <sstream>      //std::stringstream
#include <stdio.h>      //getchar()
#include <termios.h>    //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>     //STDIN_FILENO

// Debug
#include <typeinfo>

namespace objRecognition
{
// Fine-tuning settings
struct Cf_Params params_walls, params_floor;
// Debug setting
Filter_mode mode;

///**
// * @brief Color_Filter::ChangeThreshold
// * @param msg
// */
//void Color_Filter::ChangeThreshold(const std_msgs::String::ConstPtr &msg)
//{
//  if ( tolower(msg->data.c_str()[1]) == 'z' )
//    {
//      if (image_sub_ == 0) //not subscribing
//        {
//          image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Color_Filter::color_filter, this);
//          puts("RESUMED TO SUBSCRIBE TO TOPIC /camera/rgb/image_color");
//        }
//      else //don't subscribe to the camera and just be idle
//        {
//          image_sub_.shutdown();
//          puts("STOPPED TO SUBSCRIBE TO TOPIC /camera/rgb/image_color");
//        }
//      return;
//    }
//}

/**
 * @brief on_trackbar
 *  Local function to react on slider movements
 */
void on_trackbar( int, void* ) //for later use
{
}

void Color_Filter::showConfigurationPanel()
{
  //FLAG_SHOW_IMAGE = true;
  FLAG_CHANGE_THRESHOLD = true;
  cv::namedWindow("Trackbar walls", 1);
  cv::namedWindow("Trackbar floor", 1);
  cv::createTrackbar( "Walls HUE MIN:", "Trackbar walls", &params_walls.H_MIN, 180, on_trackbar);
  cv::createTrackbar( "Walls HUE MAX:", "Trackbar walls", &params_walls.H_MAX, 180, on_trackbar);
  cv::createTrackbar( "Walls SAT MIN:", "Trackbar walls", &params_walls.S_MIN, 255, on_trackbar);
  cv::createTrackbar( "Walls SAT MAX:", "Trackbar walls", &params_walls.S_MAX, 255, on_trackbar);
  cv::createTrackbar( "Walls VAL MIN:", "Trackbar walls", &params_walls.V_MIN, 255, on_trackbar);
  cv::createTrackbar( "Walls VAL MAX:", "Trackbar walls", &params_walls.V_MAX, 255, on_trackbar);

  cv::createTrackbar( "Floor HUE MIN:", "Trackbar floor", &params_floor.H_MIN, 180, on_trackbar);
  cv::createTrackbar( "Floor HUE MAX:", "Trackbar floor", &params_floor.H_MAX, 180, on_trackbar);
  cv::createTrackbar( "Floor SAT MIN:", "Trackbar floor", &params_floor.S_MIN, 255, on_trackbar);
  cv::createTrackbar( "Floor SAT MAX:", "Trackbar floor", &params_floor.S_MAX, 255, on_trackbar);
  cv::createTrackbar( "Floor VAL MIN:", "Trackbar floor", &params_floor.V_MIN, 255, on_trackbar);
  cv::createTrackbar( "Floor VAL MAX:", "Trackbar floor", &params_floor.V_MAX, 255, on_trackbar);

  /// Show some stuff
  on_trackbar( params_walls.H_MIN, 0 );
  on_trackbar( params_walls.H_MAX, 0 );
  on_trackbar( params_walls.S_MIN, 0 );
  on_trackbar( params_walls.S_MAX, 0 );
  on_trackbar( params_walls.V_MIN, 0 );
  on_trackbar( params_walls.V_MAX, 0 );

  on_trackbar( params_floor.H_MIN, 0 );
  on_trackbar( params_floor.H_MAX, 0 );
  on_trackbar( params_floor.S_MIN, 0 );
  on_trackbar( params_floor.S_MAX, 0 );
  on_trackbar( params_floor.V_MIN, 0 );
  on_trackbar( params_floor.V_MAX, 0 );
}

/**
 * @brief Color_Filter::color_filter
 * @param msg
 */
void Color_Filter::color_filter(const sensor_msgs::ImageConstPtr &msg, bool publishFilteredImg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  cv::Mat src_image, hsv_image, im_walls, im_floor, remove_walls, remove_floor, remove_background,filter_image;
  src_image = cv_ptr->image; //get the source image from the pointer

  //remove top part of image (set to black)
  int y_remove = 100;
  for(int i=0; i<y_remove; i++)
    {
      for(int j=0; j<src_image.cols; j++)
        {
          src_image.at<cv::Vec3b>(i,j)[0] = 0;
          src_image.at<cv::Vec3b>(i,j)[1] = 0;
          src_image.at<cv::Vec3b>(i,j)[2] = 0;
        }
    }

  cv::cvtColor(src_image, hsv_image,CV_BGR2HSV);
  cv::inRange(hsv_image,cv::Scalar(params_walls.H_MIN, params_walls.S_MIN, params_walls.V_MIN),
              cv::Scalar(params_walls.H_MAX, params_walls.S_MAX, params_walls.V_MAX), im_walls);
  cv::inRange(hsv_image,cv::Scalar(params_floor.H_MIN,params_floor.S_MIN,params_floor.V_MIN),cv::Scalar(params_floor.H_MAX,params_floor.S_MAX,params_floor.V_MAX),im_floor);
  bitwise_not(im_walls, remove_walls); //invert the pixels so that the walls are removed
  bitwise_not(im_floor, remove_floor); //invert the pixels so that the floor is removed
  bitwise_and(remove_walls, remove_floor, remove_background); //Grayscale image with the background removed
  cvtColor(remove_background, remove_background, CV_GRAY2BGR); //change image to a BGR image
  bitwise_and(src_image, remove_background, filter_image); //Remove the background





  //Does some erosion and dilation to remove some of the pixels
  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::erode(filter_image, filter_image, cv::Mat());
  cv::dilate(filter_image, filter_image, element);



  //CONTOURS DETECTION!
  ///*
  //http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/bounding_rects_circles/bounding_rects_circles.html

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  /// Detect edges using Threshold
  cv::Mat filter_image_bin;
  threshold( filter_image, filter_image_bin, 0, 255, 0 ); //0 == THRESH_BINARY
  cvtColor(filter_image_bin, filter_image_bin, CV_RGB2GRAY); //one channel

  /// Find contours
  findContours( filter_image_bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  //clear the object vector, and creates a temporary vector storing all detectable things
  objects.clear();
  objects.reserve(contours.size());
  std::vector<DetectedObject> temp_objects;
  temp_objects.reserve(contours.size());
  DetectedObject newObj;

  std::vector<cv::Point> cur_contours_poly;
  for(unsigned int i = 0; i < contours.size(); i++ )
    {
      approxPolyDP( cv::Mat(contours[i]), cur_contours_poly, 3, true );
      double area = contourArea(cur_contours_poly);
      if (area >= minArea && area <= maxArea) //only store objects with "lagom" area
        {
          newObj.contours_poly = cur_contours_poly;
          temp_objects.push_back(newObj);
        }
    }


  //remove objects that are to narrow to be a real object
  for(std::vector<DetectedObject>::iterator it = temp_objects.begin(); it != temp_objects.end(); ++it)
    {

      it->rotatedRect = minAreaRect( cv::Mat(it->contours_poly) );

      double ratio = (double)std::max( it->rotatedRect.size.height , it->rotatedRect.size.width ) /
                     (double)std::min( it->rotatedRect.size.height , it->rotatedRect.size.width );

      if (ratio > minRatio) //if it's a too narrow rectangle, ignore it
        continue;

      it->boundRect = boundingRect( cv::Mat(it->contours_poly) );

      /// Get the mass centers using the moment of each contour
      cv::Moments mu = moments( it->contours_poly, false );

      it->mc = cv::Point( (int)mu.m10/mu.m00 , (int)mu.m01/mu.m00 );
      objects.push_back(*it); //copy the object and put it in the vector called objects

    }

  //crop out the objects from the image


  cv::Mat crop_img = cv::Mat::zeros( src_image.size(), src_image.type());
  cv::Mat mask = cv::Mat::zeros( src_image.size(), src_image.type());
  cv::Scalar color_white = cv::Scalar( 255, 255, 255);
  for(std::vector<DetectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
  {
	  cv::Rect ROI(it->boundRect.x, it->boundRect.y, it->boundRect.width , it->boundRect.height);
	  rectangle(mask, ROI.tl(), ROI.br(), color_white, CV_FILLED); //mask

	  // Cut out ROI and store it in crop_img
	  src_image.copyTo(crop_img, mask);
  }
  //for some reason the source image get overwritten. Retrieve it back!
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  src_image = cv_ptr->image.clone();



  // Publish filtered image (if advised to do)
  if (publishFilteredImg) {
    cv_bridge::CvImagePtr img_ptr = cv_ptr;
    //filter_image.copyTo(img_ptr->image);
    crop_img.copyTo(img_ptr->image);
    img_pub_.publish(img_ptr->toImageMsg());
  }

  //track objects

  //if there where no previous rectangles, add some!
  unsigned int prev_rects_size = prev_rects.size();
  if (prev_rects_size == 0)
    {
      for(std::vector<DetectedObject>::iterator it = temp_objects.begin(); it != temp_objects.end(); ++it)
        {
          ObjectRectangle newRect;
          it->ROI_id = ROI_id_counter;
          newRect.ROI_id = ROI_id_counter;
          newRect.boundRect = it->boundRect;
          ++ROI_id_counter;
          prev_rects.push_back(newRect);
        }
    }

  //Create ros-message
//  if (publishFilteredImg)
//    {
//      color_filter::Objects obj_msg;
//      obj_msg.ROI.reserve( contours_poly.size() );
//      for (unsigned int i=0; i < boundRect.size(); i++)
//        {
//          cv::Rect rect = boundRect[i];
//          color_filter::Rect2D_ rect2D_msg;
//          rect2D_msg.x = rect.x;
//          rect2D_msg.y = rect.y;
//          rect2D_msg.height = rect.height;
//          rect2D_msg.width = rect.width;
//          obj_msg.ROI.push_back(rect2D_msg);
//          obj_msg.ROI_id = ROI_id_counter;
//        }
//      obj_pub_.publish(obj_msg);
//    }

  /*
                else
                {
                        for(std::vector<ObjectRectangle>::iterator it = prev_rects.begin(); it != prev_rects.end(); ++it)
                        {


                        }
                }
                */

  //		if (boundRect.size() == 1)
  //		{
  //			//std::cout<<"boundRect[0]: ("<<boundRect[0].tl()<<", "<<boundRect[0].br()<<")"<<std::endl;
  //			//std::cout<<"prev_rect: ("<<prev_rect.tl()<<", "<<prev_rect.br()<<")"<<std::endl;
  //			//std::cout<<"(boundRect[0] & prev_rect): "<<(boundRect[0] & prev_rect).tl()<<", "<<(boundRect[0] & prev_rect).br()<<")"<<std::endl;
  //			if ((boundRect[0] & prev_rect).height == 0) //if its not overlapping with the previous rectangle, its a new object
  //			{
  //				ROI_id_counter++;
  //				prev_rect = boundRect[0];
  //				flag_send_msg = true;
  //			}
  //			else
  //			{
  //				prev_rect = boundRect[0];
  //			}
  //		}
  //		else
  //			prev_rect = cv::Rect();




  //		//could not get the damned openCV-merging-rectangles-function to work properly.
  //		//Had to create my own way of merging intersecting rectangles!
  //		bool MERGE_DONE = false;
  //                unsigned int i = 0;
  //		while(!MERGE_DONE && boundRect.size()>1)
  //		{
  //			bool have_merged = false;
  //			for (unsigned int j = i+1; j<boundRect.size(); j++)
  //			{
  //                                cv::Rect &rect_a = boundRect[i];
  //
  //                                cv::Rect &rect_b = boundRect[j];
  //				cv::Rect intersect = rect_a & rect_b;
  //				if (intersect.height != 0 && intersect.width != 0) //rectangles intersect!
  //				{
  //					int x = std::min(rect_a.x , rect_b.x);
  //					int y = std::min(rect_a.y , rect_b.y);
  //					int width = std::max(rect_a.br().x , rect_b.br().x) - x;
  //					int height = std::max(rect_a.br().y , rect_b.br().y) - y;
  //					cv::Rect merged_rect(x,y,width, height);
  //
  //					boundRect[i] = merged_rect; //write over the old rectangle with the bigger one
  //					boundRect.erase(boundRect.begin() + j); //delete the rectangle that is already merged
  //
  //					have_merged = true;
  //					break;
  //				}
  //			}
  //
  //			//if we have merged two rectangles, start all over again in case there exist
  //			//rectangles that will intersect with the new bigger added rectangle.
  //			if (have_merged)
  //			{
  //				i = 0;
  //				continue;
  //			}
  //			i++;
  //			if (i == boundRect.size()) MERGE_DONE = true;
  //		}
  //
  //		bool flag_send_msg = false;
  //		if (boundRect.size() == 1)
  //		{
  //			//std::cout<<"boundRect[0]: ("<<boundRect[0].tl()<<", "<<boundRect[0].br()<<")"<<std::endl;
  //			//std::cout<<"prev_rect: ("<<prev_rect.tl()<<", "<<prev_rect.br()<<")"<<std::endl;
  //			//std::cout<<"(boundRect[0] & prev_rect): "<<(boundRect[0] & prev_rect).tl()<<", "<<(boundRect[0] & prev_rect).br()<<")"<<std::endl;
  //			if ((boundRect[0] & prev_rect).height == 0) //if its not overlapping with the previous rectangle, its a new object
  //			{
  //				ROI_id_counter++;
  //				prev_rect = boundRect[0];
  //				flag_send_msg = true;
  //			}
  //			else
  //			{
  //				prev_rect = boundRect[0];
  //			}
  //		}
  //		else
  //			prev_rect = cv::Rect();
  //
  //
  //

  if (FLAG_SHOW_IMAGE)
    {
      cv::imshow("Original", src_image);


      cv::Mat drawing = cv::Mat::zeros( filter_image_bin.size(), CV_8UC3 );
      cv::Scalar color_red = cv::Scalar( 0, 0, 255 );
      cv::Scalar color_blue = cv::Scalar( 255, 0, 0);
      cv::Scalar color_white = cv::Scalar( 255, 255, 255);


      cv::Point2f rect_points[4];
      for(std::vector<DetectedObject>::iterator it = objects.begin(); it != objects.end(); ++it)
        {
          //draw the mass centers
          line(drawing, it->mc, it->mc, color_white, 10, 8, 0);

          rectangle( drawing, it->boundRect.tl(), it->boundRect.br(), color_red, 2, 8, 0 );

          //draw rotatedRect
          it->rotatedRect.points( rect_points );
          for( int j = 0; j < 4; j++ )
            line( drawing, rect_points[j], rect_points[(j+1)%4], color_blue, 1, 8 );

          //http://opencv-users.1802565.n2.nabble.com/draw-only-one-contour-using-drawContours-td7404400.html
          std::vector<std::vector<cv::Point> > T = std::vector<std::vector<cv::Point> >(1,it->contours_poly);
          for (unsigned int i = 0; i < T.size(); i++)
            drawContours( drawing, T,i, color_red, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );

        }

      imshow( "Found objects", drawing );

      cv::waitKey(3);
    }

  if (FLAG_CHANGE_THRESHOLD)
    {
      cv::imshow("Original", src_image);
      cv::imshow("HSV", hsv_image);
      cv::imshow("Only the walls", im_walls);
      cv::imshow("Only the floor", im_floor);
      cv::imshow("Result", filter_image);
      cv::waitKey(3);
    }
  //*/


  //BLOB DETECTION!
  /*

                //http://stackoverflow.com/questions/8076889/tutorial-on-opencv-simpleblobdetector
                // set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
                cv::SimpleBlobDetector::Params params;
                params.minDistBetweenBlobs = 20.0f;
                params.filterByInertia = false;
                params.filterByConvexity = false;
                params.filterByCircularity = false;
                params.filterByColor = false;
                params.filterByArea = true;
                params.minArea = 100.0f;
                params.maxArea = 1000000.0f;

                // ... any other params you don't want default value


                // set up and create the detector using the parameters
                cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
                blob_detector->create("SimpleBlob");

                // detect!


                cv::erode(remove_background, remove_background, cv::Mat());
                cv::dilate(filter_image, filter_image, element);
                //cv::imshow("remove_background", remove_background);


                cv::Mat imageROI;
                int y_remove = 100;
                imageROI= remove_background(cv::Rect(0,y_remove,remove_background.cols,remove_background.rows - y_remove));
                std::vector<cv::KeyPoint> keypoints;
                blob_detector->detect(imageROI, keypoints);


                // extract the x y coordinates of the keypoints:
                if (keypoints.size()>0)
                        std::cout<<"\nFound "<<keypoints.size()<<" keypoints"<<std::endl;
                for (int i=0; i<keypoints.size(); i++){
                        keypoints[i].pt.y += y_remove;
                    float X=keypoints[i].pt.x;
                    float Y=keypoints[i].pt.y;
                    std::cout<<"X: "<<X<<std::endl;
                    std::cout<<"Y: "<<Y<<std::endl<<std::endl;
                }l

                cv::Mat found_objects;
                drawKeypoints(remove_background, keypoints,found_objects, cv::Scalar(0,0,255));
                cv::imshow("found_objects", found_objects);
  */
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
} // END namespace objRecognition

//int main(int argc, char** argv)
//{
//	ros::init(argc, argv, "color_filter");
//	if (argc != 1)
//	{
//		if (atoi(argv[1]) == 1)
//                {
//			FLAG_SHOW_IMAGE = true;
//		}
//		else if (atoi(argv[1]) >= 2)
//		{
//			//FLAG_SHOW_IMAGE = true;
//			FLAG_CHANGE_THRESHOLD = true;
//			cv::namedWindow("Trackbar walls", 1);
//			cv::namedWindow("Trackbar floor", 1);
//			cv::createTrackbar( "Walls HUE MIN:", "Trackbar walls", &params_walls.H_MIN, 180, on_trackbar);
//			cv::createTrackbar( "Walls HUE MAX:", "Trackbar walls", &params_walls.H_MAX, 180, on_trackbar);
//			cv::createTrackbar( "Walls SAT MIN:", "Trackbar walls", &params_walls.S_MIN, 255, on_trackbar);
//			cv::createTrackbar( "Walls SAT MAX:", "Trackbar walls", &params_walls.S_MAX, 255, on_trackbar);
//			cv::createTrackbar( "Walls VAL MIN:", "Trackbar walls", &params_walls.V_MIN, 255, on_trackbar);
//			cv::createTrackbar( "Walls VAL MAX:", "Trackbar walls", &params_walls.V_MAX, 255, on_trackbar);

//			cv::createTrackbar( "Floor HUE MIN:", "Trackbar floor", &params_floor.H_MIN, 180, on_trackbar);
//			cv::createTrackbar( "Floor HUE MAX:", "Trackbar floor", &params_floor.H_MAX, 180, on_trackbar);
//			cv::createTrackbar( "Floor SAT MIN:", "Trackbar floor", &params_floor.S_MIN, 255, on_trackbar);
//			cv::createTrackbar( "Floor SAT MAX:", "Trackbar floor", &params_floor.S_MAX, 255, on_trackbar);
//			cv::createTrackbar( "Floor VAL MIN:", "Trackbar floor", &params_floor.V_MIN, 255, on_trackbar);
//			cv::createTrackbar( "Floor VAL MAX:", "Trackbar floor", &params_floor.V_MAX, 255, on_trackbar);

//			/// Show some stuff
//			on_trackbar( params_walls.H_MIN, 0 );
//			on_trackbar( params_walls.H_MAX, 0 );
//			on_trackbar( params_walls.S_MIN, 0 );
//			on_trackbar( params_walls.S_MAX, 0 );
//			on_trackbar( params_walls.V_MIN, 0 );
//			on_trackbar( params_walls.V_MAX, 0 );

//			on_trackbar( params_floor.H_MIN, 0 );
//			on_trackbar( params_floor.H_MAX, 0 );
//			on_trackbar( params_floor.S_MIN, 0 );
//			on_trackbar( params_floor.S_MAX, 0 );
//			on_trackbar( params_floor.V_MIN, 0 );
//			on_trackbar( params_floor.V_MAX, 0 );
//		}
//	}


//	//cv::namedWindow("Linear Blend", 1);
//	//cv::createTrackbar( " Threshold:", "Linear Blend", &params_floor.S_MAX, 255, on_trackbar);
//	/// Show some stuff
//	//on_trackbar( params_floor.S_MAX, 0 );



//        objRecognition::Color_Filter cf;
//	puts("Color filter is up and running!");
//	puts("Message is being sent to /recognition/detect");
//	ros::spin();

//	return 0;
//}

