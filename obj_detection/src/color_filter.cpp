#include "headers/color_filter.hpp"

namespace objRecognition
{

class Color_Filter
{
private:
  // ROS communication & image acquisition
  ros::NodeHandle& nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber keyboard_sub;
  ros::Publisher obj_pub_;
  image_transport::Publisher img_pub_;
  // previous ROI
  cv::Rect prev_rect;
  int ROI_id_counter;

public:
  Color_Filter(ros::NodeHandle& nh): nh_(nh), it_(nh), ROI_id_counter(0)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Color_Filter::color_filter, this);
    keyboard_sub = nh_.subscribe("/keyboard/input", 1, &Color_Filter::keyboardChangeThreshold, this);
    img_pub_ = it_.advertise("/color_filter/filtered_image", 1);
    // TODO: save directly in vector so master node can access
    obj_pub_ = nh_.advertise<color_filter::Objects>("/recognition/detect", 1);
  }

  ~Color_Filter()
  {
    cv::destroyAllWindows();
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void keyboardChangeThreshold(const std_msgs::String::ConstPtr& msg)
  {
    if ( tolower(msg->data.c_str()[1]) == 'z' )
      {
        if (image_sub_ == 0) //not subscribing
          {
            image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &Color_Filter::color_filter, this);
            puts("RESUMED TO SUBSCRIBE TO TOPIC /camera/rgb/image_color");
          }
        else //don't subscribe to the camera and just be idle
          {
            image_sub_.shutdown();
            puts("STOPPED TO SUBSCRIBE TO TOPIC /camera/rgb/image_color");
          }
        return;
      }


    puts("---------------------");
    if (msg->data.c_str()[0] == '1')
      {
        puts("Wall filter parameters");
        mode = WALLS;
        H_MIN = wall_H_MIN;
        H_MAX = wall_H_MAX;
        S_MIN = wall_S_MIN;
        S_MAX = wall_S_MAX;
        V_MIN = wall_V_MIN;
        V_MAX = wall_V_MAX;
      }
    else if (msg->data.c_str()[0] == '2')
      {
        puts("Floor filter parameters");
        mode = FLOOR;
        H_MIN = floor_H_MIN;
        H_MAX = floor_H_MAX;
        S_MIN = floor_S_MIN;
        S_MAX = floor_S_MAX;
        V_MIN = floor_V_MIN;
        V_MAX = floor_V_MAX;
      }


    //Hue-part
    if ( tolower(msg->data.c_str()[1]) == 'a')
      {
        H_MIN -= CHANGE;
        if (H_MIN < 0)
          H_MIN = 0;
      }
    else if ( tolower(msg->data.c_str()[1]) == 'q')
      {
        H_MIN += CHANGE;
        if (H_MIN > H_MAX)
          H_MIN = H_MAX;
      }
    else if (tolower(msg->data.c_str()[1]) == 's')
      {
        H_MAX -= CHANGE;
        if (H_MAX < H_MIN)
          H_MAX = H_MIN;
      }
    else if (tolower(msg->data.c_str()[1]) == 'w')
      {
        H_MAX += CHANGE;
        if (H_MAX > MAX)
          H_MAX = MAX;
      }

    //Saturate-part
    else if (tolower(msg->data.c_str()[1]) == 'd')
      {
        S_MIN -= CHANGE;
        if (S_MIN < 0)
          S_MIN = 0;
      }
    else if (tolower(msg->data.c_str()[1]) == 'e')
      {
        S_MIN += CHANGE;
        if (S_MIN > S_MAX)
          S_MIN = S_MAX;
      }
    else if (tolower(msg->data.c_str()[1]) == 'f')
      {
        S_MAX -= CHANGE;
        if (S_MAX < S_MIN)
          S_MAX = S_MIN;
      }
    else if (tolower(msg->data.c_str()[1]) == 'r')
      {
        S_MAX += CHANGE;
        if (S_MAX > MAX)
          S_MAX = MAX;
      }

    //Value-part
    else if (tolower(msg->data.c_str()[1]) == 'g')
      {
        V_MIN -= CHANGE;
        if (V_MIN < 0)
          V_MIN = 0;
      }
    else if (tolower(msg->data.c_str()[1]) == 't')
      {
        V_MIN += CHANGE;
        if (V_MIN > V_MAX)
          V_MIN = V_MAX;
      }
    else if (tolower(msg->data.c_str()[1]) == 'h')
      {
        V_MAX -= CHANGE;
        if (V_MAX < V_MIN)
          V_MAX = V_MIN;
      }
    else if (tolower(msg->data.c_str()[1]) == 'y')
      {
        V_MAX += CHANGE;
        if (V_MAX > MAX)
          V_MAX = MAX;
      }

    //update
    switch(mode)
      {
      case WALLS:
      {
        wall_H_MIN = H_MIN;
        wall_H_MAX = H_MAX;
        wall_S_MIN = S_MIN;
        wall_S_MAX = S_MAX;
        wall_V_MIN = V_MIN;
        wall_V_MAX = V_MAX;
        break;
      }
      case FLOOR:
      {
        floor_H_MIN = H_MIN;
        floor_H_MAX = H_MAX;
        floor_S_MIN = S_MIN;
        floor_S_MAX = S_MAX;
        floor_V_MIN = V_MIN;
        floor_V_MAX = V_MAX;
        break;
      }

      }

    std::cout<<"H_MIN: "<< H_MIN<<" H_MAX: "<<H_MAX<< " S_MIN: "<<S_MIN<<" S_MAX: "<<S_MAX<<" V_MIN: "<<V_MIN<<" V_MAX: "<<V_MAX<<std::endl;
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void color_filter(const sensor_msgs::ImageConstPtr& msg)
  {
    // Save computation power: 10 frames/s should be enough
    ros::Duration(0.1).sleep();

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
      for(int j=0; j<src_image.cols; j++)
        {
          src_image.at<cv::Vec3b>(i,j)[0] = 0;
          src_image.at<cv::Vec3b>(i,j)[1] = 0;
          src_image.at<cv::Vec3b>(i,j)[2] = 0;
        }


    cv::cvtColor(src_image, hsv_image,CV_BGR2HSV);
    cv::inRange(hsv_image,cv::Scalar(wall_H_MIN,wall_S_MIN,wall_V_MIN),cv::Scalar(wall_H_MAX,wall_S_MAX,wall_V_MAX),im_walls);
    cv::inRange(hsv_image,cv::Scalar(floor_H_MIN,floor_S_MIN,floor_V_MIN),cv::Scalar(floor_H_MAX,floor_S_MAX,floor_V_MAX),im_floor);
    bitwise_not(im_walls, remove_walls); //invert the pixels so that the walls are removed
    bitwise_not(im_floor, remove_floor); //invert the pixels so that the floor is removed
    bitwise_and(remove_walls, remove_floor, remove_background); //Grayscale image with the background removed
    cvtColor(remove_background, remove_background, CV_GRAY2BGR); //change image to a BGR image
    bitwise_and(src_image, remove_background, filter_image); //Remove the background

    // Publish filtered image (TODO: only 2 times per second, also remember what was sent)
    cv_bridge::CvImagePtr img_ptr = cv_ptr;
    filter_image.copyTo(img_ptr->image);
    img_pub_.publish(img_ptr->toImageMsg());

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

    std::vector< std::vector<cv::Point> > contours_poly;
    std::vector<cv::Point> cur_contours_poly;
    contours_poly.reserve(contours.size()); //reserve space
    for(unsigned int i = 0; i < contours.size(); i++ )
      {
        approxPolyDP( cv::Mat(contours[i]), cur_contours_poly, 3, true );
        double area = contourArea(cur_contours_poly);
        //std::cout<<"Area of contour nr "<<i<<": "<<area<<std::endl;
        if (area >= minArea && area<= maxArea) //save the closed approximated contour if it's area is "lagom"
          {
            //std::cout<<"\nContour is lagom! nr "<<i<<": "<<area<<std::endl;
            contours_poly.push_back(cur_contours_poly);
          }
      }



//		/// Get the moments
//		std::vector<cv::Moments> mu(contours_poly.size() );
//		for( unsigned int i = 0; i < contours_poly.size(); i++ )
//		{
//			mu[i] = moments( contours_poly[i], false );
//		}
//
//		///  Get the mass centers:
//		std::vector<cv::Point> mc( contours_poly.size() );
//		for( int i = 0; i < contours_poly.size(); i++ )
//		{
//			mc[i] = cv::Point( (int)mu[i].m10/mu[i].m00 , (int)mu[i].m01/mu[i].m00 );
//			int x = mc[i].x;
//			int y = mc[i].y;
//			std::cout<<"mc["<<i<<"].x "<<mc[i].x<<std::endl;
//			std::cout<<"mc["<<i<<"].y "<<mc[i].y<<std::endl;
//			cv::Vec3b pixel = hsv_image.at<cv::Vec3b>(y,x);
//	         int h = pixel[0];
//	         int s = pixel[1];
//	         int v = pixel[2];
//	         std::cout << "h:" << h<< " s:" << s << " v:" << v << std::endl;
//
//		}
//		std::cout<<""<<std::endl;


    /// Approximate contours to polygons + get bounding rectangles
    cv::Rect rect;
    std::vector<cv::Rect> boundRect;
    boundRect.reserve( contours_poly.size() );
    for(unsigned int i = 0; i < contours_poly.size(); i++ )
      {
        rect = boundingRect( cv::Mat(contours_poly[i]) );
        double ratio = (double)std::max(rect.height,rect.width)  / (double)std::min(rect.height,rect.width);
        //std::cout<<ratio<<std::endl;
        if (ratio > 4.3) //if it's a too narrow rectangle, ignore it
          {
            continue;
          }
        boundRect.push_back(rect);

      }

    //could not get the damned openCV-merging-rectangles-function to work properly.
    //Had to create my own way of merging intersecting rectangles!
    bool MERGE_DONE = false;
    unsigned int i = 0;
    while(!MERGE_DONE && boundRect.size()>1)
      {
        bool have_merged = false;
        for (unsigned int j = i+1; j<boundRect.size(); j++)
          {
            cv::Rect &rect_a = boundRect[i];

            cv::Rect &rect_b = boundRect[j];
            cv::Rect intersect = rect_a & rect_b;
            if (intersect.height != 0 && intersect.width != 0) //rectangles intersect!
              {
                int x = std::min(rect_a.x , rect_b.x);
                int y = std::min(rect_a.y , rect_b.y);
                int width = std::max(rect_a.br().x , rect_b.br().x) - x;
                int height = std::max(rect_a.br().y , rect_b.br().y) - y;
                cv::Rect merged_rect(x,y,width, height);

                boundRect[i] = merged_rect; //write over the old rectangle with the bigger one
                boundRect.erase(boundRect.begin() + j); //delete the rectangle that is already merged

                have_merged = true;
                break;
              }
          }

        //if we have merged two rectangles, start all over again in case there exist
        //rectangles that will intersect with the new bigger added rectangle.
        if (have_merged)
          {
            i = 0;
            continue;
          }
        i++;
        if (i == boundRect.size()) MERGE_DONE = true;
      }

    bool flag_send_msg = false;
    if (boundRect.size() == 1)
      {
        std::cout<<"boundRect[0]: ("<<boundRect[0].tl()<<", "<<boundRect[0].br()<<")"<<std::endl;
        std::cout<<"prev_rect: ("<<prev_rect.tl()<<", "<<prev_rect.br()<<")"<<std::endl;
        std::cout<<"(boundRect[0] & prev_rect): "<<(boundRect[0] & prev_rect).tl()<<", "<<(boundRect[0] & prev_rect).br()<<")"<<std::endl;
        if ((boundRect[0] & prev_rect).height == 0) //if its not overlapping with the previous rectangle, its a new object
          {
            ROI_id_counter++;
            prev_rect = boundRect[0];
            flag_send_msg = true;
          }
        else
          {
            prev_rect = boundRect[0];
          }
      }
    else
      prev_rect = cv::Rect();
    /*
     * TODO: replace here with struct rectObject and write to vector. This can be accessed directly
     * by the recognition master (without msg transfer)
     */
    //Create ros-message
    if (flag_send_msg)
      {
        color_filter::Objects obj_msg;
        obj_msg.ROI.reserve( contours_poly.size() );
        for (unsigned int i=0; i < boundRect.size(); i++)
          {
            cv::Rect rect = boundRect[i];
            color_filter::Rect2D_ rect2D_msg;
            rect2D_msg.x = rect.x;
            rect2D_msg.y = rect.y;
            rect2D_msg.height = rect.height;
            rect2D_msg.width = rect.width;
            obj_msg.ROI.push_back(rect2D_msg);
            obj_msg.ROI_id = ROI_id_counter;
          }
        obj_pub_.publish(obj_msg);
      }




    if (FLAG_SHOW_IMAGE)
      {
        cv::imshow("Original", src_image);

        /// Draw polygonal contour + bonding rectangles
        cv::Mat drawing = cv::Mat::zeros( filter_image_bin.size(), CV_8UC3 );

        for(unsigned int i = 0; i< contours_poly.size(); i++ )
          {
            cv::Scalar color = cv::Scalar( 0, 0, 255 );
            drawContours( drawing, contours_poly, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
          }
        for(unsigned int i = 0; i< boundRect.size(); i++ )
          {
            cv::Scalar color = cv::Scalar( 0, 0, 255 );
            rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
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

};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void on_trackbar( int, void* ) //for later use
{
  /*
  alpha = (double) alpha_slider/alpha_slider_max ;
  beta = ( 1.0 - alpha );

  addWeighted( src1, alpha, src2, beta, 0.0, dst);

  imshow( "Linear Blend", dst );
  */
}

} // END Namespace objRecognition

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//int main(int argc, char** argv)
//{
//        ros::init(argc, argv, "color_filter");
//        if (argc != 1)
//        {
//                if (atoi(argv[1]) == 1)
//                {
//                        objRecognition::FLAG_SHOW_IMAGE = true;
//                }
//                else if (atoi(argv[1]) >= 2)
//                {
//                        //FLAG_SHOW_IMAGE = true;
//                        objRecognition::FLAG_CHANGE_THRESHOLD = true;
//                }
//        }
//        /*
//        cv::Mat drawing = cv::Mat::zeros( 400,400, CV_8UC3 );
//        cv::Scalar color = cv::Scalar( 0, 0, 255 );
//        cv::Point p1_tl,p1_br,p2_tl,p2_br;

//        p1_tl.x = 10;
//        p1_tl.y = 10;
//        p1_br.x = 200;
//        p1_br.y = 200;
//        p2_tl.x = 50;
//        p2_tl.y = 50;
//        p2_br.x = 250;
//        p2_br.y = 250;

//        rectangle( drawing, p1_tl, p1_br, color, 2, 8, 0 );
//        rectangle( drawing, p2_tl, p2_br, color, 2, 8, 0 );
//        cv::imshow("test",drawing);
//        */
//        //cv::namedWindow("Linear Blend", 1);
//        //cv::createTrackbar( " Threshold:", "Linear Blend", &floor_S_MAX, 255, on_trackbar);
//        /// Show some stuff
//        //on_trackbar( floor_S_MAX, 0 );

//        objRecognition::Color_Filter cf;
//        ROS_INFO("Color filter is up and running!");
//        ROS_INFO("Message is being sent to /recognition/detect");

//        ros::spin();

//        return 0;
//}
