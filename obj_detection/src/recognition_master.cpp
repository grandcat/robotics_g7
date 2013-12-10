#include "headers/recognition_master.hpp"
#include "color_filter/Rect2D_.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>      // CV GUI
#include <sensor_msgs/image_encodings.h>

#define RGB_DEPTH_IMG_ALIGNMENT 15          // Alignment correction of depth image to RGB image (in pixel)

namespace objRecognition
{

/**
 * @brief RecognitionMaster::runRecognitionPipeline
 *  Start of the recongition pipeline. This will execute the necessary recognition slaves at a given frame
 *  rate and calculate results.
 * @param msg Primesense image
 */
void RecognitionMaster::runRecognitionPipeline(const sensor_msgs::ImageConstPtr& msg)
{
  ros::Duration(0.1).sleep();
  ++cRcvdDepthFrames;

//  ROS_INFO("[recognition master] received depth image %f", ros::Time::now().toSec());
  // Convert depth image to OpenCV Mat format
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      curDepthImg = cv_ptr->image;
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[Recognition master] cv_bridge exception: %s", e.what());
      return;
    }

  // Turn off recognition while turning 90 degree
  if (!activeDetection)
    return;

  // Get object positions (if any)
  objRecog_Contourfilter.depth_contour_filter(curDepthImg);
  lastObjPositions = objRecog_Contourfilter.getDetectedObjects();
  unsigned int cDetectedObjs = lastObjPositions.size();
  ROS_DEBUG("[Recognition master] Detected amount of objects: %u", cDetectedObjs);

  // Reject if no object was detected
  if (cDetectedObjs != 1)
  {
    ROS_DEBUG("Rejected basic obj detection, more than 2 objects or no objects.");
    return;
  }
//  ROS_DEBUG("1. Obj:\n cm y:%i x:%i", lastObjPositions[0].mc.y, lastObjPositions[0].mc.x);

  explorer::Object relMazePos = translateCvToMap(lastObjPositions[0].mc.y, lastObjPositions[0].mc.x);
  // If position couldn't determined, reject frame
  if (relMazePos.x == -1)
    return;

  ROS_DEBUG("Depth used positon, y: %i, x: %i", lastObjPositions[0].mc.y, lastObjPositions[0].mc.x);

  if (lastRecognizedId == OBJTYPE_NO_OBJECT)
    {
      ROS_DEBUG("Rejected basic obj detection: feature detection didn't work!");
      lastRecognizedId = OBJTYPE_UNKNOWN_OBJECT_DETECTED;
//      return;
    }
  // Don't send last object id again
  if (lastRememberedObjId == lastRecognizedId)
  {
    ++count_LastObjType;
    ROS_DEBUG("Received same object (%s) %i times", TEXT_OBJECTS[lastRememberedObjId].c_str(), count_LastObjType);
  }
  else
  {
    // Detected obj changed, reset counter
    count_LastObjType = 0;
  }
  lastRememberedObjId = lastRecognizedId;
  // Check whether it detected several times the same object
  if (count_LastObjType < 5)
    return;

  // Dont't send last reported object again (is probably the same one)
  if (lastSendObjId == lastRecognizedId) {
    return;
  }
  lastSendObjId = lastRecognizedId;

  // Get recent PCL obj position if there are any -> overwrite contour pos
  if ((lastPclObjPos[0] != -1) && fabs(ros::Time::now().toSec() - lastPclTime.toSec()) < 2.0)
  {
    relMazePos.x = lastPclObjPos[0];
    relMazePos.y = lastPclObjPos[1];
    ROS_INFO("PCL and contour time close, will take PCL; position: x:%f y:%f",
             lastPclObjPos[0], lastPclObjPos[1]);
  }

  // Report object with position
  relMazePos.id = (int)lastRecognizedId;
  ROS_WARN("Report detected object: %s", TEXT_OBJECTS[lastRecognizedId].c_str());
  pub_recognition_result.publish(relMazePos);

  // create competition message
  contest_msgs::evidence msgEvidence;
  msgEvidence.group_number = 7;
  msgEvidence.stamp = ros::Time::now();
  msgEvidence.image_evidence = curRGBImg;
  msgEvidence.object_id = TEXT_OBJECTS[lastRecognizedId];
  pub_evidence.publish(msgEvidence);

  // TESTING (sleeping after each processing is good for system performance)
//  ros::Duration(5).sleep();
}


/**
 * @brief RecognitionMaster::rcvObjType Store obj type information for processing later in rcvSlaveRecognition
 * @param msg
 */
void RecognitionMaster::rcvObjType(const object_recognition::Recognized_objects::ConstPtr msg)
{
  std::vector<int> detectedObjTypes = msg->obj_type;
  lastRecognizedId = (enum EObjectTypes)detectedObjTypes[0];
}

void RecognitionMaster::rcvDepthImg(const sensor_msgs::ImageConstPtr& msg)
{
  ++cRcvdDepthFrames;
//  ros::Duration(4).sleep(); // <-- same thread as rgb image, therefore same rate
  ROS_INFO("[recognition master] received depth image %f", ros::Time::now().toSec());
  // Convert depth image to OpenCV Mat format
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      curDepthImg = cv_ptr->image;
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("[Recognition master] cv_bridge exception: %s", e.what());
      return;
  }
}

void RecognitionMaster::rcvRGBImg(const sensor_msgs::ImageConstPtr& msg)
{
  curRGBImg = *msg;
}

void RecognitionMaster::rcvEKFStop(const explorer::Stop_EKF::ConstPtr& msg)
{
  // Deactivate detection and recognition while turning (lot of false positives)
  if (msg->stop)
  {
    activeDetection = false;
    ROS_INFO("[Recognition master] Paused recognition due to robot turning.");
  }
  else
  {
    activeDetection = true;
    count_LastObjType = 0;
    ROS_INFO("[Recognition master] Reactivated recognition.");
  }

}

void RecognitionMaster::rcvPclObjPos(const explorer::Object::ConstPtr& msg)
{
  ROS_INFO("[Recognition master] Received PCL obj pos.");
  lastPclObjPos[0] = msg->x;    // Distance to object (depth)
  lastPclObjPos[1] = -(msg->y); // distance from center point (same as contour position)
  lastPclTime = ros::Time::now();
}

explorer::Object RecognitionMaster::translateCvToMap(int y, int x)
{
  // Correct alignment of depth image to RGB image
  x += RGB_DEPTH_IMG_ALIGNMENT;
//  y += RGB_DEPTH_IMG_ALIGNMENT;

  explorer::Object relMazePos;
  relMazePos.x = -1;

  if (cRcvdDepthFrames == 0)
    {
      // No frame available: no position estimation possible
      ROS_WARN("[Recognition master] no depth image available");
      return relMazePos;
    }

  float sumDistance = 0;
  int nAvg = 0;

  for (int i = 0; i <= 10; i++)
    {
      for (int j = 0; j <= 10; j++)
        {
          if ((((y + j - 5) < curDepthImg.rows) & ((x + i - 5) < curDepthImg.cols))
              && (((y + j - 5) >= 0) & ((x + i - 5) >= 0)))
            {
              float curDepth = curDepthImg.at<float>(y + j - 5, x + i - 5);
              // DEBUG: mark point for visualization
              curDepthImg.at<float>(y + j - 5, x + i - 5) = 255;

//                                ROS_INFO("Raw value: %f", curDepth);
              if (!isnan(curDepth))
                {
                  sumDistance += curDepth;
                  ++nAvg;
                }
            }
        }
    }

  if (nAvg != 0)
    {
      relMazePos.x = sumDistance / nAvg;// * 1000;
      // calculate y approximation
      relMazePos.y = (320 - x) * (relMazePos.x / 530);

      if (debugWindows)
      {
        // DEBUG: show visualization
        cv::imshow("Depth image with marked point", curDepthImg);
        cvWaitKey(10);
      }
    }
  else
    {
      relMazePos.x = -1;
    }

  return relMazePos;
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "recognition_master");
  ros::NodeHandle nh;
  ROS_INFO("[Recognition master] Starting up...");

  objRecognition::RecognitionMaster recognMaster(nh);

  // Parameters
  if (argc != 1 && atoi(argv[1]) == 1)
  {
    recognMaster.setDebugView();
  }


  ros::spin();
  return 0;
}
