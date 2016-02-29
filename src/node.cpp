/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//Boost headers
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/filesystem.hpp>

// C++ std headers
#include <string>

class CameraV4LPublisher {
public:
    CameraV4LPublisher(ros::NodeHandle& nh);
    virtual ~CameraV4LPublisher();

    void run();

protected:

    void open();
    void locateCameraNode();
    bool isThereAnySubscriber() const;
    bool getImage(cv::Mat& img, bool emptyBuffer);
    void publish(const cv::Mat& img);

    ros::NodeHandle& _nh;
    int _node;
    int _frameRate;
    cv::Size _imageSize;
    boost::scoped_ptr<cv::VideoCapture> _capture;
    std::string _deviceLocation;

    boost::scoped_ptr<image_transport::ImageTransport> _imageTransport;
    sensor_msgs::CameraInfo _camInfoMsg;
    boost::scoped_ptr<image_transport::CameraPublisher> _cameraPublisher;

};

CameraV4LPublisher::CameraV4LPublisher(ros::NodeHandle &nh):
    _nh(nh), _node(0), _frameRate(15), _imageSize(640, 480)
{
  _nh.param<int>("frame_rate", _frameRate, _frameRate);
  _nh.param<int>("width", _imageSize.width, _imageSize.width);
  _nh.param<int>("height", _imageSize.height, _imageSize.height);

  open();
}

CameraV4LPublisher::~CameraV4LPublisher()
{
  if ( _cameraPublisher.get() != NULL )
    _cameraPublisher->shutdown();
}

void CameraV4LPublisher::locateCameraNode()
{
  int i = 0;
  bool found = false;

  while ( i < 10 && !found )
  {
    std::stringstream path;
    path << "/dev/video" << i;
    if ( boost::filesystem::exists( path.str() ) )
    {
      found = true;
      _deviceLocation = path.str();
      _node = i;
    }
    else
      ++i;
  }

  if ( !found )
  {
    _node = -1;
    throw std::runtime_error("Error in CameraV4LPublisher::locateCameraNode: no camera found");
  }
  else
    ROS_INFO_STREAM("Camera found in /dev/video" << _node);
}

void CameraV4LPublisher::open()
{
  locateCameraNode();

  if ( _node == -1 )
    throw std::runtime_error("Error in CameraV4LPublisher::open(): device not found");

  ROS_INFO_STREAM("Creating cv::VideoCapture in node " << _node);
  _capture.reset( new cv::VideoCapture(_node) );

  ROS_INFO_STREAM("Setting image width to " << _imageSize.width << " pixels");
  _capture->set(CV_CAP_PROP_FRAME_WIDTH,  _imageSize.width);
  ROS_INFO_STREAM("Setting image height to " << _imageSize.height << " pixels");
  _capture->set(CV_CAP_PROP_FRAME_HEIGHT, _imageSize.height);

  //set up ROS publisher
  _imageTransport.reset( new image_transport::ImageTransport(_nh) );
  _cameraPublisher.reset( new image_transport::CameraPublisher(
                            _imageTransport->advertiseCamera(_nh.getNamespace()+"/image", 1)) );
}

bool CameraV4LPublisher::isThereAnySubscriber() const
{
  return _cameraPublisher->getNumSubscribers() > 0;
}

bool CameraV4LPublisher::getImage(cv::Mat& img, bool emptyBuffer)
{
  bool ok = false;
  if ( emptyBuffer )
  {
    for (int i = 0; i < 4; ++i) //v4l stores a buffer of 4 images
      _capture->grab();
  }
  if ( _capture->grab() )
    if ( _capture->retrieve(img) )
      ok = true;

  return ok;
}

void CameraV4LPublisher::publish(const cv::Mat& img)
{
  sensor_msgs::Image msgImage;
  cv_bridge::CvImage cvImg;

  if ( img.channels() == 3 && img.depth() == CV_8U )
    cvImg.encoding = "bgr8";
  else if ( img.channels() == 1 && img.depth() == CV_8U )
    cvImg.encoding = "mono8";
  else
  {
    std::runtime_error("Error in CameraV4LPublisher::run(): only 24-bit RGB and 8-bit Mono images are supported");
  }

  if ( img.data != NULL )
    cvImg.image = img;
  cvImg.toImageMsg(msgImage);

  msgImage.header.stamp = ros::Time::now();
  _camInfoMsg.header.stamp = msgImage.header.stamp;
  _cameraPublisher->publish(msgImage, _camInfoMsg, msgImage.header.stamp);
}

void CameraV4LPublisher::run()
{
  ROS_INFO_STREAM("Setting loop rate to: " << _frameRate << " Hz");
  ros::Rate loopRate(static_cast<double>(_frameRate));

  bool stopped = true;
  cv::Mat img;

  while ( ros::ok() )
  {
    if (isThereAnySubscriber())
    {
       if ( stopped )
         ROS_INFO("Subscriber detected => Starting camera");
       getImage(img, stopped);
       stopped = false;
       publish(img);
    }
    else if ( !stopped )
    {
      ROS_INFO("There are no subscribers => Stopping camera");
      stopped = true;
    }

    loopRate.sleep();
  }
  ROS_INFO("Stopping CameraV4LPublisher ...");
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_v4l");

  ros::NodeHandle nh("~");             //use node name as sufix of the namespace

  CameraV4LPublisher camera(nh);
  camera.run();


  return 0;
}
