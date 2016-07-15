/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-2015
 *  Technische Hochschule Nürnberg Georg Simon Ohm
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
 *   * Neither the name of Nuremberg Institute of Technology
 *     Georg Simon Ohm nor the authors names may be used to endorse
 *     or promote products derived from this software without specific
 *     prior written permission.
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
 *
 * Author: Stefan May
 *********************************************************************/

#include <stdint.h>

#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include "libirimager/ImageBuilder.h"

#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

#include "optris_drivers/ExtremaRegion.h"

#include "optris_drivers/GetTemperatureAt.h"
#include "optris_drivers/GetMeanTemperature.h"
#include "optris_drivers/GetMinMaxRegion.h"
#include "optris_drivers/SetManualTemperatureRange.h"
#include "optris_drivers/GetIsothermalMin.h"
#include "optris_drivers/GetIsothermalMax.h"
#include "optris_drivers/SetPaletteScalingMethod.h"
#include "optris_drivers/SetPalette.h"

unsigned char*                    _bufferThermal = NULL;
unsigned char*                    _bufferVisible = NULL;
image_transport::Publisher*       _pubThermal;
image_transport::Publisher*       _pubVisible;
unsigned int                      _frame = 0;

optris::ImageBuilder              _iBuilder;
optris::EnumOptrisColoringPalette _palette;

sensor_msgs::CameraInfo _camera_info;
image_transport::CameraPublisher* _camera_info_pub = NULL;
camera_info_manager::CameraInfoManager* _camera_info_manager = NULL;

ros::NodeHandle* node;

bool get_temperature_at(optris_drivers::GetTemperatureAt::Request& req, optris_drivers::GetTemperatureAt::Response& res)
{
  // Query the temperature of the current frame in the image builder.
  res.temperature = static_cast<float>( _iBuilder.getTemperatureAt(static_cast<int>(req.col), static_cast<int>(req.row)) );
  ROS_INFO("Requested Temperature at (%03d,Y=%03d): %f Degrees C.", req.col, req.row, res.temperature);

  // All done.
  return true;
}

bool get_mean_temperature(optris_drivers::GetMeanTemperature::Request& req, optris_drivers::GetTemperatureAt::Response& res)
{
  // Query the mean temperature of a region in the current frame in the image builder.
  res.temperature = static_cast<float>( _iBuilder.getMeanTemperature(static_cast<int>(req.col1), static_cast<int>(req.row1), static_cast<int>(req.col2), static_cast<int>(req.row2)) );
  ROS_INFO("Requested Mean Temperature in (%03d,%03d)-(%03d,%03d): %f Degrees C.", req.col1, req.row1, req.col2, req.row2, res.temperature);

  // All done.
  return true;
}

bool get_min_max_region(optris_drivers::GetMinMaxRegion::Request& req, optris_drivers::GetMinMaxRegion::Response& res)
{
  // Create a pair of Optris ExtremalRegions to read data into.
  optris::ExtremalRegion min;
  optris::ExtremalRegion max;

  // Query the min and max temperature regions in the current frame in the imager builder.
  _iBuilder.getMinMaxRegion(static_cast<int>(req.radius), &min, &max);

  // Extract the region data into the response.
  res.min.temperature = static_cast<float>(min.t);
  res.min.col1 = static_cast<uint16_t>(min.u1);
  res.min.row1 = static_cast<uint16_t>(min.v1);
  res.min.col2 = static_cast<uint16_t>(min.u2);
  res.min.row2 = static_cast<uint16_t>(min.v1);
  res.max.temperature = static_cast<float>(max.t);
  res.max.col1 = static_cast<uint16_t>(max.u1);
  res.max.row1 = static_cast<uint16_t>(max.v1);
  res.max.col2 = static_cast<uint16_t>(max.u2);
  res.max.row2 = static_cast<uint16_t>(max.v1);

  // All done.
  return true;
}

bool set_manual_temperature_range(optris_drivers::SetManualTemperatureRange::Request& req, optris_drivers::SetManualTemperatureRange::Response& res)
{
  // Set the temperature range in the image builder.
  _iBuilder.setManualTemperatureRange(static_cast<float>(req.min), static_cast<float>(req.max));
  ROS_INFO("Set manual temperature scaling range: %f to %f Degrees C.", req.min, req.max);

  // Update the relevant node parameters.
  node->setParam("temperatureMin", static_cast<double>(req.min));
  node->setParam("temperatureMax", static_cast<double>(req.max));

  // We always respond true.
  res.result = true;

 // All done.
 return true;
}

bool get_isothermal_min(optris_drivers::GetIsothermalMin::Request& req, optris_drivers::GetIsothermalMin::Response& res)
{
  // Query the isothermal min which the image builder is currently using.
  res.min = static_cast<float>( _iBuilder.getIsothermalMin() );
  ROS_INFO("Requested Isothermal Min: %f Degrees C.", res.min);

  // All done.
  return true;
}

bool get_isothermal_max(optris_drivers::GetIsothermalMax::Request& req, optris_drivers::GetIsothermalMax::Response& res)
{
  // Query the isothermal max which the image builder is currently using.
  res.max = static_cast<float>( _iBuilder.getIsothermalMax() );
  ROS_INFO("Requested Isothermal Max: %f Degrees C.", res.max);

  // All done.
  return true;
}

bool set_palette_scaling_method(optris_drivers::SetPaletteScalingMethod::Request& req, optris_drivers::SetPaletteScalingMethod::Response& res)
{
  // Check to make sure a valid scaling method was specified.
  if ((req.method < 1) || (req.method > 4))
  {
    // The method enumeration specified wasn't valid.
    ROS_ERROR("Invalid palette scaling method requested: %d is not in the range 1-4.", req.method);
    res.result = false;
    return false;
  }

  // Set the scaling method in the image builder.
  _iBuilder.setPaletteScalingMethod(static_cast<optris::EnumOptrisPaletteScalingMethod>(req.method));
  ROS_INFO("Set palette scaling method: Mode %d.", req.method);

  // Update the relevant node parameter.
  node->setParam("paletteScaling", static_cast<uint8_t>(req.method));

  // We respond true if we changed the method successfully.
  res.result = true;

 // All done.
 return true;
}

bool set_palette(optris_drivers::SetPalette::Request& req, optris_drivers::SetPalette::Response& res)
{
  // Check to maek sure a valid palette was specified.
  if ((req.palette < 1) || (req.palette > 11))
  {
    // The palette enumeration specified wasn't valid.
    ROS_ERROR("Invalid palette requested: %d is not in the range 1-11.", req.palette);
    res.result = false;
    return false;
  }

  // Set the palette in the image builder.
  _iBuilder.setPalette(static_cast<optris::EnumOptrisColoringPalette>(req.palette));
  ROS_INFO("Set palette: Mode %d.", req.palette);

  // Update the relevant node paramter.
  node->setParam("palette", static_cast<uint8_t>(req.palette));

  // We response true if we changed the palette sucessfully.
  res.result = true;

  // All done.
  return true;
}

void onThermalDataReceive(const sensor_msgs::ImageConstPtr& image)
{
   // check for any subscribers to save computation time
  if((_pubThermal->getNumSubscribers() == 0) && (_camera_info_pub->getNumSubscribers() == 0))
     return;

  unsigned short* data = (unsigned short*)&image->data[0];
  _iBuilder.setData(image->width, image->height, data);

  if(_bufferThermal==NULL)
    _bufferThermal = new unsigned char[image->width * image->height * 3];

  _iBuilder.convertTemperatureToPaletteImage(_bufferThermal, true);

  sensor_msgs::Image img;
  img.header.frame_id = "thermal_image_view";
  img.height 	        = image->height;
  img.width 	        = image->width;
  img.encoding        = "rgb8";
  img.step            = image->width*3;
  img.header.seq      = ++_frame;
  img.header.stamp    = ros::Time::now();

  // copy the image buffer
  img.data.resize(img.height*img.step);
  memcpy(&img.data[0], &_bufferThermal[0], img.height * img.step * sizeof(*_bufferThermal));

  _camera_info = _camera_info_manager->getCameraInfo();
  _camera_info.header = img.header;
  _camera_info_pub->publish(img, _camera_info);

  _pubThermal->publish(img);
}

void onVisibleDataReceive(const sensor_msgs::ImageConstPtr& image)
{
  // check for any subscribers to save computation time
  if(_pubVisible->getNumSubscribers() == 0)
     return;

  if(_bufferVisible==NULL)
    _bufferVisible = new unsigned char[image->width * image->height * 3];

  const unsigned char* data = &image->data[0];
  _iBuilder.yuv422torgb24(data, _bufferVisible, image->width, image->height);

  sensor_msgs::Image img;
  img.header.frame_id = "visible_image_view";
  img.height          = image->height;
  img.width           = image->width;
  img.encoding        = "rgb8";
  img.step            = image->width*3;
  img.data.resize(img.height*img.step);

  img.header.seq      = _frame;
  img.header.stamp    = ros::Time::now();

  for(unsigned int i=0; i<image->width*image->height*3; i++) {
    img.data[i] = _bufferVisible[i];
  }

  _pubVisible->publish(img);
}

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "optris_colorconvert_node");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");
  node = &n_;

  int palette = 6;
  n_.getParam("palette", palette);
  _palette = (optris::EnumOptrisColoringPalette) palette;

  optris::EnumOptrisPaletteScalingMethod scalingMethod = optris::eMinMax;
  int sm;
  n_.getParam("paletteScaling", sm);
  if(sm>=1 && sm <=4) scalingMethod = (optris::EnumOptrisPaletteScalingMethod) sm;

  _iBuilder.setPaletteScalingMethod(scalingMethod);
  _iBuilder.setPalette(_palette);

  double tMin     = 20.;
  double tMax     = 40.;
  double looprate = 30.;

  n_.getParam("temperatureMin", tMin);
  n_.getParam("temperatureMax", tMax);
  n_.getParam("looprate",       looprate);

  _iBuilder.setManualTemperatureRange((float)tMin, (float)tMax);

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subThermal = it.subscribe("thermal_image", 1, onThermalDataReceive);
  image_transport::Subscriber subVisible = it.subscribe("visible_image", 1, onVisibleDataReceive);

  image_transport::Publisher pubt = it.advertise("thermal_image_view", 1);
  image_transport::Publisher pubv = it.advertise("visible_image_view", 1);

  _pubThermal = &pubt;
  _pubVisible = &pubv;

  std::string camera_name;
  std::string camera_info_url;
  n_.getParam("camera_name", camera_name);
  n_.getParam("camera_info_url", camera_info_url);

  // initialize CameraInfoManager, providing set_camera_info service for geometric calibration
  // see http://wiki.ros.org/camera_info_manager
  camera_info_manager::CameraInfoManager cinfo_manager(n);
  _camera_info_manager = &cinfo_manager;

  if (!_camera_info_manager->setCameraName(camera_name))
  {
    // GUID is 16 hex digits, which should be valid.
    // If not, use it for log messages anyway.
    ROS_WARN_STREAM("[" << camera_name
                    << "] name not valid"
                    << " for camera_info_manger");
  }

  if (_camera_info_manager->validateURL(camera_info_url))
  {
    if ( !_camera_info_manager->loadCameraInfo(camera_info_url) )
    {
      ROS_WARN( "camera_info_url does not contain calibration data." );
    } 
    else if ( !_camera_info_manager->isCalibrated() )
    {
      ROS_WARN( "Camera is not calibrated. Using default values." );
    } 
  } 
  else
  {
    ROS_ERROR_STREAM_ONCE( "Calibration URL syntax is not supported by CameraInfoManager." );
  }

  // Advertise a synchronized camera raw image + info topic pair with subscriber status callbacks.
  image_transport::CameraPublisher cinfo_pub = it.advertiseCamera("image_raw", 1);
  _camera_info_pub = &cinfo_pub;

  // set to png compression
  std::string key;
  if(ros::param::search("thermal_image/compressed/format", key))
  {
     ros::param::set(key, "png");
  }
  if(ros::param::search("thermal_image/compressed/png_level", key))
  {
     ros::param::set(key, 9);
  }

  // Advertise services exposing image builder functions.
  ros::ServiceServer service_get_temperature_at = n_.advertiseService("get_temperature_at", get_temperature_at);
  ros::ServiceServer service_get_mean_temperature = n_.advertiseService("get_mean_temperature", get_mean_temperature);
  ros::ServiceServer service_set_manual_temperature_range = n_.advertiseService("set_manual_temperature_range", set_manual_temperature_range);
  ros::ServiceServer service_get_isothermal_min = n_.advertiseService("get_isothermal_min", get_isothermal_min);
  ros::ServiceServer service_get_isothermal_max = n_.advertiseService("get_isothermal_max", get_isothermal_max);
  ros::ServiceServer service_set_palette_scaling_method = n_.advertiseService("set_palette_scaling_method", set_palette_scaling_method);
  ros::ServiceServer service_set_palette = n_.advertiseService("set_palette", set_palette);

  // specify loop rate: a meaningful value according to your publisher configuration
  ros::Rate loop_rate(looprate);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if(_bufferThermal)	 delete [] _bufferThermal;
  if(_bufferVisible)  delete [] _bufferVisible;
}
