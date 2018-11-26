/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef _POINT_CLOUD_FOOTPRINT_FILTER_H
#define _POINT_CLOUD_FOOTPRINT_FILTER_H
/**
\author Tully Foote
@b ScanFootprintFilter takes input scans and corrects for footprint angle assuming a flat target.  
This is useful for ground plane extraction

**/

#include "laser_geometry/laser_geometry.h"
#include "filters/filter_base.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

namespace range_sensor_filters
{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointCloudFootprintFilter : public filters::FilterBase<sensor_msgs::PointCloud2>
{
public:
  PointCloudFootprintFilter() {}

  bool configure()
  {
    if(!getParam("inscribed_radius", inscribed_radius_))
    {
      ROS_ERROR("PointCloudFootprintFilter needs inscribed_radius to be set");
      return false;
    }

    if(!getParam("base_link_frame", base_link_frame_))
    {
      ROS_ERROR("PointCloudFootprintFilter needs base_link_frame to be set");
      return false;
    }
    
    return true;
  }

  virtual ~PointCloudFootprintFilter()
  { 

  }

  bool update(const sensor_msgs::PointCloud2& input_scan2, sensor_msgs::PointCloud2& filtered_scan2)
  {
    if(&input_scan2 == &filtered_scan2){
      ROS_ERROR("This filter does not currently support in place copying");
      return false;
    }

    PointCloud input_scan, filtered_scan, laser_cloud;
    pcl::fromROSMsg(input_scan2, input_scan);

    try{
      std_msgs::Header header;
      header = pcl_conversions::fromPCL(input_scan.header);
      tf_.waitForTransform(input_scan.header.frame_id, base_link_frame_, header.stamp, ros::Duration(0.2));
      pcl_ros::transformPointCloud(base_link_frame_, input_scan, laser_cloud, tf_);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Transform unavailable %s", ex.what());
      return false;
    }

    filtered_scan.header = input_scan.header;
    filtered_scan.points.resize (input_scan.points.size());

    int num_pts = 0;
    for (unsigned int i=0; i < laser_cloud.points.size(); i++)  
    {
      if (!inFootprint(laser_cloud.points[i])){
        filtered_scan.points[num_pts] = input_scan.points[i];
        num_pts++;
      }
    }

    // Resize output vectors
    filtered_scan.points.resize (num_pts);
    pcl::toROSMsg(filtered_scan, filtered_scan2);

    return true;
  }


  bool inFootprint(const pcl::PointXYZ& scan_pt){
    if(scan_pt.x < -1.0 * inscribed_radius_ || scan_pt.x > inscribed_radius_ || scan_pt.y < -1.0 * inscribed_radius_ || scan_pt.y > inscribed_radius_)
      return false;
    return true;
  }

private:
  tf::TransformListener tf_;
  laser_geometry::LaserProjection projector_;
  double inscribed_radius_;
  std::string base_link_frame_;
} ;

}

#endif // _POINT_CLOUD_FOOTPRINT_FILTER_H
