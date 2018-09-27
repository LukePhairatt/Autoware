/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * pixel_cloud_fusion.h
 *
 *  Created on: May, 19th, 2018
 */

#ifndef PROJECT_PIXEL_CLOUD_FUSION_H
#define PROJECT_PIXEL_CLOUD_FUSION_H

#define __APP_NAME__ "pixel_cloud_fusion"

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Eigen>
#include <atomic>

std::atomic<int>                    camera_info_counter(0);

namespace std {
	template <>
	class hash< cv::Point >{
	public :
		size_t operator()(const cv::Point &pixel_cloud ) const
		{
			return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
		}
	};
};

//NOT USE
class RosCallBack
{
  public:
    void ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg);
};

class RosPixelCloudFusionApp
{
	ros::NodeHandle                     node_handle_;
	ros::Publisher                      publisher_fused_cloud_;
	ros::Subscriber                     intrinsics_subscriber_;

	tf::TransformListener*              transform_listener_;
	tf::StampedTransform                camera_lidar_tf_;

	cv::Size                            image_size_;
	cv::Mat                             camera_instrinsics_;
	cv::Mat                             distortion_coefficients_;
	cv::Mat                             current_frame_;

  std::string 						            image_frame_id_;

	bool                                processing_;
	bool                                camera_info_ok_;
	bool                                camera_lidar_tf_ok_;


  //float                               fx_, fy_, cx_, cy_;
	pcl::PointCloud<pcl::PointXYZRGB>   colored_cloud_;

	typedef
	message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicyT;

	ros::Subscriber                     cloud_subscriber_;
	ros::Subscriber                     image_subscriber_;
	message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

	pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);

  void ImageCallback0(const sensor_msgs::Image::ConstPtr &in_image_msg);
  void ImageCallback1(const sensor_msgs::Image::ConstPtr &in_image_msg);
  void ImageCallback2(const sensor_msgs::Image::ConstPtr &in_image_msg);
  void ImageCallback3(const sensor_msgs::Image::ConstPtr &in_image_msg);
  void ImageCallback4(const sensor_msgs::Image::ConstPtr &in_image_msg);
  void ImageCallback5(const sensor_msgs::Image::ConstPtr &in_image_msg);
  void ImageCallback6(const sensor_msgs::Image::ConstPtr &in_image_msg);
  void ImageCallback7(const sensor_msgs::Image::ConstPtr &in_image_msg);

  void IntrinsicsCallback0(const sensor_msgs::CameraInfo& in_message);
  void IntrinsicsCallback1(const sensor_msgs::CameraInfo& in_message);
  void IntrinsicsCallback2(const sensor_msgs::CameraInfo& in_message);
  void IntrinsicsCallback3(const sensor_msgs::CameraInfo& in_message);
  void IntrinsicsCallback4(const sensor_msgs::CameraInfo& in_message);
  void IntrinsicsCallback5(const sensor_msgs::CameraInfo& in_message);
  void IntrinsicsCallback6(const sensor_msgs::CameraInfo& in_message);
  void IntrinsicsCallback7(const sensor_msgs::CameraInfo& in_message);


	void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg);

  /*!
   *  Helper functions
   */
  void SaveCurrentImageFrame(const sensor_msgs::Image::ConstPtr &in_image_msg, int camera_id);
  void SaveCameraInfo(const sensor_msgs::CameraInfo& in_message, int camera_id);
  bool IsCamInfoReady();

	/*!
	 * Obtains Transformation between two transforms registered in the TF Tree
	 * @param in_target_frame
	 * @param in_source_frame
	 * @return the found transformation in the tree
	 */
  tf::StampedTransform FindTransform(const std::string &in_target_frame, const std::string &in_source_frame, bool& found_tf);
  bool LookupTFCameraTransform(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg);


	/*!
	 * Reads the config params from the command line
	 * @param in_private_handle
	 */
	void InitializeRosIo(ros::NodeHandle &in_private_handle);

  /*!
    * Subscribers: Camera image topics, Camera info, tf image frame id
    */
  XmlRpc::XmlRpcValue           image_topics_vector_;             //image topic
  XmlRpc::XmlRpcValue           caminfo_topics_vector_;           //camera info
  std::vector<std::string> 			image_frame_id_vector_;           //tf image frame id (read from in_image->header.frame_id)
  std::vector<ros::Subscriber>  intrinsics_subscriber_vector_;    //caminfo subscriber nodes
  std::vector<ros::Subscriber>  image_subscriber_vector_;         //image subscriber nodes


  /*!
    * Variables: Camera params for a projection
    */
  int                               numcamera_;
  std::vector<cv::Mat>              camera_instrinsics_vector_;         //aka camera matrix, K (fx,cx,fy,cy)
  std::vector<std::string>          distortion_model_vector_;
  std::vector<cv::Mat>              distortion_coefficients_vector_;    //distortion params
  std::vector<cv::Mat>              current_image_vector_;              //current image data
  std::vector<cv::Mat>              camera_projection_vector_;          //aka projection matrix, P
  //friend class                    RosCallBack;                        //permission to access
  std::vector<tf::StampedTransform> camera_lidar_tf_vector_;            //tf cameras-lidar transform
public:
	void Run();
	RosPixelCloudFusionApp();
};

#endif //PROJECT_PIXEL_CLOUD_FUSION_H
