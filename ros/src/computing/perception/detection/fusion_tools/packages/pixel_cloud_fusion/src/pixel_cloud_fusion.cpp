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
 * pixel_cloud_fusion.cpp
 *
 *  Created on: May, 19th, 2018
 */


#include "pixel_cloud_fusion/pixel_cloud_fusion.h"

pcl::PointXYZ
RosPixelCloudFusionApp::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
	tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
	tf::Vector3 tf_point_t = in_transform * tf_point;
	return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}


void RosPixelCloudFusionApp::SaveCurrentImageFrame(const sensor_msgs::Image::ConstPtr &in_image_msg, int camera_id)
{
  // save current (undistorted) frame
  if (!camera_info_ok_)
  {
    ROS_INFO("[%s] Waiting for Intrinsics to be available for all cameras.", __APP_NAME__);
    return;
  }
  if (processing_)
    return;

  //message to cv image
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
  cv::Mat in_image = cv_image->image;
  //undistorted image depend on a camera type
  if(distortion_model_vector_[camera_id] == "fish_eye")
  {
    cv::Mat newCamMat, map1, map2;
    cv::Size imageSize(in_image.size());
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_instrinsics_vector_[camera_id], distortion_coefficients_vector_[camera_id],
                                                            imageSize, cv::Matx33d::eye(), newCamMat, 1);
    cv::fisheye::initUndistortRectifyMap(camera_instrinsics_vector_[camera_id],  distortion_coefficients_vector_[camera_id],
                                         cv::Matx33d::eye(), newCamMat, imageSize, CV_16SC2, map1, map2);
    remap(in_image, current_image_vector_[camera_id], map1, map2, cv::INTER_LINEAR);
  }
  else
  {
    cv::undistort(in_image, current_image_vector_[camera_id], camera_instrinsics_vector_[camera_id], distortion_coefficients_vector_[camera_id]);
  }

  image_frame_id_vector_[camera_id] = in_image_msg->header.frame_id;
  image_size_.height = current_image_vector_[camera_id].rows;
  image_size_.width = current_image_vector_[camera_id].cols;
}

void RosPixelCloudFusionApp::ImageCallback0(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 0);
}

void RosPixelCloudFusionApp::ImageCallback1(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 1);
}

void RosPixelCloudFusionApp::ImageCallback2(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 2);
}

void RosPixelCloudFusionApp::ImageCallback3(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 3);
}

void RosPixelCloudFusionApp::ImageCallback4(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 4);
}

void RosPixelCloudFusionApp::ImageCallback5(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 5);
}

void RosPixelCloudFusionApp::ImageCallback6(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 6);
}

void RosPixelCloudFusionApp::ImageCallback7(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
  SaveCurrentImageFrame(in_image_msg, 7);
}

void RosPixelCloudFusionApp::IntrinsicsCallback0(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 0);
}

void RosPixelCloudFusionApp::IntrinsicsCallback1(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 1);
}

void RosPixelCloudFusionApp::IntrinsicsCallback2(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 2);
}

void RosPixelCloudFusionApp::IntrinsicsCallback3(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 3);
}

void RosPixelCloudFusionApp::IntrinsicsCallback4(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 4);
}

void RosPixelCloudFusionApp::IntrinsicsCallback5(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 5);
}

void RosPixelCloudFusionApp::IntrinsicsCallback6(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 6);
}

void RosPixelCloudFusionApp::IntrinsicsCallback7(const sensor_msgs::CameraInfo &in_message)
{
  SaveCameraInfo(in_message, 7);
}

void RosPixelCloudFusionApp::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{
  //lookup tf camera(s) and lidar if not already done
	if (!camera_lidar_tf_ok_)
	{
    //camera_lidar_tf_ = FindTransform(image_frame_id_, in_cloud_msg->header.frame_id);
    camera_lidar_tf_ok_ = LookupTFCameraTransform(in_cloud_msg);

	}
  //check if all camera info
	if (!camera_info_ok_ || !camera_lidar_tf_ok_)
	{
    ROS_INFO("[%s] Waiting for Camera Intrinsics to be available.", __APP_NAME__);
		return;
	}

  //tf and camera info are all received and ready to go
  //find rgb color for points
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*in_cloud_msg, *in_cloud);
  std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;

  //find a corresponding image pixel from all camera
  pcl::PointXYZ cam_cloud;
  std::vector<bool> cam_cloud_insert(in_cloud->points.size());
  pcl::PointXYZRGB colored_3d_point;

  double fx, fy, cx, cy;
  bool run_rgb = false;
  out_cloud->points.clear();

#pragma omp for
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
    //lidar point projection onto the image
    run_rgb = true;
    for(int camid=0;camid<numcamera_;camid++)
    {
      //step 0: initial: are all camera images ready
      if(current_image_vector_[camid].rows <= 0 ||
         current_image_vector_[camid].cols <= 0 ||
         image_frame_id_vector_[camid] == "")
      {
        run_rgb = false;
      }
      //step 1: get projection matrix for this specific camera (intrinsic params 3x3)
      fx = camera_instrinsics_vector_[camid].at<double>(0, 0);
      cx = camera_instrinsics_vector_[camid].at<double>(0, 2);
      fy = camera_instrinsics_vector_[camid].at<double>(1, 1);
      cy = camera_instrinsics_vector_[camid].at<double>(1, 2);

      //step 2: a lidar point in this camera coordinate frame
      cam_cloud = TransformPoint(in_cloud->points[i], camera_lidar_tf_vector_[camid]);

      //step 3: project the point to the image plane u,v
      int u = int(cam_cloud.x * fx / cam_cloud.z + cx);
      int v = int(cam_cloud.y * fy / cam_cloud.z + cy);

      //step 4: check if it in this plane and not already inserted
      //(TODO fusing: using average RGB or finding the best one?) using the first one for now
      if ((u >= 0) && (u < image_size_.width) &&
          (v >= 0) && (v < image_size_.height) &&
          (cam_cloud.z > 0) && !(cam_cloud_insert[i]) && run_rgb
          )
      {
        projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(u, v), in_cloud->points[i]));
        cam_cloud_insert[i] = true;
      }
    }
  }

  //lookup pixels
#pragma omp for
  for(int camid=0;camid<numcamera_;camid++)
  {
      for (int row = 0; row < image_size_.height; row++)
      {
        for (int col = 0; col < image_size_.width; col++)
        {
          std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator iterator_3d_2d;
          pcl::PointXYZ corresponding_3d_point;
          pcl::PointXYZRGB colored_3d_point;
          iterator_3d_2d = projection_map.find(cv::Point(col, row));
          if (iterator_3d_2d != projection_map.end())
          {
            corresponding_3d_point = iterator_3d_2d->second;
            cv::Vec3b rgb_pixel = current_image_vector_[camid].at<cv::Vec3b>(row, col);
            colored_3d_point.x = corresponding_3d_point.x;
            colored_3d_point.y = corresponding_3d_point.y;
            colored_3d_point.z = corresponding_3d_point.z;
            colored_3d_point.r = rgb_pixel[2];
            colored_3d_point.g = rgb_pixel[1];
            colored_3d_point.b = rgb_pixel[0];
            out_cloud->points.push_back(colored_3d_point);
          }
        }
      }
  }

	// Publish PC
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*out_cloud, cloud_msg);
  cloud_msg.header = in_cloud_msg->header;
  publisher_fused_cloud_.publish(cloud_msg);
}

void RosPixelCloudFusionApp::SaveCameraInfo(const sensor_msgs::CameraInfo& in_message, int camera_id)
{
  // use frame_id to check what camera_info received
  image_size_.height = in_message.height;
  image_size_.width = in_message.width;
  distortion_model_vector_[camera_id] = in_message.distortion_model;
  camera_instrinsics_vector_[camera_id] = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      camera_instrinsics_vector_[camera_id].at<double>(row, col) = in_message.K[row * 3 + col];
    }
  }

  //standard or fisheye camera coefficient
  int dim = (in_message.distortion_model == "fish_eye") ? 4:5;

  distortion_coefficients_vector_[camera_id] = cv::Mat(1, dim, CV_64F);
  for (int col = 0; col < dim; col++)
  {
    distortion_coefficients_vector_[camera_id].at<double>(col) = in_message.D[col];
  }
  //projection matrix(use camera_intrinsic)

  //no more subscription needed
  intrinsics_subscriber_vector_[camera_id].shutdown();
  camera_info_counter++;
  //check if all camera infos received
  if(camera_info_counter == numcamera_)
  {
    camera_info_ok_ = true;
    camera_info_counter = 0;
  }
  ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);

}

tf::StampedTransform
RosPixelCloudFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame, bool& found_tf)
{
	tf::StampedTransform transform;
	try
	{
		transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
    found_tf = true;
		ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
	}
	catch (tf::TransformException ex)
	{
    //reset flag and print warning
    found_tf = false;
		ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
	}

	return transform;
}

bool RosPixelCloudFusionApp::LookupTFCameraTransform(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{
  bool found_tf;
  bool all_tf = true;
  //find all cameras tf
  for(int i=0;i<numcamera_;i++)
  {
    if(image_frame_id_vector_[i]== "" || in_cloud_msg->header.frame_id == "")
    {
      ROS_INFO("[%s] One of image or point cloud frame has on frame_id.", __APP_NAME__);
      return false;
    }
    camera_lidar_tf_vector_[i] = FindTransform(image_frame_id_vector_[i], in_cloud_msg->header.frame_id, found_tf);
    all_tf &= found_tf;
  }
  return all_tf;

}


void RosPixelCloudFusionApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
  camera_info_counter = 0;
  //get params vector for multiple cameras
  //image topics
  in_private_handle.getParam("/cam_topics", image_topics_vector_);
  ROS_ASSERT(image_topics_vector_.getType() == XmlRpc::XmlRpcValue::TypeArray);
  //camera infos
  in_private_handle.getParam("/cam_infos", caminfo_topics_vector_);
  ROS_ASSERT(caminfo_topics_vector_.getType() == XmlRpc::XmlRpcValue::TypeArray);
  //check if there is a same number of cameras and camera info
  ROS_ASSERT(image_topics_vector_.size() == caminfo_topics_vector_.size());

  //init how many cameras use for lidar color
  numcamera_ = image_topics_vector_.size();
  //init size of tf camera name vector
  image_frame_id_vector_ = std::vector<std::string>(numcamera_);
  //init size of current image frame vector
  current_image_vector_  = std::vector<cv::Mat>(numcamera_);
  //init size of camera intrinsic vector
  camera_instrinsics_vector_ = std::vector<cv::Mat>(numcamera_);
  //init size of camera projection vector
  camera_projection_vector_  = std::vector<cv::Mat>(numcamera_);
  //init size of distortion vector
  distortion_coefficients_vector_ = std::vector<cv::Mat>(numcamera_);
  //init intrinsic subscriber vector
  intrinsics_subscriber_vector_ = std::vector<ros::Subscriber>(numcamera_);
  //init image subscriber vector
  image_subscriber_vector_ = std::vector<ros::Subscriber>(numcamera_);
  //init tf transform cameras-lidar
  camera_lidar_tf_vector_ = std::vector<tf::StampedTransform>(numcamera_);
  //init distortion model type
  distortion_model_vector_ =  std::vector<std::string>(numcamera_);

	//get params
  std::string points_src, image_src, camera_info_src, fused_topic_str = "/points_fused";
	std::string name_space_str = ros::this_node::getNamespace();

	ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Image, and PointCloud.", __APP_NAME__);
	in_private_handle.param<std::string>("points_src", points_src, "/points_raw");
	ROS_INFO("[%s] points_src: %s", __APP_NAME__, points_src.c_str());

	if (name_space_str != "/")
	{
		if (name_space_str.substr(0, 2) == "//")
		{
			name_space_str.erase(name_space_str.begin());
		}
    //image_src = name_space_str + image_src;
		fused_topic_str = name_space_str + fused_topic_str;
    //camera_info_src = name_space_str + camera_info_src;
	}

	//generate subscribers and sychronizers
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[0]).c_str());
  intrinsics_subscriber_vector_[0] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[0]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback0, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[1]).c_str());
  intrinsics_subscriber_vector_[1] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[1]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback1, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[2]).c_str());
  intrinsics_subscriber_vector_[2] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[2]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback2, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[3]).c_str());
  intrinsics_subscriber_vector_[3] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[3]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback3, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[4]).c_str());
  intrinsics_subscriber_vector_[4] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[4]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback4, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[5]).c_str());
  intrinsics_subscriber_vector_[5] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[5]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback5, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[6]).c_str());
  intrinsics_subscriber_vector_[6] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[6]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback6, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(caminfo_topics_vector_[7]).c_str());
  intrinsics_subscriber_vector_[7] = in_private_handle.subscribe(static_cast<std::string>(caminfo_topics_vector_[7]),1,
                                     &RosPixelCloudFusionApp::IntrinsicsCallback7, this);


  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[0]).c_str());
  image_subscriber_vector_[0] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[0]),1,
                      &RosPixelCloudFusionApp::ImageCallback0, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[1]).c_str());
  image_subscriber_vector_[1] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[1]),1,
                      &RosPixelCloudFusionApp::ImageCallback1, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[2]).c_str());
  image_subscriber_vector_[2] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[2]),1,
                      &RosPixelCloudFusionApp::ImageCallback2, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[3]).c_str());
  image_subscriber_vector_[3] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[3]),1,
                      &RosPixelCloudFusionApp::ImageCallback3, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[4]).c_str());
  image_subscriber_vector_[4] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[4]),1,
                      &RosPixelCloudFusionApp::ImageCallback4, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[5]).c_str());
  image_subscriber_vector_[5] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[5]),1,
                      &RosPixelCloudFusionApp::ImageCallback5, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[6]).c_str());
  image_subscriber_vector_[6] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[6]),1,
                      &RosPixelCloudFusionApp::ImageCallback6, this);
  ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, static_cast<std::string>(image_topics_vector_[7]).c_str());
  image_subscriber_vector_[7] = in_private_handle.subscribe(static_cast<std::string>(image_topics_vector_[7]),1,
                      &RosPixelCloudFusionApp::ImageCallback7, this);


	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, points_src.c_str());
  cloud_subscriber_ = in_private_handle.subscribe(points_src, 1, &RosPixelCloudFusionApp::CloudCallback, this);

	publisher_fused_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(fused_topic_str, 1);
	ROS_INFO("[%s] Publishing fused pointcloud in %s", __APP_NAME__, fused_topic_str.c_str());


  // Multiple subscriptions
  /*
  for(int i=0;i<image_topics_vector_.size();i++)
  {
    intrinsics_subscriber_vector_.push_back(in_private_handle.subscribe(caminfo_topics_vector_[i], 1,
                                                         &RosPixelCloudFusionApp::IntrinsicsCallback, this));

    image_subscriber_vector_.push_back(in_private_handle.subscribe(image_topics_vector_[i], 1,
                                                                   &RosPixelCloudFusionApp::ImageCallback, this));
  }*/


}


void RosPixelCloudFusionApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	tf::TransformListener transform_listener;

	transform_listener_ = &transform_listener;

	InitializeRosIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END", __APP_NAME__);
}

RosPixelCloudFusionApp::RosPixelCloudFusionApp()
{
	camera_lidar_tf_ok_ = false;
	camera_info_ok_ = false;
	processing_ = false;
	image_frame_id_ = "";
}
