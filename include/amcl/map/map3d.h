#ifndef _MAP3D__H
#define _MAP3D__H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

//特征
#include <pcl/features/normal_3d.h>
//配准
#include <pcl/registration/icp.h> //ICP方法
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <boost/make_shared.hpp> //共享指针
#include <unordered_map>

using namespace std;

typedef pcl::PointNormal PointNormalT;
// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//namespace amcl
//{

class Map3dCloud
{
	public:
		Map3dCloud(ros::NodeHandle nh);
		~Map3dCloud(){}
		
		//void localization();
		void load_gloabalmap(const string &globalmap_pcd_dir);
		void generate_refmap(const Eigen::Isometry3d &cameraPose);
		//void set_currmap(const PointCloud::Ptr currCloud,const Eigen::Isometry3d &cameraPose);
	private:
		PointCloud::Ptr global_map_;//全局三维点云地图
		PointCloud::Ptr ref_map_;//参考子地图点云
		//PointCloud::Ptr curr_map_;//当前观测的局部地图点云
		
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		ros::Publisher global_map_pub_;
		//ros::Publisher ref_map_pub_;
        //ros::Publisher curr_map_pub_;
};

//}

#endif