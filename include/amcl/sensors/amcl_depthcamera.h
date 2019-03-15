///////////////////////////////////////////////////////////////////////////
//
// Desc: 3d point cloud model for AMCL
// Author: Lailihai
// Date: 2018.6
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_DEPTHCAMERA_H
#define AMCL_DEPTHCAMERA_H

#include "amcl_sensor.h"
#include "../map/map3d.h"
//#include "../map/NDTLite.h"
#include "../map/IRON.h"

namespace amcl
{

struct Point3DAccessor
{
    static const float &x(const PointT &pt)
    {
        return pt.x;
    }
    static const float &y(const PointT &pt)
    {
        return pt.y;
    }
    static const float &z(const PointT &pt)
    {
        return pt.z;
    }
};


class AMCLDepthCameraData : public AMCLSensorData
{
//基类AMCLSensorData中包含 AMCLSensor对象指针
public:
    AMCLDepthCameraData():pcloud(new PointCloud())
    {

    }
    virtual ~AMCLDepthCameraData()
    {
    }

    PointCloud::Ptr pcloud;//当前观测的局部地图点云

};


class AMCLDepthCamera : public AMCLSensor
{
  
  // Default constructor
  public:
    AMCLDepthCamera();
    //AMCLDepthCamera(const AMCLDepthCamera & other){};
    virtual ~AMCLDepthCamera();
    void SetCameraPose(Eigen::Isometry3d& camera_pose) 
          {this->camera_pose_ = camera_pose;}

    void SetNDTMatchParam( const float& ndt_cellSize,
                           const float& ndt_subsamplingFactor,
                           const float& ndt_clippingDistance,
                           const float& iron_matchingTolerance,
                           const float& iron_entropyThreshold,
                           const float& iron_neighborSearchRadius,
                           const std::string& global_map3d_dir, 
                           const bool flag_global_process,
                           
                           const double &globalmap_downsam_res,
                           const double &currmap_downsam_res
                           
                           );
    virtual bool UpdateSensor(pf_t *pf, AMCLSensorData *data);
  private: 
    static double NDTmatchModel(AMCLDepthCameraData *data, pf_sample_set_t* set);
    //double time;
    //int parameters;//在这里定义点云配准相关的参数
    static IRON::NDTLiteConfig ndtcfg_;//NDT变换的配置参数
    static IRON::NDTMapLiteCreator<float, std::vector<PointT, Eigen::aligned_allocator<PointT> >, Point3DAccessor> creator_;

    // prepare IRON engine
    static IRON::IRONConfig ironcfg_;
    static IRON::IRONEnginef engine_;

    // camera pose offset relative to robot
    //pf_vector_t camera_pose_;
    Eigen::Isometry3d camera_pose_;
    static Map3dCloud* map3d_;

    //double globalmap_downsam_res_;
    static double currmap_downsam_res_;

};

IRON::NDTMapLiteCreator<float, std::vector<PointT, Eigen::aligned_allocator<PointT> >, Point3DAccessor > AMCLDepthCamera::creator_( *(new IRON::NDTLiteConfig()) );
IRON::IRONEnginef AMCLDepthCamera::engine_( *(new IRON::IRONConfig()) );
Map3dCloud* AMCLDepthCamera::map3d_(new  Map3dCloud());
double AMCLDepthCamera::currmap_downsam_res_(0.03);
IRON::NDTLiteConfig AMCLDepthCamera::ndtcfg_( *(new IRON::NDTLiteConfig()) );
IRON::IRONConfig AMCLDepthCamera::ironcfg_( *(new IRON::IRONConfig()) );
}

#endif
