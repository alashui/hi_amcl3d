///////////////////////////////////////////////////////////////////////////
//
// Desc: 3d point cloud model for AMCL
// Author: Lailihai
// Date: 2018.6
//
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_3DPCLD_H
#define AMCL_3DPCLD_H

#include "amcl_sensor.h"
#include "../map/map.h"

namespace amcl
{

// Laser sensor data
class AMCL3DPCloudData : public AMCLSensorData
{
  public:
    AMCL3DPCloudData () {ranges=NULL;};
    virtual ~AMCL3DPCloudData() {delete [] ranges;};

    PointCloud::Ptr pcloud;//当前观测的局部地图点云

};


// Laseretric sensor model
class AMCL3DPCloud : public AMCLSensor
{
  // Default constructor
  public: AMCL3DPCloud(size_t max_beams, map_t* map);

  public: virtual ~AMCL3DPCloud(); 


  private: 
    int parameters;//在这里定义点云配准相关的参数

};


}

#endif
