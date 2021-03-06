/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <fstream>
//#include <Eigen/core>
#include <Eigen/Geometry>

#include "amcl_depthcamera.h"
#include "../map/IRON.h"

using namespace amcl;


////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLDepthCamera::AMCLDepthCamera(): AMCLSensor()

                                    
                                    //creator_(ndtcfg_),
                                    //engine_(ironcfg_)
{
    /*
    
    //time = 0.0;

    // prepare NDT map creator (take a look into NDTLite.h for an explanation of parameters)
    //NDTLiteConfig ndtcfg;
    // some important variables
    ndtcfg_.cellSize = 0.2f;          // default: 0.1m x 0.1m x 0.1m cells
    ndtcfg_.subsamplingFactor =0.9f;// 0.2f; // default: take only 20% of points (drawn randomly)
    ndtcfg_.clippingDistance = 5.0f;  // limit memory consumption of NDTLiteCreator by
                                        // throwing away points that are far away from the sensor
    //std::vector<PointT, Eigen::aligned_allocator<PointT> > points;                                    
    //NDTMapLiteCreator<float, std::vector<PointT>, Point3DAccessor> creator(ndtcfg);
    //creator_.setConfig(ndtcfg_);
 
    // prepare IRON engine (IRON.h for a summary of available options)
    //IRONConfig ironcfg;
    // important variables
    ironcfg_.matchingTolerance = //0.05f;   // RANSAC inlier threshold: higher values will make registration more robust
                                0.2f;       // but the transform computation more inaccurate; default: half of NDT-cell size
    ironcfg_.entropyThreshold = //0.55f;    // the lower, the more keypoints will be found and the slower registration
                                0.8f;
    ironcfg_.neighborSearchRadius = //0.5f; // radius around each NDT-cell for searching neighboring cells
                                    0.7f;
    //IRONEnginef engine(ironcfg);
    //engine_.setConfig(ironcfg_);

    return;
    */
}

AMCLDepthCamera::~AMCLDepthCamera()
{

}
void AMCLDepthCamera::SetNDTMatchParam( const float& ndt_cellSize,
                                        const float& ndt_subsamplingFactor,
                                        const float& ndt_clippingDistance,
                                        const float& iron_matchingTolerance,
                                        const float& iron_entropyThreshold,
                                        const float& iron_neighborSearchRadius,
                                        const std::string& global_map3d_dir,
                                        const bool flag_global_process,   
                                        const double &globalmap_downsam_res,
                                        const double &currmap_downsam_res
                                        )
{
    ndtcfg_.cellSize = ndt_cellSize;         
    ndtcfg_.subsamplingFactor = ndt_subsamplingFactor;
    ndtcfg_.clippingDistance =  ndt_clippingDistance; 
    creator_.setConfig(ndtcfg_);

    ironcfg_.matchingTolerance = iron_matchingTolerance;
    ironcfg_.entropyThreshold = iron_entropyThreshold;
    ironcfg_.neighborSearchRadius = iron_neighborSearchRadius;
    engine_.setConfig(ironcfg_);

    if(flag_global_process)
        map3d_->load_gloabalmap(global_map3d_dir,globalmap_downsam_res);
    else 
        map3d_->load_gloabalmap_no_process(global_map3d_dir,globalmap_downsam_res);

    //globalmap_downsam_res_(globalmap_downsam_res);
    currmap_downsam_res_=currmap_downsam_res;
}
////////////////////////////////////////////////////////////////////////////////
// Apply the DepthCamera sensor model
bool AMCLDepthCamera::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{

  pf_update_sensor(pf, (pf_sensor_model_fn_t) NDTmatchModel, data);
  return true;
}

double AMCLDepthCamera::NDTmatchModel(AMCLDepthCameraData *data, pf_sample_set_t* set)
{

ofstream fout("/home/robot/aa_pcd_amcl3d/debug_output.txt");

    // 下采样观测点云数据
	PointCloud::Ptr data_pcloud_filtered( new PointCloud );   
    //double downsample_resolution = 0.03;
    boost::shared_ptr<pcl::VoxelGrid<PointT> > voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(currmap_downsam_res_, currmap_downsam_res_, currmap_downsam_res_);
    voxelgrid->setInputCloud(data->pcloud);

    voxelgrid->filter(*data_pcloud_filtered);
pcl::io::savePCDFileASCII("/home/robot/aa_pcd_amcl3d/map_cur.pcd" , *data_pcloud_filtered);



  AMCLDepthCamera *self;
  //double p(0.0);
  double total_weight(0);
  pf_sample_t *sample;
  pf_vector_t pose;

  self = (AMCLDepthCamera*) data->sensor;

  total_weight = 0.0;

  // Compute the sample weights

  std::cout<< "particles number: " << set->sample_count << endl;


fout<< "particles number: " << set->sample_count << endl<<endl;
struct timespec tpstart;
struct timespec tpend;	

//printf("clock_gettime time: %.12f\n", timedif);

double max_weight(0);
uint max_index(0);
  for (uint j = 0; j < set->sample_count; j++)
  {

clock_gettime(CLOCK_REALTIME, &tpstart); 

    sample = set->samples + j;
    pose = sample->pose;

    //将pose 变换为pose_eigen;
    Eigen::AngleAxisd rotation_vector (pose.v[2],Eigen::Vector3d(0,0,1));
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    pose_eigen.rotate(rotation_vector);
    pose_eigen.pretranslate( Eigen::Vector3d(pose.v[0],pose.v[1],0) );

    // Take account of the laser pose relative to the robot
    //这里的laser_pose为激光传感器相对于base_frame的位置，根据粒子表示的机器人位姿可将激光传感器变换到全局坐标

    //pose = pf_vector_coord_add(self->camera_pose_, pose);//变换后的得到激光传感器的全局位姿
    Eigen::Isometry3d camera_pose_w_( pose_eigen * self->camera_pose_ );//变换后的得到深度相机的全局位姿
//Eigen::Isometry3d camera_pose_w_( pose_eigen);


    //*******计算粒子权值*********//
struct timespec t1;
struct timespec t2;	
struct timespec t4;

uint desc1_size(0),desc2_size(0),matches_size(0),inlier_size(0);


clock_gettime(CLOCK_REALTIME, &t1); 

    self->map3d_->generate_refmap(camera_pose_w_);//根据粒子位姿生成期望观测；

clock_gettime(CLOCK_REALTIME, &t2); 


double td1 = (t2.tv_sec-t1.tv_sec)+(t2.tv_nsec-t1.tv_nsec)/1000000000.0;



    double dist_2(0);
    double angle_2(0);

    if(self->map3d_->ref_map_->points.size()==0)
    {
       sample->weight=0; 
    }
    else
    {

    
pcl::io::savePCDFileASCII("/home/robot/aa_pcd_amcl3d/map_ref"+std::to_string(j)+".pcd" , *self->map3d_->ref_map_);

        //匹配观测点云 *(data->pcloud)  和  期望观测点云 *(self->map3d_->ref_map_) 
        // container for point clouds,
        // sensor poses, ndt maps, descriptors and matches
        //std::vector<Pt3D> cloud1,cloud2;
clock_gettime(CLOCK_REALTIME, &t4);

        Eigen::Affine3f sensorPose1=Eigen::Affine3f::Identity();
        Eigen::Affine3f sensorPose2=Eigen::Affine3f::Identity();
        IRON::NDTMapLitef ndtMap1,ndtMap2;
        IRON::IRONDescriptorVectorf descriptors1,descriptors2;
        IRON::IRONMatchVectorf matches,inlierset;

        // PLEASE NOTE: NDTMapLiteCreator and IRONEngine should exist only once, as
        // they do some additional computation during construction which would lead to
        // unnecessary overhead if created in a loop over and over again

        // build NDT maps
        self->creator_.createMapFromPointValues(&ndtMap1, self->map3d_->ref_map_->points, sensorPose1);
        self->creator_.createMapFromPointValues(&ndtMap2, data_pcloud_filtered->points, sensorPose2);

        // compute IRON keypoints and descriptors
        self->engine_.computeDescriptors(&descriptors1, ndtMap1);
        self->engine_.computeDescriptors(&descriptors2, ndtMap2);

        //cout<< "KEYPOINTS 1: " << descriptors1.size() << " cells\n"
        //    << "KEYPOINTS 2: " << descriptors2.size() << " cells\n";

        // match keypoint descriptors
        self->engine_.computeMatches(&matches, descriptors1, descriptors2);

        //cout<< "MATCHES....: " << matches.size() << std::endl;

        // reject outliers
        self->engine_.detectOutliers(&inlierset, matches);
        //cout << "INLIERS....: " << inlierset.size() << std::endl;

        // compute transform from inlierset
        IRON::IRONTransformResultf result = engine_.computeTransform(inlierset);
        double pose_x= result.tf.matrix()(0,3);
		double pose_y= result.tf.matrix()(1,3);
        double pose_z= result.tf.matrix()(2,3);
        dist_2=	pose_x*pose_x + pose_y*pose_y +pose_z*pose_z;		
		double pose_theta= acos(0.5*((result.tf.matrix()(0,0)+result.tf.matrix()(1,1)+result.tf.matrix()(2,2))-1)) ;
        angle_2=pose_theta*pose_theta;
        //std::cout << "result:  "<< std::endl << result.tf.matrix() << std::endl;
        uint inlier_size_(inlierset.size() );  
/*          
        for (uint i = 0; i < inlier_size_; ++i)
        {
            /*
            p += exp( (double)-1.0 * ((double)1.0/inlier_size_)*
                                ( pow( inlierset[i].from->mu()(0)-inlierset[i].to->mu()(0) , 2 ) +
                                pow( inlierset[i].from->mu()(1)-inlierset[i].to->mu()(1) , 2 ) 
                               //+ pow( inlierset[i].from->mu()(2)-inlierset[i].to->mu()(2) , 2 )
                                ) 
                    );
            *
            sum_matches_diff += ( pow( inlierset[i].from->mu()(0)-inlierset[i].to->mu()(0) , 2 ) +
                                  pow( inlierset[i].from->mu()(1)-inlierset[i].to->mu()(1) , 2 )   );
        }
*/
        if(inlier_size_>=1)
        {
/*
            sum_matches_diff /= inlier_size_;
            if(sum_matches_diff<0.01)sum_matches_diff=0.01;
            sample->weight *=(  inlier_size_<10 ? 0:
                                exp(   (double)-1.0 *
                                       ( 200.0/((double)inlier_size_*3.0+1.0/sum_matches_diff) )   
                                   )
                                
                             );
*/             



            sample->weight *=( //(dist_2>4||angle_2>2.5) ?  0.0001 :
                                 (   30/( 1.0+10.0/inlier_size_ )
                                   + exp(   (double)-1.0*(0.1+dist_2 +angle_2))
                                 ) 
                             );
                             
        }
        else
        {
            sample->weight *=((dist_2>4||angle_2>2.5||inlier_size_<=5) ?  0.0001 : exp(   (double)-1.0*(0.01+dist_2 +angle_2)));

        }

        //sample->weight *= (inlier_size_<3 ? 0:p*(double)1.0/inlier_size_);
        total_weight += sample->weight;


desc1_size=descriptors1.size();
desc2_size=descriptors2.size();
matches_size=matches.size();
inlier_size=inlierset.size();

        self->engine_.releaseDescriptors(&descriptors1);
        self->engine_.releaseDescriptors(&descriptors2);
    }

if(sample->weight>max_weight)
{
    max_index=j;
    max_weight=sample->weight;
}

clock_gettime(CLOCK_REALTIME, &tpend);
double timedif = (tpend.tv_sec-tpstart.tv_sec)+(tpend.tv_nsec-tpstart.tv_nsec)/1000000000.0;
double td4 = (tpend.tv_sec-t4.tv_sec)+(tpend.tv_nsec-t4.tv_nsec)/1000000000.0;

cout<<"第"<<j<<"个粒子权值： "<<sample->weight<<endl;

fout<<"第"<<j<<"个粒子: "<<endl;
fout<<"生期望观测时间： "<<td1 <<endl;

fout<<"cloud   size: "<<data->pcloud->points.size()<<endl;
fout<<"filteredsize: "<<data_pcloud_filtered->points.size()<<endl;
fout<<"refmap  size: "<<self->map3d_->ref_map_->points.size()<<endl;

fout<<"refmap  KEYPOINTS: " << desc1_size << endl
    <<"currmap KEYPOINTS: " << desc2_size << endl;
fout<<"MATCHES.... : " << matches_size << endl;
fout<<"INLIERS.... : " << inlier_size << endl;
fout<<"dist and angle : "<<dist_2<<"  "<<angle_2<<endl;

fout<<"粒子权值    : "<<sample->weight<<endl;

fout<<"匹配耗费的时间： "<<td4 <<endl;
fout<<"权值计算总时间： "<<timedif <<endl;

fout<<"robot pose  : "<<pose.v[0]<<" "<<pose.v[1]<<" "<<pose.v[2]*(180.0/M_PI)<<endl<<endl;


  }


cout<<"最大权值： "<< max_weight<<endl;
cout<<"位姿： "<<(set->samples + max_index)->pose.v[0]<<" "
              <<(set->samples + max_index)->pose.v[1]<<" "
              <<(set->samples + max_index)->pose.v[2]*(180.0/M_PI)<<endl;
fout<<"最大权值： "<< max_weight<<endl;
fout<<"位姿： "<<(set->samples + max_index)->pose.v[0]<<" "
              <<(set->samples + max_index)->pose.v[1]<<" "
              <<(set->samples + max_index)->pose.v[2]*(180.0/M_PI)<<endl;
  return(total_weight);
}

















/*
 * IRON Reference Implementation:
 * *************************************************************************************************************
 * Schmiedel, Th., Einhorn, E., Gross, H.-M.
 * IRON: A Fast Interest Point Descriptor for Robust NDT-Map Matching and Its Application to Robot Localization.
 * IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS), Hamburg, Germany, 2015
 * *************************************************************************************************************
 * Copyright (C) 2015, Thomas Schmiedel (thomas.schmiedel@tu-ilmenau.de)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

/*
///////////////////////////////
// INCLUDES
///////////////////////////////
#include <fstream>
#include "IRON.h"


using namespace IRON;


///////////////////////////////
// POINTS AND ACCESSOR:
///////////////////////////////
//
// This is an example data structure which describes a simple 3D point.
// You can, however, use any point type you like, as long as an accessor
// is provided (see below)
//
// In many cases this will prevent expensive conversion between point cloud types
//
struct Pt3D
{
    Pt3D() { }

    Pt3D(float x, float y, float z)
    {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    float data[3];

    float &x()
    {
        return data[0];
    }

    float &y()
    {
        return data[1];
    }

    float &z()
    {
        return data[2];
    }
};


//
// Whatever point type is provided, this short accessor is needed
// to return const references to x, y, z; so if you cannot modify a
// given point type, you can still use it for NDT-map creation
//
struct Pt3DAccessor
{
    static const float &x(const Pt3D &s)
    {
        return s.data[0];
    }
    static const float &y(const Pt3D &s)
    {
        return s.data[1];
    }
    static const float &z(const Pt3D &s)
    {
        return s.data[2];
    }
};


//
// load ascii pcd file,
// this wont work with
// binary pcd files
// those are just some helper
// functions for demonstration
// purposes
//
void fillCloud(const char *file, std::vector<Pt3D> &cloud, Eigen::Affine3f &sensorPose)
{
    cloud.clear();
    std::ifstream ifs;
    ifs.open(file, std::ios::in);
    if (ifs.fail())
    {
        std::cerr << "file not found!\n";
        exit(EXIT_FAILURE);
    }
    std::string l;
    float x, y, z, qx, qy, qz, qw;
    for (uint i = 0; i < 8u; ++i) { std::getline(ifs, l); }
    ifs >> l >> x >> y >> z >> qw >> qx >> qy >> qz;//第九行为相机位姿信息
    for (uint i = 0; i < 3u; ++i) { std::getline(ifs, l); }
    while (true)
    {
        Pt3D pts;
        ifs >> pts.x();
        ifs >> pts.y();
        ifs >> pts.z();
        float ttt;
        ifs >> ttt;
        if (ifs.eof()) break;
        cloud.push_back(pts);
    }
    ifs.close();
    sensorPose = Eigen::Translation3f(x, y, z) * Eigen::Quaternionf(qw, qx, qy, qz);
}


///////////////////////////////
///////////////////////////////
//
//           MAIN
//
///////////////////////////////
///////////////////////////////
int main(int argc, char **argv)
{
    // simple check
    if (argc != 3)
    {
        std::cerr << "Usage:\n"
                  << "       ./demo <example1.pcd> <example2.pcd>\n";
        return EXIT_FAILURE;
    }

    // container for point clouds,
    // sensor poses, ndt maps, descriptors and matches
    std::vector<Pt3D> cloud1,
                      cloud2;
    Eigen::Affine3f sensorPose1,
                    sensorPose2;
    NDTMapLitef ndtMap1,
                ndtMap2;
    IRONDescriptorVectorf descriptors1,
                          descriptors2;
    IRONMatchVectorf matches,
                     inlierset;

    // read pcd files
    // this will load ascii pcl point clouds only,
    // please make sure they have the following structure:
    // (since this is just a demo application,
    // we felt such a simple map loader is sufficient)
    //
    // # .PCD v.7 - Point Cloud Data file format
    // VERSION .7
    // FIELDS x y z
    // SIZE 4 4 4
    // TYPE F F F
    // COUNT 1 1 1
    // WIDTH <number of points>
    // HEIGHT 1
    // VIEWPOINT <x y z qw qx qy qz>
    // POINTS <number of points>
    // DATA ascii
    // -1.54531 -1.10236 2.7652
    // -1.52864 -1.09423 2.7448
    // ...
    //
    // Please note:
    // - One line corresponds to one point (x, y, z)
    // - All point values are expected to be given w.r.t. the sensor pose at (0, 0, 0)
    // - VIEWPOINT must be set to the sensor pose (x, y, z, qw, qx, qy, qz) -> Rotation as Quaternion
    //
    fillCloud(argv[1], cloud1, sensorPose1);
    fillCloud(argv[2], cloud2, sensorPose2);

	std::cerr<<"size12: "<< cloud1.size() <<" "<< cloud2.size()<<std::endl;
    // now shift the second map a bit in x direction (this will shift the new NDT-map as well)
    sensorPose2.translation().x() += 4.0f;

    // prepare NDT map creator (take a look into NDTLite.h for an explanation of parameters)
    NDTLiteConfig ndtcfg;
    // some important variables
    ndtcfg.cellSize = 0.1f;          // default: 0.1m x 0.1m x 0.1m cells
    ndtcfg.subsamplingFactor =0.8f;// 0.2f; // default: take only 20% of points (drawn randomly)
    ndtcfg.clippingDistance = 5.0f;  // limit memory consumption of NDTLiteCreator by
                                       // throwing away points that are far away from the sensor
    NDTMapLiteCreator<float, std::vector<Pt3D>, Pt3DAccessor> creator(ndtcfg);

    // prepare IRON engine (IRON.h for a summary of available options)
    IRONConfig ironcfg;
    // important variables
    ironcfg.matchingTolerance = //0.05f;   // RANSAC inlier threshold: higher values will make registration more robust
                               0.2f;       // but the transform computation more inaccurate; default: half of NDT-cell size
    ironcfg.entropyThreshold = //0.55f;    // the lower, the more keypoints will be found and the slower registration
    						   0.75f;
    ironcfg.neighborSearchRadius = //0.5f; // radius around each NDT-cell for searching neighboring cells
    								0.8f;
    IRONEnginef engine(ironcfg);

    // PLEASE NOTE: NDTMapLiteCreator and IRONEngine should exist only once, as
    // they do some additional computation during construction which would lead to
    // unnecessary overhead if created in a loop over and over again

    // build NDT maps
    creator.createMapFromPointValues(&ndtMap1, cloud1, sensorPose1);
    creator.createMapFromPointValues(&ndtMap2, cloud2, sensorPose2);
    std::cerr << "NDT MAP 1..: " << ndtMap1.cells.size() << " cells\n"
              << "NDT MAP 2..: " << ndtMap2.cells.size() << " cells\n";

    // compute IRON keypoints and descriptors
    engine.computeDescriptors(&descriptors1, ndtMap1);
    engine.computeDescriptors(&descriptors2, ndtMap2);
    std::cerr << "KEYPOINTS 1: " << descriptors1.size() << " cells\n"
              << "KEYPOINTS 2: " << descriptors2.size() << " cells\n";

    // match keypoint descriptors
    engine.computeMatches(&matches, descriptors1, descriptors2);
    std::cerr << "MATCHES....: " << matches.size() << std::endl;

    // reject outliers
    engine.detectOutliers(&inlierset, matches);
    std::cerr << "INLIERS....: " << inlierset.size() << std::endl;

    // compute transform from inlierset
    IRONTransformResultf result = engine.computeTransform(inlierset);
    std::cerr << "result:  "<< std::endl << result.tf.matrix() << std::endl;
    if (!result.success)
    {
        std::cerr << "REGISTRATION FAILED\n";
        engine.releaseDescriptors(&descriptors1);
        engine.releaseDescriptors(&descriptors2);
        return EXIT_FAILURE;
    }

    // save results in order to be plotted
    storeMeans("means1.txt", ndtMap1);
    storeMeans("means2.txt", ndtMap2);
    storeMeans("means_keypoints1.txt", descriptors1);
    storeMeans("means_keypoints2.txt", descriptors2);
    storeMatches("matches.txt", matches);
    storeMatches("inlierset.txt", inlierset);
    storeCovs("covs1.txt", ndtMap1);
    storeCovs("covs2.txt", ndtMap2);
    storeCovs("covs_keypoints1.txt", descriptors1);
    storeCovs("covs_keypoints2.txt", descriptors2);
    storeMeans("means_transformed1.txt", ndtMap1, result.tf);

    // clean up descriptors, they won't take care of allocated memory by themselves
    engine.releaseDescriptors(&descriptors1);
    engine.releaseDescriptors(&descriptors2);

    std::cerr << "done.\n"
              << "run: ./plot_means_keypoints.sh to plot both maps and final matches\n"
              << "run: ./plot_result.sh to plot both maps after alignment\n"
              << "run: ./plot_covs_keypoints.sh to view map covariances\n"
              << "(please note: you will need gnuplot-x11 for this to work)\n";

    return EXIT_SUCCESS;
}*/
