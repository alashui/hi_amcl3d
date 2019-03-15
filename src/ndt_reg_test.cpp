#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <fstream>
//#include <Eigen/core>
#include <Eigen/Geometry>

#include "../map/IRON.h"

#include <string>
using namespace std;

// OpenCV 库
#include <pcl_ros/point_cloud.h>
#include <iostream>
#include <fstream>
#include <string>  
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/io.h>
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

using namespace IRON;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 


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
        //float ttt;
        //ifs >> ttt;
        if (ifs.eof()) break;
        cloud.push_back(pts);
    }
    ifs.close();
    sensorPose = Eigen::Translation3f(x, y, z) * Eigen::Quaternionf(qw, qx, qy, qz);
}


///////////////////////////////
// store ndt mean points
///////////////////////////////
void storeMeans(const char *file, const NDTMapLitef &map,
                const Eigen::Affine3f &transform = Eigen::Affine3f::Identity())
{
    std::ofstream of;
    of.open(file, std::ios::out);
    for (uint i = 0; i < map.cells.size(); ++i)
    {
        const Eigen::Vector3f res = transform * map.cells[i].mu;
        //of << res(0) << " " << res(1) << " " << res(2) << std::endl;
        of << res(0) << " " << res(2) << " " << -res(1) << std::endl;
    }
    of.close();
}
void storeMeans(const char *file, const IRONDescriptorVectorf &vec)
{
    std::ofstream of;
    of.open(file, std::ios::out);
    for (uint i = 0; i < vec.size(); ++i)
    {
        //of << vec[i].mu()(0) << " " << vec[i].mu()(1) << " " << vec[i].mu()(2) << std::endl;
        of << vec[i].mu()(0) << " " << vec[i].mu()(2) << " " << -vec[i].mu()(1) << std::endl;
    }
    of.close();
}


///////////////////////////////
// store cov maps
///////////////////////////////
void storeCovs(const char *file, const NDTMapLitef &map, uint samples = 20u)
{
    std::ofstream of;
    of.open(file, std::ios::out);
    XorShift rnd;
    for (uint i = 0; i < map.cells.size(); ++i)
    {
        const Eigen::Matrix<float, 3, 3> &cov = map.cells[i].cov;
        const Eigen::Matrix<float, 3, 3> L = cov.llt().matrixL();
        for (uint k = 0; k < samples; ++k)
        {
            float r1 = (rnd.rand() % 100000) / 100000.0,
                  r2 = (rnd.rand() % 100000) / 100000.0;
            float th = 2.0 * M_PI * r1,
                  ph = std::acos(2.0 * r2 - 1.0);
            float  x = std::cos(th) * std::sin(ph),
                   y = std::sin(th) * std::sin(ph),
                   z = std::cos(ph);
            Eigen::Vector3f pt(x, y, z), res;
            res = 1.0 * L * pt;
            
            
            //of << map.cells[i].mu(0) + res(0) << " "
            //   << map.cells[i].mu(1) + res(1) << " "
            //   << map.cells[i].mu(2) + res(2) << "\n";
            of << map.cells[i].mu(0) + res(0) << " "
               << map.cells[i].mu(2) + res(2) << " "
               << map.cells[i].mu(1) + res(1) << "\n";
        }
    }
    of.close();
}
void storeCovs(const char *file, const IRONDescriptorVectorf &vec, uint samples = 20u)
{
    std::ofstream of;
    of.open(file, std::ios::out);
    XorShift rnd;
    for (uint i = 0; i < vec.size(); ++i)
    {
        const Eigen::Matrix<float, 3, 3> &cov = vec[i].cov();
        const Eigen::Matrix<float, 3, 3> L = cov.llt().matrixL();
        for (uint k = 0; k < samples; ++k)
        {
            float r1 = (rnd.rand() % 100000) / 100000.0,
                  r2 = (rnd.rand() % 100000) / 100000.0;
            float th = 2.0 * M_PI * r1,
                  ph = std::acos(2.0 * r2 - 1.0);
            float  x = std::cos(th) * std::sin(ph),
                   y = std::sin(th) * std::sin(ph),
                   z = std::cos(ph);
            Eigen::Vector3f pt(x, y, z), res;
            res = 1.0 * L * pt;
            
            
            //of << vec[i].mu()(0) + res(0) << " "
            //   << vec[i].mu()(1) + res(1) << " "
            //   << vec[i].mu()(2) + res(2) << "\n";
        	of << vec[i].mu()(0) + res(0) << " "
               << vec[i].mu()(2) + res(2) << " "
               << vec[i].mu()(1) + res(1) << "\n";
        
        }
    }
    of.close();
}


///////////////////////////////
// store matches
///////////////////////////////
void storeMatches(const char *file, const IRONMatchVectorf &matches)
{
    std::ofstream of;
    of.open(file, std::ios::out);
    for (uint i = 0; i < matches.size(); ++i)
    {
        //of << matches[i].from->mu()(0) << " " << matches[i].from->mu()(1) << " " << matches[i].from->mu()(2) << std::endl;
        //of << matches[i].to->mu()(0) << " " << matches[i].to->mu()(1) << " " << matches[i].to->mu()(2) << std::endl;
        of << matches[i].from->mu()(0) << " " << matches[i].from->mu()(2) << " " << -matches[i].from->mu()(1) << std::endl;
        of << matches[i].to->mu()(0) << " " << matches[i].to->mu()(2) << " " << -matches[i].to->mu()(1) << std::endl;
        of << "\n\n";
    }
    of.close();
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

struct timespec t1;
struct timespec t2;
clock_gettime(CLOCK_REALTIME, &t1); 


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
    ndtcfg.subsamplingFactor =0.9f;// 0.2f; // default: take only 20% of points (drawn randomly)
    ndtcfg.clippingDistance = 4.5f;  // limit memory consumption of NDTLiteCreator by
                                       // throwing away points that are far away from the sensor
    NDTMapLiteCreator<float, std::vector<Pt3D>, Pt3DAccessor> creator(ndtcfg);

    // prepare IRON engine (IRON.h for a summary of available options)
    IRONConfig ironcfg;
    // important variables
    ironcfg.matchingTolerance = 0.05f;   // RANSAC inlier threshold: higher values will make registration more robust
                                      // but the transform computation more inaccurate; default: half of NDT-cell size
    ironcfg.entropyThreshold = 0.55f;    // the lower, the more keypoints will be found and the slower registration
    						   
    ironcfg.neighborSearchRadius = 0.5f; // radius around each NDT-cell for searching neighboring cells
    								
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
    
    
clock_gettime(CLOCK_REALTIME, &t2); 
double td1 = (t2.tv_sec-t1.tv_sec)+(t2.tv_nsec-t1.tv_nsec)/1000000000.0;
std::cerr<<"匹配耗费时间： "<<td1 <<endl;


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



string pcd1_dir=argv[1];
PointCloud::Ptr pcld1( new PointCloud );
pcl::io::loadPCDFile(pcd1_dir, *pcld1);
PointCloud::Ptr pcld12( new PointCloud );
pcl::transformPointCloud( *pcld1, *pcld12,result.tf.matrix());
pcl::io::savePCDFile("pcld1.pcd", *pcld12); 


string pcd2_dir=argv[2];
PointCloud::Ptr pcld2( new PointCloud );
pcl::io::loadPCDFile(pcd2_dir, *pcld2);

PointCloud::Ptr pcld22( new PointCloud );
pcl::transformPointCloud( *pcld2, *pcld22,sensorPose2.matrix());
pcl::io::savePCDFile("pcld2.pcd", *pcld22);


    return EXIT_SUCCESS;

}




