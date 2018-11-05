#include "map3d.h"
#include <boost/timer.hpp>

//using namespace amcl;		
Map3dCloud::Map3dCloud(ros::NodeHandle nh):nh_(nh),
									   global_map_(new PointCloud()),
									   ref_map_(new PointCloud())
									   
{
    global_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
    //curr_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/currmap", 5, true);
    //ref_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/refmap", 5, true);	
}
//void Map3dCloud::localization(){}
void Map3dCloud::load_gloabalmap(const string &globalmap_pcd_dir)
{
    cout<<"load globalmap..."<<endl;
    pcl::io::loadPCDFile(globalmap_pcd_dir, *global_map_);
    global_map_->header.frame_id = "map";
    // downsample globalmap
    double downsample_resolution = private_nh_.param<double>("downsample_resolution", 0.03);
    boost::shared_ptr<pcl::VoxelGrid<PointT> > voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(global_map_);

    //PointCloud::Ptr filtered(new PointCloud());
    voxelgrid->filter(*global_map_);
cout<<"...1234"<<endl;
    //*global_map_ = *filtered;
   
	sensor_msgs::PointCloud2 msg_global_map_;
	pcl::toROSMsg(*global_map_, msg_global_map_);
    global_map_pub_.publish(msg_global_map_);
    ros::spinOnce();
    ros::Duration(0.1).sleep();	
    
    cout<<"全局点云共有"<< global_map_->size()<<"个点."<<endl;
}


void Map3dCloud::generate_refmap(const Eigen::Isometry3d &cameraPose)
{
	cout<<"generate refmap..."<<endl;
	PointCloud::Ptr newCloud( new PointCloud );
	//pcl::transformPointCloud( *curr_map_, *newCloud,cameraPose.matrix() );//将观测点云变换到全局坐标
	  //*newCloud = *curr_map_;
	
	//cout<<"cameraPose : "<<endl<<cameraPose.matrix()<<endl;

	PointCloud::Ptr global_map_c( new PointCloud );
	pcl::transformPointCloud( *global_map_, *global_map_c,cameraPose.inverse().matrix() );
	//PointCloud::Ptr tempCloud2(new PointCloud);
	for (int i=0; i<global_map_c->points.size();i++)
	{
		PointT pt;
		pt.x=global_map_c->points[i].x;
		pt.y=global_map_c->points[i].y;
		pt.z=global_map_c->points[i].z;
		
		if(pt.z<4.0 && pt.z>0.8)
		{
			if(    pt.x > (-0.6*pt.z) && pt.x< (0.6*pt.z) 
				&& pt.y > (-0.5*pt.z) && pt.y< (0.5*pt.z)     
			  )
			{
				//pt.b=global_map_c->points[i].b;
				//pt.g=global_map_c->points[i].g;
				//pt.r=global_map_c->points[i].r;
				
				ref_map_->points.push_back(pt);
			}
		}
		
	}
	ref_map_->height = 1;
	ref_map_->width = ref_map_->points.size();

	//ref_map_->is_dense = false;
	
    ref_map_->is_dense = false;

	/*
    cout<<"参考点云共有"<<ref_map_->size()<<"个点."<<endl;
	pcl::io::savePCDFileASCII("/home/robot/catkin_ws/data/map_ref.pcd" , *ref_map_); 

	//发布消息（在rviz中显示）
	PointCloud::Ptr ref_map_w( new PointCloud );
	pcl::transformPointCloud( *ref_map_, *ref_map_w,cameraPose.matrix() );
	ref_map_w->header.frame_id = "map";	
	sensor_msgs::PointCloud2 msg_ref_map_;
	pcl::toROSMsg(*ref_map_w, msg_ref_map_);
    ref_map_pub_.publish(msg_ref_map_);
    ros::spinOnce();
    ros::Duration(0.1).sleep(); 
	
	*/  
}

/*
void Map3dCloud::set_currmap(const PointCloud::Ptr currCloud,const Eigen::Isometry3d &cameraPose)
{
	cout<<"set currmap..."<<endl;
	*curr_map_=*currCloud;


    
	cout<<"当前点云共有"<< curr_map_->size()<<"个点."<<endl;
	curr_map_->is_dense = false;
    pcl::io::savePCDFileASCII("/home/robot/catkin_ws/data/map_curr.pcd" , *curr_map_); 
    
    //发布消息（在rviz中显示）	
	Eigen::Isometry3d cameraPose_zplus(cameraPose);
	cameraPose_zplus.translation().y() -=3.0f;//y轴平移５米便于区分显示
	
	PointCloud::Ptr curr_map_w( new PointCloud );
	pcl::transformPointCloud( *curr_map_, *curr_map_w,cameraPose_zplus.matrix() );
	
    curr_map_w->header.frame_id = "map";
	sensor_msgs::PointCloud2 msg_curr_map_;
	pcl::toROSMsg(*curr_map_w, msg_curr_map_);
    curr_map_pub_.publish(msg_curr_map_);
    ros::spinOnce();
    ros::Duration(0.1).sleep();   	
}


bool readimgfromfile(const string &image_database_dir,const int &index,
					 cv::Mat &colorImg,cv::Mat &depthImg,
					 Eigen::Isometry3d &cameraPose)
{
	//string image_database_dir(argv[1]);
	//string image_database_dir ( "/home/robot/a_myfile/image_data2/follower_image_map226" );
	string dir_result_after_g2o( image_database_dir + "/result_after.g2o" );

	ifstream fin(dir_result_after_g2o);	 
	if (!fin)								
	{
		cerr<<"cannot find result_after.g2o file"<<endl;
		return false;
	}

	string temp;	
	int num_line=0;
	//int frame_num=0;
	cout<<"file (rgb and depth) reading..."<<endl;
	while(getline(fin,temp))		//获取一行,一行数据格式为  VERTEX_SE3:QUAT 1   0 0 0 0 0 0 1  
									//八位数字分别表示 x y z qx qy qz qw(前三位为位置，后四位为四元数表示的旋转角)
	{	
		//对每一行数据的读入
		num_line++;
		if(num_line > index)
		{
			cerr<<"index invalid: index less to zero!"<<endl;
			return false;
		}
		else if(num_line < index)continue;
		 
		else if (num_line==2)
		{
			return false;	//忽略第二行 数据为 FIX 1
			cerr<<"index invalid: index=2!"<<endl;
		}
		string temp_str;  //忽略每行前面的字符串 VERTEX_SE3:QUAT
		vector<double> double_vec; 
		double num;
		istringstream iss(temp);
		iss >> temp_str;
		
		
		if (temp_str=="VERTEX_SE3:QUAT")   	
		{	
			while(iss >> num)  		//分别将这一行数据读入						
				double_vec.push_back(num);
			
			//Frame::Ptr frame(new Frame());
			int frame_id = double_vec[0];
			//frame_ids.insert(make_pair(frame_id,frame_num++));
			string rgb_dir = image_database_dir+"/rgb/rgb"+to_string(frame_id)+".png";
			string depth_dir = image_database_dir+"/depth/depth"+to_string(frame_id)+".png";     
			colorImg= cv::imread(rgb_dir) ;
	   		depthImg= cv::imread(depth_dir,-1) ;
	   		
			//将x y z q1 q2 q3 q4 转换为sophus::se3 表示
			Eigen::Vector3d t(double_vec[1],double_vec[2],double_vec[3]);
			Eigen::Quaterniond q(double_vec[7],double_vec[4],double_vec[5],double_vec[6]);
											//Eigen::Quaterniond里使用顺序qw qx qy qz 				
			Eigen::Isometry3d T(q);
			//cameraPose.rotate(q)
			T.pretranslate(t);
			cameraPose=T;

			cout<<"read file rgb"<<frame_id<<"  depth"<<frame_id<<endl;
			return true;
		}
		else if(temp_str== "EDGE_SE3:QUAT" )
		{
			cerr<<"index invalid:index exceeded normal range!"<<endl;
			return false;
		}
	}
}
void image2cloud(const cv::Mat &color,const cv::Mat &depth,PointCloud::Ptr outputCloud,bool downsample_flag)
{
    // 相机内参 
    static const double cx = 319.5;
    static const double cy = 239.5;
    static const double fx = 525.0;
    static const double fy = 525.0;
    static const double depthScale = 1000.0;
        
    PointCloud::Ptr pointCloud_temp( new PointCloud ); 
    for ( int v=0; v<color.rows; v++ )
        for ( int u=0; u<color.cols; u++ )
        {
            unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
            if ( d==0 ) continue; // 为0表示没有测量到
      		if (d>=6000) continue;
           
            PointT p ;
            p.z = double(d)/depthScale;//pointWorld[2];
            p.x = (u-cx)*p.z/fx;//pointWorld[0];
            p.y = (v-cy)*p.z/fy;//pointWorld[1];
            
            p.b = color.data[ v*color.step+u*color.channels() ];
            p.g = color.data[ v*color.step+u*color.channels()+1 ];
            p.r = color.data[ v*color.step+u*color.channels()+2 ];
            pointCloud_temp->points.push_back( p );
        }


	PointCloud::Ptr pointCloud_voxfiltered( new PointCloud ); 
	if(downsample_flag)
	{
		pcl::VoxelGrid<PointT> voxel; // 网格滤波器，调整地图分辨率
		const double gridsize = 0.02; //分辨率
		voxel.setLeafSize( gridsize, gridsize, gridsize );
		voxel.setInputCloud( pointCloud_temp );
		voxel.filter( *pointCloud_voxfiltered );	
	}        
	else *pointCloud_voxfiltered = *pointCloud_temp;      

			
	pcl::PassThrough<PointT> pass; // z方向区间滤波器，由于rgbd相机的有效深度区间有限，把太远的去掉
	pass.setFilterFieldName("z");
	pass.setFilterLimits( 0.4, 4.0); //截取0-3m
	pass.setInputCloud( pointCloud_voxfiltered );
	pass.filter( *outputCloud );
	
}


int main( int argc, char** argv )
{

	if(argc!=2)
	{
		cerr<<"please give a directory of image data!"<<endl;
		return 1;
	}
	
	const string image_database_dir(argv[1]);
	//string image_database_dir ( "/home/robot/a_myfile/image_data2/follower_image_map226" );
	//string dir_result_after_g2o( image_database_dir + "/result_after.g2o" );

	ros::init(argc, argv, "pcl_localization"); //string here is the node name
	ros::NodeHandle nodehandle;

	cv::Mat color_img,depth_img;
	Eigen::Isometry3d camera_pose;
	
	//for(int tem=0;tem<10;tem++)
	if(readimgfromfile(image_database_dir,215,color_img,depth_img,camera_pose))
	{
		PointCloud::Ptr imgCloud( new PointCloud );
		image2cloud(color_img,depth_img,imgCloud,true);
		
		const string globalmap_pcd_dir(image_database_dir+"/map_reg_g2o.pcd");
		Map3dCloud pcld_loc(nodehandle);
		pcld_loc.load_gloabalmap(globalmap_pcd_dir);
		pcld_loc.set_currmap(imgCloud,camera_pose);
		pcld_loc.generate_refmap(camera_pose);
	
	}
	else
	{
		cerr << "get image failed!" << endl;
	}

}
*/


