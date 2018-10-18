/*************************************************************
    > Filename:    /home/mySlam/include/slamBase.h
    > Author:      raopei
    > Mail:        raopei1994@163.com
    > Created Time:18-10-12
    > Function:    1.define the camera intrinsic parameters
                   2.add 3d points to the point cloud map
                   3.transform the 2d image point to 3d world point

 **************************************************************/
#ifndef SLAM_SLAMBASE_H
#define SLAM_SLAMBASE_H
//add header files and standard librar
#include <fstream>
#include <vector>
#include <map>

using namespace std;

//opencv library
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//PCL library
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

//Eigen library
#include "Eigen/Core"
#include "Eigen/Geometry"

//type definition
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

//struct of camera intrinsic parameters
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx,cy,fx,fy,scale;
};

//Function interface
//image to point cloud
PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS &camera);

//transform single 2d image point to world 3d point
//input 3d point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera);

//features detection and matching

//structure of frame
struct FRAME
{
    cv::Mat rgb,depth;
    cv::Mat desp;
    vector<cv::KeyPoint> kp;
};

//the result of function of PNP
struct RESULT_OF_PNP
{
    cv:: Mat rvec,tvec;
    int inliers;
};

//Extraction of key points and feature descriptors
void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor );

// Calculate the movement between adjacent two frames.
//input: frame1, frame2, camera intrinsic parameters.
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

//
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

// joinPointCloud
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) ;

// class for getting parameters
class ParameterReader
{
public:
    ParameterReader( string filename="/home/raopei/mySlam/parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};


inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}

#endif //SLAM_SLAMBASE_H
