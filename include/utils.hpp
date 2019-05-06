#pragma once

#include "header.h"
#include <boost/thread/thread.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <iomanip>
#include <fstream>

using namespace std;
using namespace pcl;

double D_MAX = std::numeric_limits<double>::max();
double D_MIN = std::numeric_limits<double>::min();

//removeNan: NaN要素を点群データから除去するメソッド
//input : target(NaN要素を除去する対象の点群)
//output: cloud(除去を行った点群)
pcl::PointCloud<PointXYZ>::Ptr removeNan(pcl::PointCloud<pcl::PointXYZ>::Ptr target){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int n_point = target->points.size();
  for(int i=0;i<n_point; i++){
    pcl::PointXYZ tmp_point;
    if(std::isfinite(target->points[i].x) || std::isfinite(target->points[i].y) || std::isfinite(target->points[i].z)){
      tmp_point.x = target->points[i].x;
      tmp_point.y = target->points[i].y;
      tmp_point.z = target->points[i].z;
      cloud->points.push_back(tmp_point);
    }
  }
//  cout << "varid points:" << cloud->points.size() << endl;
  return cloud;
}

pcl::visualization::PCLVisualizer::Ptr simpleVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void
addRGBtoPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color, int r, int g, int b){
  int n_point = cloud->points.size();
  for(int i=0; i<n_point ; i++){
    pcl::PointXYZRGB point;
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = cloud->points[i].z;
    point.r = r;
    point.g = g;
    point.b = b;
    cloud_color->points.push_back(point);
  }
}

void
addRGBtoPointCloudWithNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color, pcl::PointCloud<pcl::Normal>::Ptr &normal){
  int n_point = cloud->points.size();
  for(int i=0; i<n_point ; i++){
    pcl::PointXYZRGB point;
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = cloud->points[i].z;
    point.r = static_cast<unsigned char>(normal->points[i].normal_x*255.0);
    point.g = static_cast<unsigned char>(normal->points[i].normal_y*255.0);
    point.b = static_cast<unsigned char>(normal->points[i].normal_z*255.0);
    cloud_color->points.push_back(point);
  }
}

void
addRGBtoPointCloudWithNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color, Eigen::VectorXf &normals){
  int n_point = cloud->points.size();
  for(int i=0; i<n_point ; i++){
    pcl::PointXYZRGB point;
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = cloud->points[i].z;
    point.r = static_cast<unsigned char>(normals[i*3+0]*255.0);
    point.g = static_cast<unsigned char>(normals[i*3+1]*255.0);
    point.b = static_cast<unsigned char>(normals[i*3+2]*255.0);
    cloud_color->points.push_back(point);
  }
}
