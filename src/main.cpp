#include "header.h"
#include "utils.hpp"
#include "ParameterManager.hpp"
#include "BipartiteSeparation.hpp"
#include "SurfaceNormalWithHull.hpp"
#include <pcl/filters/statistical_outlier_removal.h>

typedef std::pair<int, int> HULL_TUPLE;

using namespace pcl;

std::string CFG_PARAM_PATH = "/home/inaho-00/work/cpp/myBipartiteSurfaceNormal/cfg/recognition_parameter.toml";

int
main (int argc, char** argv)
{
    ParameterManager cfg_param(CFG_PARAM_PATH);
    std::string DATA_PATH = cfg_param.ReadStringData("Param", "data_path");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (DATA_PATH, *cloud) == -1) 
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    cloud = removeNan(cloud);
    cout << cloud->points.size()  << endl;
    pcl::PointIndices red_indices; pcl::PointIndices blue_indices;
    BipartiteSeparation(cloud, red_indices, blue_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_red (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blue (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (boost::make_shared<pcl::PointIndices>(red_indices));
    extract.setNegative (false);
    extract.filter (*cloud_red);

    extract.setInputCloud (cloud);
    extract.setIndices (boost::make_shared<pcl::PointIndices>(blue_indices));
    extract.setNegative (false);
    extract.filter (*cloud_blue);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_red_draw (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blue_draw (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    std::vector<EdgeDesc> red_mst; Graph red_graph; 
    deriveMinimumSpanningTree(cloud_red, red_mst, red_graph);
    std::vector<EdgeDesc> blue_mst;  Graph blue_graph; 
    deriveMinimumSpanningTree(cloud_blue, blue_mst, blue_graph);

    //calcHull4SurfaceNormal(cloud);
    std::vector<HULL_TUPLE> red_hull = calcHull4SurfaceNormal(cloud_red, cloud_blue);
    std::vector<HULL_TUPLE> blue_hull = calcHull4SurfaceNormal(cloud_blue, cloud_red);
    pcl::PointCloud<pcl::Normal>::Ptr red_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr blue_normals (new pcl::PointCloud<pcl::Normal>);

    int n_point_red = cloud_red->points.size();
    //Eigen::SparseMatrix<float, Eigen::ColMajor> A_red(n_point_red*3, n_point_red*3);
    std::vector<Eigen::MatrixXf> A_red;
    Eigen::VectorXf p_red(n_point_red*3);    
    Eigen::VectorXf b_red(n_point_red*3);
    Eigen::VectorXf n_red(n_point_red*3);
    Hull2NormalParam(cloud_red, cloud_blue, red_hull, A_red, p_red, b_red, n_red);

    int n_point_blue = cloud_blue->points.size();
    //Eigen::SparseMatrix<float, Eigen::ColMajor> A_blue(n_point_blue*3, n_point_blue*3);
    std::vector<Eigen::MatrixXf> A_blue;
    Eigen::VectorXf p_blue(n_point_blue*3);    
    Eigen::VectorXf b_blue(n_point_blue*3);
    Eigen::VectorXf n_blue(n_point_blue*3);
    Hull2NormalParam(cloud_blue, cloud_red, blue_hull, A_blue, p_blue, b_blue, n_blue);

    Eigen::SparseMatrix<float, Eigen::ColMajor> B_red(n_point_red*3, n_point_red*3);
    Eigen::VectorXf v_red(n_point_red*3);    
    Eigen::VectorXf m_red(n_point_red*3);
    Eigen::VectorXf w(n_point_red);

    getOptimizationParams(red_mst, red_graph, p_red, A_red, b_red, B_red, v_red, m_red, w, cloud_red);
    end = std::chrono::system_clock::now();  // 計測終了時間
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
    cout << "time:" << elapsed << endl;

    start = std::chrono::system_clock::now();  // 計測終了時間

    Eigen::VectorXf p_red_new;
    OptimP(B_red, v_red, m_red, p_red, n_point_red, p_red_new, w);

/*
    Eigen::SparseMatrix<float, Eigen::ColMajor> B_blue(n_point_blue*3, n_point_blue*3);
    Eigen::VectorXf v_blue(n_point_blue*3);    
    Eigen::VectorXf m_blue(n_point_blue*3);
    getOptimizationParams(blue_mst, blue_graph, p_blue, A_blue, b_blue, n_blue, B_blue, v_blue, m_blue);
    */

    end = std::chrono::system_clock::now();  // 計測終了時間
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間をミリ秒に変換
    cout << "time:" << elapsed << endl;

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_red_new (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i<n_point_red;i++){
        pcl::PointXYZ _point;
        _point.x = p_red_new(i*3 + 0);
        _point.y = p_red_new(i*3 + 1);
        _point.z = p_red_new(i*3 + 2);
        cloud_red_new->points.push_back(_point);
    }

    std::vector<HULL_TUPLE> red_hull_new = calcHull4SurfaceNormal(cloud_red_new, cloud_blue);
    Hull2NormalParam(cloud_red_new, cloud_blue, red_hull_new, A_red, p_red, b_red, n_red);


    //addRGBtoPointCloud(_cloud_red, cloud_red, 255, 0, 0);
    //addRGBtoPointCloud(_cloud_blue, cloud_blue, 0, 0, 255);
    //cout << red_indices.indices.size() << endl;
    //cout << blue_indices.indices.size() << endl;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis();
    //viewer->addPointCloud<pcl::PointXYZ> (cloud_red_new, "red");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "red");  


    addRGBtoPointCloudWithNormal(cloud_red_new, cloud_red_draw, n_red);
//    addRGBtoPointCloudWithNormal(cloud_blue_new, cloud_blue_draw, n_blue);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_red(cloud_red_draw);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_red_draw, rgb_red, "red");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "red");  
/*
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_blue(cloud_blue_draw);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_blue_draw, rgb_blue, "blue");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "blue");  
*/

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }


    return (0);
}