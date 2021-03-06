#ifndef STEREO_LIDAR_CALIBRATION_EXTRACTCHESSBOARD_HPP
#define STEREO_LIDAR_CALIBRATION_EXTRACTCHESSBOARD_HPP

#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include "config.hpp"
#include "utils.hpp"
#include "box_fitting.hpp"

using namespace Eigen;
using namespace std;

struct PassFilterParams{
    double min_x,max_x;
    double min_y,max_y;
    double min_z, max_z;
    PassFilterParams(std::vector<double> params){
        assert(params.size()==6);
        min_x = params[0];
        max_x = params[1];
        min_y = params[2];
        max_y = params[3];
        min_z = params[4];
        max_z = params[5];
    }
};

class ChessboardExtractor{
public:

    ChessboardExtractor() : m_pass_filter_params(PassFilterParams(Config::passFilterParams())),
                                     m_checkerboard_grid_size(Config::checkerboardGridSize()),
                                     m_checkerboard_square_size(Config::checkerboardSquareSize()),
                                     m_checkerboard_padding(Config::checkerboardPadding()) {}

public:
    // 主函数， 从环境点云中提取marker board 点云
    // bool extract(std::string& lidarPath, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud);
    // 点云聚类， 获取可能的点云簇
    void pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
        std::vector<pcl::PointIndices>& pcd_clusters);
    // 对上面的点云簇进行平面拟合 
    bool fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
        std::vector<pcl::PointIndices>& indices_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_pcd);

    bool extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
        std::vector<pcl::PointIndices>& indices_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr& chessboard_pcd);

    bool fitBoardCubic(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                       std::vector<pcl::PointIndices> &indices_clusters,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd,
                       Eigen::Affine3f &board_pose,
                       Eigen::Vector3f &board_size);
    // 将原始点云投影到估计的平面上
    void projPlaneFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &proj_cloud,
                         pcl::ModelCoefficients::Ptr &coeff);

    void pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd);

private:
    // 预处理， 滤出固定范围外的点云
    void getPointCloudInROI();
    void extract_pcd_by_indices(pcl::PointIndices &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &output_pcd);
    bool check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd);
    bool check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd, Eigen::VectorXf& coeff);
    bool check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd,
                                              Eigen::VectorXf &coeff,
                                              Eigen::Affine3f &pose,
                                              Eigen::Vector3f &size);
    PassFilterParams m_pass_filter_params;
    double m_checkerboard_square_size;        // 单位是m
    cv::Size m_checkerboard_grid_size;     // 标定板的内部方格数，注意水平方向为7
    vector<double> m_checkerboard_padding; // 标定板相对靶纸边缘的padding,方向？
};





#endif