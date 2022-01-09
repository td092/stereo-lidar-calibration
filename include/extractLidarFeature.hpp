#ifndef STEREO_LIDAR_CALIBRATION_EXTRACTLIDARFEATURE_HPP
#define STEREO_LIDAR_CALIBRATION_EXTRACTLIDARFEATURE_HPP
#include <iostream>
#include <memory>   /// std::shared_ptr
#include <pcl/common/common.h>
#include <vector>
#include <Eigen/Core>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/intersections.h>
#include <Eigen/Core>
#include <pcl/common/geometry.h>
#include <algorithm>
#include <unordered_map>
#include <map>
#include "extractChessboard.hpp"
#include "optimization.hpp"
#include "utils.hpp"


using namespace Eigen;
using namespace cv;
using namespace std;
using namespace pcl;

class CloudResults;

bool loadPointXYZ(string& path, PointCloud<PointXYZ>::Ptr& pcd);
void getValidDataSet(ImageResults& images_features, CloudResults& cloud_features);
void getValidDataSet(ImageResults& images_features1,ImageResults& images_features2, CloudResults& cloud_features);


class CloudResults{
public:
    vector<int> valid_index; // 有效的点云index
    vector<vector<Vector3d>> corners_3d; // 点云3d角点
    vector<PointCloud<PointXYZ>> plane_points_3d; 
    //vector<vector<Vector3d>> plane_points_3d;
    vector<vector<VectorXd>> lines_3d; // 点云边缘直线方程
    vector<vector<PointCloud<PointXYZ>>> lines_points_3d ;//直线包含的点云
};



template<typename T>
class LidarFeatureDetector{
public:
    LidarFeatureDetector(T &config) : m_chessboard_extractor(ChessboardExtractor<T>(config)),
                                    m_checkerboard_grid_size(config.checkerboard_grid_size),
                                     m_checkerboard_square_size(config.checkerboard_square_size),
                                     m_checkerboard_padding(config.checkerboard_padding) {}
    // void getCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ>& chessboard_corners); // 从直线方程中获取四个角点

    // void getEdgePointCloud(std::vector<std::vector<pcl::PointXYZ>>& rings,pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pcd);// 从rings中提取边缘点云

    // bool extractFourEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
    //                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &line_points,
    //                       std::vector<Eigen::VectorXf> &line_params); // 从边缘点云中拟合四个直线

    
    // void detectPlane(pcl::PointCloud<PointXYZ>::Ptr& plane_pcd);
    bool extractPlaneCloud(PointCloud<PointXYZ>::Ptr &input_cloud, PointCloud<PointXYZ>::Ptr &plane_pcd);
    bool extractPlaneCloud1(PointCloud<PointXYZ>::Ptr &input_cloud,
                            PointCloud<PointXYZ>::Ptr &plane_pcd,
                            Eigen::Affine3f &board_pose,
                            Eigen::Vector3f &board_size);
    void extractEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd);
    bool extractFourLines(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
                          std::vector<Eigen::VectorXf> &lines_params,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);
    bool estimateFourCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ>& corners); // 从直线方程中获取四个角点

    // 调整lidar points 的顺序
    void reorder_corners(std::vector<pcl::PointXYZ>& ori_corners, std::vector<pcl::PointXYZ>& reordered_corners);

    // 调整边缘直线上的点云顺序
    void reorder_line_points(std::vector<pcl::PointXYZ>  &reordered_corners,
                            vector<PointCloud<PointXYZ>> &reordered_lines_points,
                            vector<pair<pair<PointXYZ, PointXYZ>, PointCloud<PointXYZ>>> &line_corners_to_points);
    // 利用边界线点云优化角点
    void refineCorners(vector<Vector3d>& corners_3d, vector<PointCloud<PointXYZ>>& lines_points_3d);

    void estimateCornersFromBoardParams(Eigen::Affine3f& board_pose, Eigen::Vector3f& board_size, vector<PointXYZ>& corners);

private:
    // void getRings(std::vector<std::vector<pcl::PointXYZ>>& rings); // 将marker board点云按照线束id存储
    void remove_inliers(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                        std::vector<int> inliers_indices,
                        pcl::PointCloud<pcl::PointXYZ> &cloud_out);

    void reorder_lines(std::vector<Eigen::VectorXf> &lines_params,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);

    void reorder_lines1(std::vector<Eigen::VectorXf> &lines_params,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);

    bool isValidCorners(vector<PointXYZ>& corners);



    pcl::PointCloud<pcl::PointXYZ>::Ptr m_marker_board_pcd;
    int m_number_of_rings;

private:
    ChessboardExtractor<T> m_chessboard_extractor;
    const double m_checkerboard_square_size;
    const Size m_checkerboard_grid_size;    
    const vector<double> m_checkerboard_padding; 
};


template<typename T>
void LidarFeatureDetector<T>::estimateCornersFromBoardParams(Eigen::Affine3f& board_pose, Eigen::Vector3f& board_size, vector<PointXYZ>& corners)
{
    corners.clear();
    PointCloud<PointXYZ> corners_temp;
    double w = board_size(0);
    double l = board_size(1);
    double h = board_size(2);
    corners_temp.push_back(PointXYZ(-w/2.0,-l/2.0,h/2.0));
    corners_temp.push_back(PointXYZ(w/2.0, -l/2.0, h/2.0));
    corners_temp.push_back(PointXYZ(w/2.0, l/2.0, h/2.0));
    corners_temp.push_back(PointXYZ(-w/2.0, l/2.0, h/2.0));
    pcl::transformPointCloud (corners_temp, corners_temp, board_pose.matrix());
    for(auto& point:corners_temp.points){
        corners.push_back(point);
    }
    return;
}

template<typename T>
void LidarFeatureDetector<T>::refineCorners(vector<Vector3d>& corners_3d, vector<PointCloud<PointXYZ>>& line_points_3d)
{
    assert(line_points_3d.size()== corners_3d.size());
    assert(line_points_3d.size()==4);

    double corners[12] = {corners_3d[0](0), corners_3d[0](1), corners_3d[0](2),
                          corners_3d[1](0), corners_3d[1](1), corners_3d[1](2),
                          corners_3d[2](0), corners_3d[2](1), corners_3d[2](2),
                          corners_3d[3](0), corners_3d[3](1), corners_3d[3](2)};
    ceres::Problem problem;
    vector<double> boad_lengths;
    for(int i=0;i<4;++i){
        double board_length;
        if(i%2==0){
            board_length = (m_checkerboard_grid_size.width + 1)*m_checkerboard_square_size + m_checkerboard_padding[i];
        }else{
            board_length = (m_checkerboard_grid_size.height + 1)*m_checkerboard_square_size + m_checkerboard_padding[i];
        }
        boad_lengths.push_back(board_length);
    }
    for (std::size_t i = 0; i < line_points_3d.size(); ++i)
    {

        ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<BoardCornersError, 1, 12>(new BoardCornersError(line_points_3d[i], i, boad_lengths));
        problem.AddResidualBlock(cost_function, nullptr, corners);
    }

    ceres::Solver::Options option;
    option.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(option, &problem, &summary);
    vector<Vector3d> corners_temp;
    for(int i=0;i<4;++i){
        corners_temp.push_back(Vector3d(corners[i*3], corners[i*3+1], corners[i*3+2]));
    }
    corners_3d = corners_temp;
    return;
}

template<typename T>
bool LidarFeatureDetector<T>::isValidCorners(vector<PointXYZ>& corners)
{
    int corner_num = corners.size();
    if (corner_num != 4)
    {
        return false;
    }
    double len_x = (m_checkerboard_grid_size.width+1)*m_checkerboard_square_size + m_checkerboard_padding[1] + m_checkerboard_padding[3];
    double len_y = (m_checkerboard_grid_size.height+1)*m_checkerboard_square_size+m_checkerboard_padding[0] + m_checkerboard_padding[2];
    vector<double> board_length = {len_x, len_x, len_y, len_y};
    sort(board_length.begin(), board_length.end());
    const double board_angle = M_PI / 2.0;

    // 计算距离评分和角度评分
    pcl::PointXYZ pre_corner, cur_corner, next_corner;
    double length_diff = 0.0, angle_diff = 0.0;
    vector<double> distances;
    for (size_t i = 0; i < corner_num; ++i)
    {
        pre_corner = corners[(i + 3) % 4];
        cur_corner = corners[i];
        next_corner = corners[(i + 1) % 4];
        double dist = pcl::euclideanDistance(cur_corner, next_corner);
        distances.push_back(dist);
        Eigen::Vector3f a, b;
        a << (cur_corner.x - pre_corner.x), (cur_corner.y - pre_corner.y), (cur_corner.z - pre_corner.z);
        b << (cur_corner.x - next_corner.x), (cur_corner.y - next_corner.y), (cur_corner.z - next_corner.z);
        double angle = std::acos(a.dot(b) / (a.norm() * b.norm()));
        angle_diff += std::abs(angle - board_angle);
    }
    double angle_score = 1 - angle_diff / board_angle;
    
    sort(distances.begin(), distances.end());
    for(int i=0;i<corner_num;++i){
        length_diff += abs(board_length[i]-distances[i]);
    }
    length_diff /= corner_num;
    double length_core = 1 - length_diff / board_length[0];
    // debug
    // cout<< angle_score<<endl;
    // cout<< length_core <<endl;
    // cout<< "------------------"<<endl;
    if(angle_score<0.9) return false;
    if(length_core<0.9) return false;
    return true;
}

template<typename T>
bool LidarFeatureDetector<T>::extractFourLines(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
                                            std::vector<Eigen::VectorXf> &lines_params,
                                            std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points)
{
    // debug
    // display_colored_by_depth(edge_pcd);

    lines_params.clear();
    lines_points.clear();
    for (int i = 0; i < 4; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(edge_pcd);
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_ptr));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_l(model_l);
        ransac_l.setDistanceThreshold(0.03);
        ransac_l.computeModel();
        std::vector<int> line_inliers;
        ransac_l.getInliers(line_inliers);
        if (!line_inliers.empty())
        {
            pcl::PointCloud<pcl::PointXYZ> line_i;
            pcl::copyPointCloud<pcl::PointXYZ>(*edge_pcd,
                                               line_inliers,
                                               line_i);
            lines_points.push_back(line_i);                       // 边界点云
            lines_params.push_back(ransac_l.model_coefficients_); // 边界线的参数
        }
        else
        {
            break;
        }
        pcl::PointCloud<pcl::PointXYZ> plane_no_line;
        remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
        *edge_pcd = plane_no_line;
    }
    if (lines_points.size() == 4)
    {
        reorder_lines1(lines_params,lines_points );
        return true;
    }
    return false;
}
template<typename T>
void LidarFeatureDetector<T>::reorder_lines(std::vector<Eigen::VectorXf> &v_line_coeff,
                                         std::vector<pcl::PointCloud<pcl::PointXYZ>> &v_line_cloud)
{
    int line_num = v_line_coeff.size();
    std::vector<std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>>> coeff_cloud_v;
    for (size_t i = 0; i < line_num; ++i)
    {
        coeff_cloud_v.emplace_back(std::make_pair(v_line_coeff[i], v_line_cloud[i]));
    }
    std::sort(coeff_cloud_v.begin(), coeff_cloud_v.end(), [](std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>> &lhs, std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>> &rhs)
              { return lhs.first(2) > rhs.first(2); });
    // sort by descending order
    // The six coefficients of the line are given by a point on the line and the direction of the line as:
    // [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
    // line order in clockwise: top-left, top-right, bottom-right, bottom-left
    if (coeff_cloud_v[0].first(1) < coeff_cloud_v[1].first(1))
    {
        std::swap(coeff_cloud_v[0], coeff_cloud_v[1]);
    }
    if (coeff_cloud_v[2].first(1) > coeff_cloud_v[3].first(1))
    {
        std::swap(coeff_cloud_v[2], coeff_cloud_v[3]);
    }

    v_line_coeff.clear();
    v_line_cloud.clear();
    for (size_t i = 0; i < line_num; ++i)
    {
        // std::cout << "coeff: " << coeff_cloud_v[i].first.size() << std::endl;
        // std::cout << "cloud: " << coeff_cloud_v[i].second.size() << std::endl;
        v_line_coeff.emplace_back(coeff_cloud_v[i].first);
        v_line_cloud.emplace_back(coeff_cloud_v[i].second);
    }
    return;
}
template<typename T>
void LidarFeatureDetector<T>::reorder_lines1(std::vector<Eigen::VectorXf> &v_line_coeff,
                                         std::vector<pcl::PointCloud<pcl::PointXYZ>> &v_line_cloud)
{
    assert(v_line_coeff.size()==v_line_cloud.size()&& v_line_cloud.size()==4);
    Eigen::Vector3d line0,line1;
    line0 << v_line_coeff[0](3) , v_line_coeff[0](4), v_line_coeff[0](5);
    line1<< v_line_coeff[1](3) , v_line_coeff[1](4), v_line_coeff[1](5);    
    double rad =line0.dot(line1) /(line0.norm()*line1.norm());
    if(abs(rad)>0.2){
        Eigen::VectorXf line = v_line_coeff[1];
        pcl::PointCloud<pcl::PointXYZ> cloud = v_line_cloud[1];
        v_line_coeff[1] = v_line_coeff[2];
        v_line_cloud[1] = v_line_cloud[2];
        v_line_cloud[2] = cloud;
        v_line_coeff[2] = line;
        // swap(v_line_cloud[1], v_line_cloud[2]);
    }
    return;
}
template<typename T>
void LidarFeatureDetector<T>::remove_inliers(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                          std::vector<int> inliers_indices,
                                          pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    std::vector<int> outliers_indicies;
    for (size_t i = 0; i < cloud_in.size(); i++)
    {
        if (find(inliers_indices.begin(), inliers_indices.end(), i) == inliers_indices.end())
        {
            outliers_indicies.push_back(i);
        }
    }
    pcl::copyPointCloud<pcl::PointXYZ>(cloud_in, outliers_indicies, cloud_out);
}
// bool LidarFeatureDetector::extractFourEdges(pcl::PointCloud<pcl::PointXYZIr>::Ptr &edge_pcd,
//                                             std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_points,
//                                             std::vector<Eigen::VectorXf> &line_params)
// {
//     for (int i = 0; i < 4; i++)
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(edge_pcd);
//         pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud_ptr));
//         pcl::RandomSampleConsensus<pcl::PointXYZI> ransac_l(model_l);
//         ransac_l.setDistanceThreshold(0.02);
//         ransac_l.computeModel();
//         std::vector<int> line_inliers;
//         ransac_l.getInliers(line_inliers);
//         if (!line_inliers.empty())
//         {
//             pcl::PointCloud<pcl::PointXYZI> line_i;
//             pcl::copyPointCloud<pcl::PointXYZI>(*edge_pcd, line_inliers, line_i);
//             line_points.push_back(line_i);                       // 边界点云
//             line_params.push_back(ransac_l.model_coefficients_); // 边界线的参数
//         }
//         else
//         {
//             break;
//         }
//         pcl::PointCloud<pcl::PointXYZI> plane_no_line;
//         remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
//         *cloud_msg_pcl = plane_no_line;
//     }
//     if (line_points.size() == 4)
//     {
//         return true;
//     }
//     return false;
// }

// void LidarFeatureDetector::detectPlane(pcl::PointCloud<PointXYZIr>::Ptr& plane_pcd)
// {
//     pcl::PointCloud<PointXYZIr>::Ptr cloud_in_ptr(new pcl::PointCloud<PointXYZIr>);
//     *cloud_in_ptr = m_marker_board_pcd;
//     pcl::SampleConsensusModelPlane<pcl::PointXYZIr>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZIr>(cloud_in_ptr));
//     pcl::RandomSampleConsensus<PointXYZIr> ransac(model_p);
//     ransac.setDistanceThreshold(0.05);
//     bool model_computed = ransac.computeModel();
//     std::vector<int> inlier_indices;
//     if (model_computed) {
//         ransac.getInliers(inlier_indices);
//         pcl::copyPointCloud<PointXYZIr>(*cloud_in_ptr,inlier_indices,plane_pcd);
//     }
//     return;
// }

// void LidarFeatureDetector::getCorners(std::vector<Eigen::VectorXf> &line_params,
//                                      vector<pcl::PointXYZ> &chessboard_corners)
// {
//     Eigen::Vector4f p1, p2, p_intersect;
//     for(int i=0;i<4;++i){
//         pcl::lineToLineSegment(line_params[i],line_params[(i+1)%4], p1,p2);
//         for (int j = 0; j < 4; j++) {
//             p_intersect(j) = (p1(j) + p2(j)) / 2.0;
//         }
//         chessboard_corners->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
//     }
//     return;
// }
template<typename T>
bool LidarFeatureDetector<T>::estimateFourCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ> &corners)
{
    Eigen::Vector4f p1, p2, p_intersect;
    for (int i = 0; i < 4; ++i)
    {
        pcl::lineToLineSegment(line_params[i], line_params[(i + 1) % 4], p1, p2);
        for (int j = 0; j < 4; j++)
        {
            p_intersect(j) = (p1(j) + p2(j)) / 2.0;
        }
        corners.push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
    }
    if(!isValidCorners(corners)) return false;
    return true;
}


template<typename T>
void LidarFeatureDetector<T>::extractEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd)
{
    edge_pcd->clear();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    n.setInputCloud(input_cloud);
    n.setSearchMethod(tree);
    //n.setRadiusSearch(1);
    n.setKSearch(50); 
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.compute(*normals);

    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
    boundEst.setInputCloud(input_cloud);
    boundEst.setInputNormals(normals);
    //boundEst.setRadiusSearch(1.0);
    boundEst.setKSearch(50);
    boundEst.setAngleThreshold(M_PI / 2); 

    boundEst.setSearchMethod(tree);
    pcl::PointCloud<pcl::Boundary> boundaries;
    boundEst.compute(boundaries);
    for (int i = 0; i < input_cloud->points.size(); i++)
    {

        if (boundaries[i].boundary_point > 0)
        {
            edge_pcd->push_back(input_cloud->points[i]);
        }
    }
    //debug
    // display_colored_by_depth(edge_pcd);
    // exit(17);
    // return;
}


template<typename T>
bool LidarFeatureDetector<T>::extractPlaneCloud(PointCloud<PointXYZ>::Ptr &input_cloud, PointCloud<PointXYZ>::Ptr &plane_pcd)
{
    m_chessboard_extractor.pass_filter(input_cloud); // 带通滤波
    vector<PointIndices> indices_clusters;
    m_chessboard_extractor.pcd_clustering(input_cloud, indices_clusters); // 聚类
    if(m_chessboard_extractor.extractChessboard(input_cloud, indices_clusters,plane_pcd)){
        //Todo 保留平面上的点
        // debug 
        // display_colored_by_depth(plane_pcd);
        return true;
    }
         // 查找平面
    return false;
}

template <typename T>
bool LidarFeatureDetector<T>::extractPlaneCloud1(PointCloud<PointXYZ>::Ptr &input_cloud,
                                                 PointCloud<PointXYZ>::Ptr &plane_pcd,
                                                 Eigen::Affine3f &board_pose,
                                                 Eigen::Vector3f &board_size)
{
    m_chessboard_extractor.pass_filter(input_cloud);
    vector<PointIndices> indices_clusters;
    m_chessboard_extractor.pcd_clustering(input_cloud, indices_clusters); // 聚类
    if (m_chessboard_extractor.fitBoardCubic(input_cloud, indices_clusters, plane_pcd, board_pose, board_size))
    {
        //Todo 保留平面上的点
        // debug
        // display_colored_by_depth(plane_pcd);
        return true;
    }
    // 查找平面
    return false;
}

// 调整lidar points 的顺序
template<typename T>
void LidarFeatureDetector<T>::reorder_corners(std::vector<pcl::PointXYZ>& ori_corners, std::vector<pcl::PointXYZ>& reordered_corners)
{
    reordered_corners.clear();
    float d1 = pcl::geometry::distance(ori_corners[0], ori_corners[0]);
    float d2 = pcl::geometry::distance(ori_corners[1], ori_corners[2]);
    if(d1<d2){
        std::reverse(ori_corners.begin()+1, ori_corners.end());
    }
    if(std::max(ori_corners[0].z, ori_corners[1].z)< std::max(ori_corners[2].z, ori_corners[3].z)){
        reordered_corners.push_back(ori_corners[2]);
        reordered_corners.push_back(ori_corners[3]);
        reordered_corners.push_back(ori_corners[0]);
        reordered_corners.push_back(ori_corners[1]);
    }else{
        reordered_corners = ori_corners;
    }
    if(reordered_corners[0].y<reordered_corners[1].y){
        std::reverse(reordered_corners.begin(), reordered_corners.begin()+2);
        std::reverse(reordered_corners.begin()+2, reordered_corners.end());
    }
    return;
}

// 调整边缘直线上的点云顺序
template<typename T>
void LidarFeatureDetector<T>::reorder_line_points(std::vector<pcl::PointXYZ>  &reordered_corners,
                         vector<PointCloud<PointXYZ>> &reordered_lines_points,
                         vector<pair<pair<PointXYZ, PointXYZ>, PointCloud<PointXYZ>>> &line_corners_to_points)
{
    reordered_lines_points.clear();
    for(int i=0;i<4;++i){
        auto p1 = make_pair(reordered_corners[i], reordered_corners[(i+1)%4]);
        auto p2 = make_pair(reordered_corners[(i+1)%4], reordered_corners[i]);
        for(int j=0;j<4;++j){
            auto& corners_points = line_corners_to_points[j];
            if(pcl::geometry::distance(corners_points.first.first, p1.first)< 1e-2 && pcl::geometry::distance(corners_points.first.second, p1.second)<1e-2){
                reordered_lines_points.push_back(corners_points.second);
            }
            if(pcl::geometry::distance(corners_points.first.first, p2.first)< 1e-2 && pcl::geometry::distance(corners_points.first.second, p2.second)<1e-2){
                reordered_lines_points.push_back(corners_points.second);
            }
        }
    }
    return;
}

template<typename T>
void processCloud(T& config, vector<string>& cloud_paths, CloudResults& cloud_features)
{
    LidarFeatureDetector<T> lidar_feature_detector(config);
    
    for(int i=0;i<cloud_paths.size();++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!loadPointXYZ(cloud_paths[i], input_cloud)) continue;
        // cout<< input_cloud->points.size()<<endl;
        // display_colored_by_depth(input_cloud);
        PointCloud<PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f board_pose;
        Eigen::Vector3f board_size;
        //if(!lidar_feature_detector.extractPlaneCloud(input_cloud,plane_cloud)) continue;
        if(!lidar_feature_detector.extractPlaneCloud1(input_cloud, plane_cloud, board_pose, board_size)) continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        lidar_feature_detector.extractEdgeCloud(plane_cloud, edge_cloud);// 提取边缘点云
        std::vector<Eigen::VectorXf> lines_params;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> lines_points;
        if (!lidar_feature_detector.extractFourLines(edge_cloud, lines_params, lines_points)) continue;// 提取四条边界线
        std::vector<pcl::PointXYZ> corners, reordered_corners;
        if (lidar_feature_detector.estimateFourCorners(lines_params, corners))
        {
            
            vector<pair<std::pair<pcl::PointXYZ, pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>> line_corners_to_points;
            for (int i = 0; i < 4; i++)
            {
                line_corners_to_points.push_back(make_pair(make_pair(corners[i], corners[(i + 1) % 4]), lines_points[(i + 1) % 4]));
            }                                                                   // 建立角点和边缘直线上点云的对应关系
            lidar_feature_detector.reorder_corners(corners, reordered_corners); // 角点重排序，与camera corner 保持对应关系
            std::vector<pcl::PointCloud<pcl::PointXYZ>> reordered_lines_points;
            lidar_feature_detector.reorder_line_points(reordered_corners, reordered_lines_points, line_corners_to_points); // 重排序边缘直线上的点
            // debug
            // display_multi_clouds(reordered_lines_points);
            vector<Vector3d> corners_temp;
            for (auto &corner : reordered_corners)
            {
                corners_temp.push_back(Vector3d(corner.x, corner.y, corner.z));
            }
            lidar_feature_detector.refineCorners(corners_temp, reordered_lines_points);
            cloud_features.lines_points_3d.push_back(reordered_lines_points);
            cloud_features.corners_3d.push_back(corners_temp);
        }
        else
        {
            lidar_feature_detector.estimateCornersFromBoardParams(board_pose, board_size, corners);
            lidar_feature_detector.reorder_corners(corners, reordered_corners); 
            vector<Vector3d> corners_temp;
            for (auto &corner : reordered_corners)
            {
                corners_temp.push_back(Vector3d(corner.x, corner.y, corner.z));
            }
            cloud_features.corners_3d.push_back(corners_temp);
        }
        // debug
        // display_four_corners(corners);
        cloud_features.plane_points_3d.push_back(*plane_cloud);
        cloud_features.valid_index.push_back(i);
    }
    return;
}



#endif