#include "extractChessboard.hpp"
#include "extractImageFeature.hpp"
#include "extractLidarFeature.hpp"
#include <pcl/common/geometry.h>
#include <sophus/so3.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core/eigen.hpp>
#include "optimization.hpp"
#include "utils.hpp"
#include "config.hpp"
#include "evaluation.hpp"
using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    string config_file;
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("config,c", boost::program_options::value<std::string>(&config_file)->default_value("config.yaml"), "config file.");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }
    Config::loadFromFile(config_file); // 
    std::string images_dir = Config::leftImageDatasetPath();
    std::string clouds_dir = Config::lidarCloudDatasetPath();
    std::vector<std::string> images;
    get_data_by_path(images, images_dir, Config::imageFormat());
    std::vector<std::string> clouds;
    get_data_by_path(clouds, clouds_dir, Config::cloudFormat());
    if(!check_data_by_index(images, clouds)){
        cerr <<"图像和点云文件应存在且文件名相同！！"<<endl;
        exit(-1);
    }

    ImageResults images_features;
    processImage(images, images_features, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs());
    cout<< "有效的图像帧有： "<<endl;
    for(auto& id:images_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    CloudResults cloud_features;
    processCloud(clouds, cloud_features);
    cout<< "有效的点云帧有： "<<endl;
    for(auto& id:cloud_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;
    getValidDataSet(images_features, cloud_features);
    auto& image_3d_corners = images_features.corners_3d; // 图像3d角点
    auto& image_planes_3d = images_features.planes_3d;
    auto& cloud_3d_corners = cloud_features.corners_3d; // 点云3d角点
    auto& plane_clouds_3d  = cloud_features.plane_points_3d;

    Eigen::VectorXd R_t(6);
    calculateInitialRt(cloud_3d_corners, image_3d_corners, R_t);
    cout<< "initial Rt \n"<< R_t<<endl; 

    // debug 
    // {
    //     for(auto& corners: image_3d_corners){
    //         for(auto& corner: corners)
    //         std::cout<< corner.transpose()<<std::endl;
    //     }
    //     exit(17);
    // }

    Optimizer optimizer_lc;
    ceres::Problem problem;
    std::vector<Vector3d> p1, p2;
    assert(image_3d_corners.size()==cloud_3d_corners.size());
    for(int i=0;i<image_3d_corners.size();++i){
        auto& image_corners = image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        optimizer_lc.addPointToPointConstriants(problem,cloud_corners,image_corners,R_t);
    }

    assert(image_planes_3d.size()==plane_clouds_3d.size());
    cout<< "use " << image_planes_3d.size() << " planes"<<endl;
    for(int i=0;i<plane_clouds_3d.size();++i){
        auto& image_plane = image_planes_3d[i];
        auto& plane_cloud = plane_clouds_3d[i];
        // cout<< image_plane <<endl;
        // cout<< plane_cloud.size() <<endl;
        // display_colored_by_depth(plane_cloud.makeShared());
        optimizer_lc.addPointToPlaneConstriants(problem,plane_cloud, image_plane,R_t );
    }
    
    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    Eigen::Matrix4d Rt = Matrix<double,4,4>::Identity();
    cout<< "Rt \n"<< R_t<<endl; 
    Eigen::Matrix3d R = Sophus::SO3d::exp(R_t.head(3)).matrix();
    Rt.block(0,0,3,3) = R;
    Rt.block(0,3,3,1) = R_t.tail(3);
    std::cout << Rt << std::endl;
    std::cout<< Config::tformL1C1()<<std::endl;
    std::cout << Rt-Config::tformL1C1()<<std::endl;


    bool eval =  false;
    if(eval){
        // debug
        // {
        //     auto mat = Config::tformL1C1();
        //     Eigen::Matrix3d mat_R = mat.block(0,0,3,3);
        //     Eigen::AngleAxisd aa;
        //     aa.fromRotationMatrix(mat_R);
        //     R_t.head(3) = aa.angle() * aa.axis();
        //     R_t.tail(3) = mat.block(0,3,3,1);
        // }

        cv::Mat rvec = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
        rvec.at<double>(0,0) = R_t(0);
        rvec.at<double>(1,0) = R_t(1);
        rvec.at<double>(2,0) = R_t(2);
        cv::Mat tvec = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
        tvec.at<double>(0,0) = R_t(3);
        tvec.at<double>(1,0) = R_t(4);
        tvec.at<double>(2,0) = R_t(5);

        auto& valid_index = cloud_features.valid_index;

        for(int i = 0;i<valid_index.size();++i)
        {
            int k = valid_index[i];
            cv::Mat img = cv::imread(images[k], cv::IMREAD_COLOR);

            // projectLidarOnImage(img, plane_clouds_3d[i] , rvec, tvec, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs());
            // cv::imshow("eval", img);
            // cv::waitKey(0);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (!loadPointXYZ(clouds[k], cloud)) continue;
            // debug 
            // {
            //     std::vector<pcl::PointCloud<pcl::PointXYZ>> show_cloud;
            //     show_cloud.push_back(*cloud);
            //     show_cloud.push_back(plane_clouds_3d[i]);
            //     display_multi_clouds(show_cloud);
            // }

            auto color_pcd = colorizeCloud(img, *cloud , rvec, tvec, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs());
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
            // viewer->addPointCloud<pcl::PointXYZRGB>(*color_pcd , "sample cloud");
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(color_pcd);
            viewer->addPointCloud<pcl::PointXYZRGB> (color_pcd, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小
            viewer->addCoordinateSystem (1.0);
            viewer->initCameraParameters ();
            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                sleep(0.1);
            }
        }
    }

    bool compute_error = true;
    if(compute_error){
        auto translation_error = computeTranslationError(image_3d_corners, cloud_3d_corners,Rt );
        std::cout<< " translation_error:\n ";
        for(auto& error:translation_error){
            std::cout<< error <<" ";
        }
        std::cout<<std::endl;
        auto rotation_error = computeRotationError(image_3d_corners, cloud_3d_corners,Rt );
        std::cout<< " rotation_error:\n ";
        for(auto& error:rotation_error){
            std::cout<< error <<" ";
        }
        std::cout<<std::endl;
        auto reprojection_error = computeReprojectionError(image_3d_corners, cloud_3d_corners,Rt );
        std::cout<< " reprojection_error:\n ";
        for(auto& error:reprojection_error){
            std::cout<< error <<" ";
        }
        std::cout<<std::endl;
    }
    
    return 0;
}