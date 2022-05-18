#include "config.hpp"
#include <gtest/gtest.h>
#include <opencv2/core.hpp>

// to do , 需要对配置文件不合法参数报错。

// mono camera lidar calibration config

class MonoConfigTest : public testing::Test
{
    virtual void SetUp()
    {
        std::string config_file = "../config/hdl64_mono.yaml";
        Config::loadFromFile(config_file);
    }

    virtual void TearTown()
    {

    }
};

TEST_F(MonoConfigTest, getCalibType)
{
    std::string calib_type = "mono_lidar";
    EXPECT_EQ(calib_type, Config::calibType());
}

TEST_F(MonoConfigTest, getLeftImageDatasetPath)
{
    std::string left_image_dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/images";
    EXPECT_EQ(left_image_dataset_path, Config::leftImageDatasetPath());
}

TEST_F(MonoConfigTest, getLidarCloudDatasetPath)
{
    std::string lidar_cloud_dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/pointCloud";
    EXPECT_EQ(lidar_cloud_dataset_path, Config::lidarCloudDatasetPath());
}

TEST_F(MonoConfigTest, getImageFormat)
{
    std::string image_format = "png";
    EXPECT_EQ(image_format, Config::imageFormat());
}

TEST_F(MonoConfigTest, getCloudFormat)
{
    std::string cloud_format = "pcd";
    EXPECT_EQ(cloud_format, Config::cloudFormat());
}

TEST_F(MonoConfigTest, getLeftCameraMatrix)
{
    cv::Mat left_camera_matrix = (cv::Mat_<double>(3,3) <<1614.546232069338, 0, 
            641.2276358621397, 0, 1614.669013419422, 480.1410561665820, 0, 0, 1);

    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
             EXPECT_EQ(left_camera_matrix.at<double>(i,j), Config::leftCameraMatrix().at<double>(i,j));
        }
    }
}


TEST_F(MonoConfigTest, getLeftCameraDistCoeffs)
{
    cv::Mat left_camera_dist_coeffs = (cv::Mat_<double>(5,1) <<-0.004497294509341,
            0.020426051162860, 0,  0, 0);

    for(int i=0;i<5;++i){
             EXPECT_EQ(left_camera_dist_coeffs.at<double>(i,0), Config::leftCameraDistCoeffs().at<double>(i,0));
    }
}

TEST_F(MonoConfigTest, getCheckerboardSquareSize)
{
    double checkerboard_square_size = 0.2;
    EXPECT_EQ(checkerboard_square_size, Config::checkerboardSquareSize());
}

TEST_F(MonoConfigTest, getCheckerboardGridSize)
{
    cv::Size checkerboard_grid_size = cv::Size(7,4);
    EXPECT_EQ(checkerboard_grid_size.width, Config::checkerboardGridSize().width);
    EXPECT_EQ(checkerboard_grid_size.height, Config::checkerboardGridSize().height);
}

TEST_F(MonoConfigTest, getCheckerboardPadding)
{
    std::vector<double>  checkerboard_padding = {0,0,0,0};
    for(int i = 0;i<4;++i){
        EXPECT_EQ(checkerboard_padding[i], Config::checkerboardPadding()[i]);
    }
}

TEST_F(MonoConfigTest, getPassFilterParams)
{
    std::vector<double>  pass_filter_params = {-10,10,-10,10,-10,10};
    for(int i = 0;i<6;++i){
        EXPECT_EQ(pass_filter_params[i], Config::passFilterParams()[i]);
    }
}


// stereo camera lidar calibration config

class StereoConfigTest : public testing::Test
{
    virtual void SetUp()
    {
        std::string config_file = "../config/hdl64_stereo.yaml";
        Config::loadFromFile(config_file);
    }

    virtual void TearTown()
    {

    }
};

TEST_F(StereoConfigTest, getCalibType)
{
    std::string calib_type = "stereo_lidar";
    EXPECT_EQ(calib_type, Config::calibType());
}

TEST_F(StereoConfigTest, getLeftImageDatasetPath)
{
    std::string left_image_dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64_stereo/left_images";
    EXPECT_EQ(left_image_dataset_path, Config::leftImageDatasetPath());
}

TEST_F(StereoConfigTest, getRightImageDatasetPath)
{
    std::string right_image_dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64_stereo/right_images";
    EXPECT_EQ(right_image_dataset_path, Config::rightImageDatasetPath());
}

TEST_F(StereoConfigTest, getLidarCloudDatasetPath)
{
    std::string lidar_cloud_dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64_stereo/pointclouds";
    EXPECT_EQ(lidar_cloud_dataset_path, Config::lidarCloudDatasetPath());
}

TEST_F(StereoConfigTest, getImageFormat)
{
    std::string image_format = "png";
    EXPECT_EQ(image_format, Config::imageFormat());
}

TEST_F(StereoConfigTest, getCloudFormat)
{
    std::string cloud_format = "pcd";
    EXPECT_EQ(cloud_format, Config::cloudFormat());
}

TEST_F(StereoConfigTest, getLeftCameraMatrix)
{
    cv::Mat left_camera_matrix = (cv::Mat_<double>(3,3) <<1625.51202880799, 0, 
            640.192524836779, 0, 1625.60338178797, 480.619178662295, 0, 0, 1);

    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
             EXPECT_EQ(left_camera_matrix.at<double>(i,j), Config::leftCameraMatrix().at<double>(i,j));
        }
    }
}


TEST_F(StereoConfigTest, getLeftCameraDistCoeffs)
{
    cv::Mat left_camera_dist_coeffs = (cv::Mat_<double>(5,1) <<-0.000103058512210208,
            -0.00564163864590723, 0,  0, 0);

    for(int i=0;i<5;++i){
             EXPECT_EQ(left_camera_dist_coeffs.at<double>(i,0), Config::leftCameraDistCoeffs().at<double>(i,0));
    }
}

TEST_F(StereoConfigTest, getRightCameraMatrix)
{
    cv::Mat right_camera_matrix = (cv::Mat_<double>(3,3) <<1624.85736756878, 0, 
            639.970457013175, 0, 1624.93598961025, 480.848625423083, 0, 0, 1);

    for(int i=0;i<3;++i){
        for(int j=0;j<3;++j){
             EXPECT_EQ(right_camera_matrix.at<double>(i,j), Config::rightCameraMatrix().at<double>(i,j));
        }
    }
}


TEST_F(StereoConfigTest, getRightCameraDistCoeffs)
{
    cv::Mat right_camera_dist_coeffs = (cv::Mat_<double>(5,1) <<-0.00132142538297009,
            0.0200478364181037, 0,  0, 0);

    for(int i=0;i<5;++i){
             EXPECT_EQ(right_camera_dist_coeffs.at<double>(i,0), Config::rightCameraDistCoeffs().at<double>(i,0));
    }
}

TEST_F(StereoConfigTest, getCheckerboardSquareSize)
{
    double checkerboard_square_size = 0.2;
    EXPECT_EQ(checkerboard_square_size, Config::checkerboardSquareSize());
}

TEST_F(StereoConfigTest, getCheckerboardGridSize)
{
    cv::Size checkerboard_grid_size = cv::Size(7,4);
    EXPECT_EQ(checkerboard_grid_size.width, Config::checkerboardGridSize().width);
    EXPECT_EQ(checkerboard_grid_size.height, Config::checkerboardGridSize().height);
}

TEST_F(StereoConfigTest, getCheckerboardPadding)
{
    std::vector<double>  checkerboard_padding = {0,0,0,0};
    for(int i = 0;i<4;++i){
        EXPECT_EQ(checkerboard_padding[i], Config::checkerboardPadding()[i]);
    }
}

TEST_F(StereoConfigTest, getPassFilterParams)
{
    std::vector<double>  pass_filter_params = {-10,10,-10,10,-10,10};
    for(int i = 0;i<6;++i){
        EXPECT_EQ(pass_filter_params[i], Config::passFilterParams()[i]);
    }
}
