#include <rclcpp/rclcpp.hpp>
#include "core.cpp" // ImagingRadarOdometryRun

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("radar4motion");
    
    int i_init_loop_frequency = 10;
    
    rclcpp::Rate loop_rate(i_init_loop_frequency);
    ImagingRadarOdometryRun<pcl::PointXYZI> imagingRadarOdometryRun(node);

    bool b_flag_run_vod_dataset = false;
    node->declare_parameter<bool>("b_flag_run_vod_dataset", false);
    node->get_parameter("b_flag_run_vod_dataset", b_flag_run_vod_dataset);

    // Run radar odometry with VOD dataset
    if (b_flag_run_vod_dataset)
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "RUNNING RADAR ODOMETRY WITH VOD DATASET");
        imagingRadarOdometryRun.ProcessRadarFiles();
    }
    // Run radar odometry (rostopic)
    else
    {
        RCLCPP_INFO_STREAM(node->get_logger(), "RUNNING RADAR ODOMETRY");
        while(rclcpp::ok())
        {
            imagingRadarOdometryRun.RunOdometry();
            rclcpp::spin_some(node);
            loop_rate.sleep();
        }
    }

    rclcpp::shutdown();
    return 0;
}