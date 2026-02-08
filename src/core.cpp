/*
 * @copyright Automotive Intelligence Lab, Hanyang University
 * @author soyeongkim@hanyang.ac.kr jiwonseok@hanyang.ac.kr
 * @file core.cpp
 * @brief
 * @version 1.0
 * @Fisrt release date 2024-12-18
 */

#include "core.hpp"

template <typename pointT>
ImagingRadarOdometryRun<pointT>::ImagingRadarOdometryRun(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      s_input_points_("velodyne_points"),
      b_accum_whole_point_clouds_(false),
      str_origin_("Map"),
      str_base_link_("base_link"),
      pc_radar_odom_submap_(new pcl::PointCloud<pcl::PointXYZI>),
      i_vod_frame_rate_(10),
      str_rcs_field_name_("RCS"),
      str_power_field_name_("power") {
    Initialization();
}

template <typename pointT>
bool ImagingRadarOdometryRun<pointT>::Initialization() {
    std::string odom_ini_path;
    std::string radar_ego_motion_ini_path;

    // Declare and get parameters from ROS2
    node_->declare_parameter<int>("i_vod_frame_rate", 10);
    node_->declare_parameter<std::string>("str_origin_", "Map");
    node_->declare_parameter<std::string>("str_base_link_", "base_link");
    node_->declare_parameter<std::string>("s_input_points_", "velodyne_points");
    node_->declare_parameter<std::string>("str_rcs_field_name_", "RCS");
    node_->declare_parameter<std::string>("str_power_field_name_", "power");
    node_->declare_parameter<bool>("b_accum_whole_point_clouds_", false);
    node_->declare_parameter<std::string>("str_odom_ini_path", "");
    node_->declare_parameter<std::string>("str_radar_ego_motion_ini_path", "");
    node_->declare_parameter<int>("min_static_points_num", 15);

    node_->get_parameter("i_vod_frame_rate", i_vod_frame_rate_);
    node_->get_parameter("str_origin_", str_origin_);
    node_->get_parameter("str_base_link_", str_base_link_);
    node_->get_parameter("s_input_points_", s_input_points_);
    node_->get_parameter("str_rcs_field_name_", str_rcs_field_name_);
    node_->get_parameter("str_power_field_name_", str_power_field_name_);
    node_->get_parameter("b_accum_whole_point_clouds_", b_accum_whole_point_clouds_);
    node_->get_parameter("str_odom_ini_path", odom_ini_path);
    node_->get_parameter("str_radar_ego_motion_ini_path", radar_ego_motion_ini_path);
    node_->get_parameter("min_static_points_num", i_min_static_points_num_);

    if (c_radar4motion_odom_.Init(odom_ini_path) == false) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "odom_ini_path registration initialization fail");
    }

    if (c_radar_ego_motion_vel_estimator_.Init(radar_ego_motion_ini_path) == false) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "radar_ego_motion_estimation initialization fail");
    }

    // Subscriber
    rossub_radar_pointcloud2_ =
            node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                s_input_points_, 10,
                std::bind(&ImagingRadarOdometryRun<pointT>::CallbackRadarPointCloud2, this, std::placeholders::_1));

    // Publishers
    rospub_radar_odom_pose_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/radar_odom_pose", 10);
    rospub_gt_odom_pose_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/gt_odom_pose", 10);
    rospub_radar_odom_pose_stamped_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/radar_odom_pose_stamped", 10);
    rospub_odom_accumulated_que_partial_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/odom_partial_map", 10);
    rospub_odom_accumulated_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/odom_accumulated_map", 1);
    rospub_static_radar_points_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/static_radar_points", 10);
    rospub_rcs_filtered_points_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/rcs_filtered_points", 10);
    rospub_vod_radar_points_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/vod_radar_points", 10);

    // TF2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    // Radar odometry param init
    vec3d_radar_ego_motion_init_vel_ = Eigen::Vector3d::Zero();
    vec3d_radar_ego_motion_init_vel_sigma_ = Eigen::Vector3d::Ones() * 100.0;
    affine3d_result_tf_ = Eigen::Affine3d::Identity();

    // Ini init
    if (odometry_ini_handler_.Init(odom_ini_path.c_str()) == false) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[RADAR ODOMETRY] INI FILE INIT FAIL");
    }
    else {
        RCLCPP_INFO_STREAM(node_->get_logger(), "[RADAR ODOMETRY] INI FILE INIT SUCCESS");
    }
    ParseINI();

    // Container init
    deque_transformed_points_.clear();
    pos_arr_odom_.poses.clear();
    vod_gt_arr_.poses.clear();

    RCLCPP_INFO_STREAM(node_->get_logger(), "[RADAR ODOMETRY] PoseArray Init");

    // VoD dataset
    // TODO: make config
    mat4d_vod_cam2radar_mat_ << -0.013857, -0.9997468, 0.01772762, 0.05283124, 0.10934269, -0.01913807, -0.99381983,
            0.98100483, 0.99390751, -0.01183297, 0.1095802, 1.44445002, 0.0, 0.0, 0.0, 1.0;

    return true;
}

template <typename pointT>
bool ImagingRadarOdometryRun<pointT>::ParseINI() {
    if (odometry_ini_handler_.IsFileUpdated() == false) {
        return true;
    }

    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "b_crop_points_",
                                          c_radar4motion_preprocessing_.b_crop_points_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "d_x_point_threshold_",
                                          c_radar4motion_preprocessing_.d_x_point_threshold_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "d_y_point_threshold_",
                                          c_radar4motion_preprocessing_.d_y_point_threshold_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "d_z_point_threshold_",
                                          c_radar4motion_preprocessing_.d_z_point_threshold_) != true)
        return false;

    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "b_local_rcs_filter_",
                                          c_radar4motion_preprocessing_.b_local_rcs_filter_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "i_local_rcs_num_",
                                          c_radar4motion_preprocessing_.i_local_rcs_num_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "d_local_rcs_radius_interval_m_",
                                          c_radar4motion_preprocessing_.d_local_rcs_radius_interval_m_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "d_local_rcs_azimuth_interval_deg_",
                                          c_radar4motion_preprocessing_.d_local_rcs_azimuth_interval_deg_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "d_local_rcs_elevation_interval_deg_",
                                          c_radar4motion_preprocessing_.d_local_rcs_elevation_interval_deg_) != true)
        return false;
    if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "b_local_rcs_normalization_",
                                          c_radar4motion_preprocessing_.b_local_rcs_normalization_) != true)
        if (odometry_ini_handler_.ParseConfig("PointCloudOdometry", "d_local_rcs_minimum_diff_",
                                              c_radar4motion_preprocessing_.d_local_rcs_minimum_diff_) != true)
            return false;

    return true;
}

template <typename pointT>
ImagingRadarOdometryRun<pointT>::~ImagingRadarOdometryRun() {
    // ROS2 subscription will be automatically cleaned up when the shared_ptr goes out of scope
    
    // Reset shared pointers
    pc_radar_odom_submap_.reset();

    // Clear any remaining container data
    deque_transformed_points_.clear();
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::RunOdometry() {
    if (b_radar_scan_exist_ == false) {
        return;
    }

    // 1. Radar ego motion estimation
    sensor_msgs::msg::PointCloud2 input_radar_msg;
    input_radar_msg = i_point_cloud_;

    sensor_msgs::msg::PointCloud2 inlier_radar_msg;
    RadarEgoMotionEstimation(input_radar_msg, inlier_radar_msg, vec3d_radar_ego_motion_init_vel_,
                             vec3d_radar_ego_motion_init_vel_sigma_);

    // 2. Preprocessing
    std::vector<std::pair<Eigen::Vector3d, double>> preprocessed_point_rcs_pairs;
    std::vector<std::pair<Eigen::Vector3d, double>> source_point_rcs_pairs =
            PointCloud2PointsRCSPairs(inlier_radar_msg);
    c_radar4motion_preprocessing_.Preprocessing(source_point_rcs_pairs, preprocessed_point_rcs_pairs);

    // 3. Run odometry
    double curr_time_sec = i_point_cloud_.header.stamp.sec + i_point_cloud_.header.stamp.nanosec / 1e9;
    Radar4MotionOdometry(curr_time_sec, preprocessed_point_rcs_pairs, vec3d_radar_ego_motion_init_vel_,
                         vec3d_radar_ego_motion_init_vel_sigma_);

    // 4. Visualization
    // 0. TF & Visualization
    BroadcastTFAndVisualizeOdomPose(affine3d_result_tf_, str_origin_, str_base_link_, i_point_cloud_.header);

    // map accumulation (for visualization, not for use in odometry)
    if (b_accum_whole_point_clouds_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr rcs_filtered_points(new pcl::PointCloud<pcl::PointXYZI>);
        EigenPointRCSToPointCloud(preprocessed_point_rcs_pairs, rcs_filtered_points);
        GlobalMapAccumulation(rcs_filtered_points, affine3d_result_tf_);
    }

    PublishSubmapPointCloud(pc_radar_odom_submap_);

    b_radar_scan_exist_ = false;
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::RadarEgoMotionEstimation(const sensor_msgs::msg::PointCloud2& input_radar_msg,
                                                               sensor_msgs::msg::PointCloud2& inlier_radar_msg,
                                                               Eigen::Vector3d& estimated_ego_motion,
                                                               Eigen::Vector3d& estimated_ego_motion_sigma) {
    // Ego motion vel estimator
    Eigen::Vector3d initial_velocity_for_ego_motion_estimation = estimated_ego_motion;
    sensor_msgs::msg::PointCloud2 outlier_msg;
    c_radar_ego_motion_vel_estimator_.estimateWithInitialVelocity(
            input_radar_msg, initial_velocity_for_ego_motion_estimation, estimated_ego_motion,
            estimated_ego_motion_sigma, inlier_radar_msg, outlier_msg);

    // Pub static points for debugging
    inlier_radar_msg.header.frame_id = i_point_cloud_.header.frame_id;
    inlier_radar_msg.header.stamp = i_point_cloud_.header.stamp;

    rospub_static_radar_points_->publish(inlier_radar_msg);

    // count inlier_radar_msg points
    int inlier_radar_msg_size = inlier_radar_msg.width * inlier_radar_msg.height;

    // Filtered static points num checking
    if (inlier_radar_msg_size < i_min_static_points_num_) {
        inlier_radar_msg = input_radar_msg;
        RCLCPP_WARN_STREAM(node_->get_logger(), "INSUFFICIENT # of STATIC POINTS!: " << inlier_radar_msg_size);
        // Prevent invalid value in ego motion estimation
        estimated_ego_motion_sigma = vec3d_radar_ego_motion_init_vel_sigma_;
    }
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::Radar4MotionOdometry(
        double time_sec, std::vector<std::pair<Eigen::Vector3d, double>>& i_point_rcs_pairs,
        Eigen::Vector3d& estimated_ego_motion, Eigen::Vector3d& estimated_ego_motion_sigma) {
    std::vector<Eigen::Vector3d> vec_sub_map;
    if (c_radar4motion_odom_.RunOdometry(time_sec, i_point_rcs_pairs, estimated_ego_motion,
                                         estimated_ego_motion_sigma)) {
        c_radar4motion_odom_.GetSubMap(vec_sub_map);
        EigenToPointCloud(vec_sub_map, pc_radar_odom_submap_);
    }
    else {
        RCLCPP_WARN_STREAM(node_->get_logger(), "ODOMETRY FAIL");
    }

    Eigen::Affine3d previous_result_tf = affine3d_result_tf_;
    c_radar4motion_odom_.GetPose(affine3d_result_tf_);

    if (pc_radar_odom_submap_ == NULL || pc_radar_odom_submap_->points.size() == 0) {
        RCLCPP_WARN_STREAM(node_->get_logger(),"SUBMAP EMPTY");
        return;
    }
}

template <typename pointT>
std::vector<std::pair<Eigen::Vector3d, double>> ImagingRadarOdometryRun<pointT>::PointCloud2PointsRCSPairs(
        const sensor_msgs::msg::PointCloud2& msg) {
    std::vector<std::pair<Eigen::Vector3d, double>> vec_point_rcs_pairs;

    bool has_x = false, has_y = false, has_z = false, has_rcs = false, has_power = false;
    std::string rcs_field_name = str_rcs_field_name_;
    std::string power_field_name = str_power_field_name_;
    for (const auto& field : msg.fields) {
        if (field.name == "x") has_x = true;
        if (field.name == "y") has_y = true;
        if (field.name == "z") has_z = true;
        if (field.name == str_rcs_field_name_) has_rcs = true;
        if (field.name == str_power_field_name_) has_power = true;
    }

    if (!has_x || !has_y || !has_z) {
        RCLCPP_ERROR(node_->get_logger(), "Required fields x, y, z are missing in the point cloud.");
        return vec_point_rcs_pairs;
    }

    if (!has_rcs) {
        for (const auto& field : msg.fields) {
            if (field.name == "radar_cross_section" || field.name == "RCS") {
                rcs_field_name = field.name;
                has_rcs = true;
                break;
            }
        }
    }

    if (!has_power) {
        for (const auto& field : msg.fields) {
            if (field.name == "signal_noise_ratio" || field.name == "power" || field.name == "snr") {
                power_field_name = field.name;
                has_power = true;
                break;
            }
        }
    }

    vec_point_rcs_pairs.reserve(msg.height * msg.width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(msg, "z");

    if (has_rcs == true) {
        sensor_msgs::PointCloud2ConstIterator<float> msg_rcs(msg, rcs_field_name);

        if (static_cast<double>(*msg_rcs) != 0.0) {
            for (size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z, ++msg_rcs) {
                vec_point_rcs_pairs.emplace_back(
                        std::make_pair(Eigen::Vector3d(*msg_x, *msg_y, *msg_z), static_cast<double>(*msg_rcs)));
            }
            return vec_point_rcs_pairs;
        }
    }

    // If RCS field does not exist, use power field
    RCLCPP_WARN_STREAM(node_->get_logger(),"The RCS field does not exist; falling back to power.");
    if (has_power) {
        sensor_msgs::PointCloud2ConstIterator<float> msg_power(msg, power_field_name);

        for (size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z, ++msg_power) {
            // Calculate rcs value
            double x = *msg_x;
            double y = *msg_y;
            double z = *msg_z;
            double power = *msg_power;

            double distance_m = sqrt(x * x + y * y);
            double point_rcs = pow(10, (power / 10)) * distance_m * distance_m * distance_m * distance_m;

            vec_point_rcs_pairs.emplace_back(std::make_pair(Eigen::Vector3d(*msg_x, *msg_y, *msg_z), point_rcs));
        }
    }
    else {
        RCLCPP_WARN_STREAM(node_->get_logger(),"The 'Power' field does not exist.");
        for (size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z) {
            vec_point_rcs_pairs.emplace_back(std::make_pair(Eigen::Vector3d(*msg_x, *msg_y, *msg_z), 0.0));
        }
    }

    return vec_point_rcs_pairs;
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::PublishSubmapPointCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& submap_point_cloud) {
    sensor_msgs::msg::PointCloud2 odom_map;
    pcl::toROSMsg(*submap_point_cloud, odom_map);
    odom_map.header.frame_id = str_origin_;
    odom_map.header.stamp = i_point_cloud_.header.stamp;

    rospub_odom_accumulated_que_partial_map_->publish(odom_map);
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::BroadcastTFAndVisualizeOdomPose(const Eigen::Affine3d& result_tf_in_ego_pose,
                                                                      const std::string& str_origin,
                                                                      const std::string& str_base_link,
                                                                      const std_msgs::msg::Header& point_cloud_header) {
    double odom_x, odom_y, odom_z, odom_roll, odom_pitch, odom_yaw = 0.0;
    pcl::getTranslationAndEulerAngles(result_tf_in_ego_pose, odom_x, odom_y, odom_z, odom_roll, odom_pitch, odom_yaw);

    tf2::Quaternion rpy2quat_odom;
    rpy2quat_odom.setRPY(odom_roll, odom_pitch, odom_yaw);

    geometry_msgs::msg::Transform transform_radar_to_map;
    transform_radar_to_map.translation.x = odom_x;
    transform_radar_to_map.translation.y = odom_y;
    transform_radar_to_map.translation.z = odom_z;
    transform_radar_to_map.rotation = tf2::toMsg(rpy2quat_odom);

    // Publish transform for radar to map
    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = point_cloud_header.stamp;
    stamped_transform.header.frame_id = str_origin;
    stamped_transform.child_frame_id = point_cloud_header.frame_id;
    stamped_transform.transform = transform_radar_to_map;
    tf_broadcaster_->sendTransform(stamped_transform);

    // Visualization
    VisualizeAndPubOdomPose(odom_x, odom_y, odom_z, rpy2quat_odom);
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::VisualizeAndPubOdomPose(double odom_x, double odom_y, double odom_z,
                                                              tf2::Quaternion odom_rpy) {
    geometry_msgs::msg::PoseStamped odom_pose_stamped;

    geometry_msgs::msg::Pose odom_pose;
    odom_pose.position.x = odom_x;
    odom_pose.position.y = odom_y;
    odom_pose.position.z = odom_z;

    odom_pose.orientation.x = odom_rpy.getX();
    odom_pose.orientation.y = odom_rpy.getY();
    odom_pose.orientation.z = odom_rpy.getZ();
    odom_pose.orientation.w = odom_rpy.getW();

    odom_pose_stamped.pose = odom_pose;

    odom_pose_stamped.header.stamp = i_point_cloud_.header.stamp;

    pos_arr_odom_.header.frame_id = str_origin_;
    pos_arr_odom_.poses.push_back(odom_pose);

    rospub_radar_odom_pose_stamped_->publish(odom_pose_stamped);
    rospub_radar_odom_pose_->publish(pos_arr_odom_);
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::GlobalMapAccumulation(
        const typename pcl::PointCloud<pointT>::Ptr input_point_cloud, Eigen::Affine3d& radar_pose) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*input_point_cloud, *pcl_transformed_cloud_ptr, radar_pose.matrix());
    deque_transformed_points_.push_front(pcl_transformed_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr accum_map(new pcl::PointCloud<pcl::PointXYZI>);

    // as accumulated point cloud
    for (int i = 0; i < deque_transformed_points_.size(); i++) {
        *accum_map += *deque_transformed_points_[i];
    }

    Downsampling(accum_map, accum_map, 0.5);
    pcl::transformPointCloud(*accum_map, *accum_map, radar_pose.inverse());

    sensor_msgs::msg::PointCloud2 odom_total_map;
    pcl::toROSMsg(*accum_map, odom_total_map);
    odom_total_map.header.frame_id = i_point_cloud_.header.frame_id;
    odom_total_map.header.stamp = i_point_cloud_.header.stamp;

    rospub_odom_accumulated_map_->publish(odom_total_map);
}

template <typename pointT>
bool ImagingRadarOdometryRun<pointT>::Downsampling(typename pcl::PointCloud<pointT>::Ptr input_point_cloud,
                                                   typename pcl::PointCloud<pointT>::Ptr& output_point_cloud,
                                                   double leaf_size) {
    if (input_point_cloud == NULL) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Downsampling] Input points are empty!");
        return false;
    }

    if (input_point_cloud->points.size() == 0) {
        return false;
    }

    // Point cloud preprocessing
    pcl::VoxelGrid<pointT> downsampling_filter;
    downsampling_filter.setInputCloud(input_point_cloud);
    downsampling_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    downsampling_filter.filter(*output_point_cloud);

    return true;
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::CallbackRadarPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    i_point_cloud_ = *msg;
    b_radar_scan_exist_ = true;
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::ProcessRadarFiles() {
    std::string vod_eval_folder_path;
    node_->declare_parameter<std::string>("vod_eval_folder_path", "");
    node_->declare_parameter<std::string>("str_vod_dataset_base_path", "/dataset/");
    node_->declare_parameter<std::string>("str_radar4motion_path", "/dataset/");
    node_->declare_parameter<int>("scenario_num", 1);

    node_->get_parameter("vod_eval_folder_path", vod_eval_folder_path);

    std::string str_vod_dataset_base_path;
    node_->get_parameter("str_vod_dataset_base_path", str_vod_dataset_base_path);

    std::string str_radar4motion_path;
    node_->get_parameter("str_radar4motion_path", str_radar4motion_path);

    int scenario_num;
    node_->get_parameter("scenario_num", scenario_num);

    // Check folder path slash
    if (vod_eval_folder_path.back() != '/') {
        vod_eval_folder_path += "/";
    }

    if (str_vod_dataset_base_path.back() != '/') {
        str_vod_dataset_base_path += "/";
    }

    std::string vod_radar_base_path = str_vod_dataset_base_path + "view_of_delft_PUBLIC/radar/training/";

    // Check folder path with file system
    if (std::filesystem::exists(vod_eval_folder_path) == false) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Invalid vod_eval_folder_path: " << vod_eval_folder_path);
        return;
    }

    // Construct the sequence file path
    std::string sequence_file_path =
            str_radar4motion_path + "/dataset/clips/delft_" + std::to_string(scenario_num) + ".txt";

    // Read the sequence file to get sequence numbers
    std::vector<std::string> scenario_numbers;
    if (!c_vod_parser_.ReadSequenceFile(sequence_file_path, scenario_numbers)) {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"Failed to read sequence file: " << sequence_file_path);
        return;
    }
    else {
        RCLCPP_INFO_STREAM(node_->get_logger(),"Read sequence file: " << sequence_file_path);
    }

    // Read whole folder root in vod_seq_folder_path
    ProcessRadarFile(vod_radar_base_path, vod_eval_folder_path, scenario_numbers, scenario_num);
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::ProcessRadarFile(const std::string& vod_seq_folder_path,
                                                       const std::string& vod_eval_folder_path,
                                                       const std::vector<std::string>& scenario_numbers,
                                                       const int scenario_num) {
    std::string pose_directory_path = vod_seq_folder_path + "pose/";
    std::string radar_directory_path = vod_seq_folder_path + "velodyne/";

    // VoD camera to radar transformation matrix setup
    Eigen::Affine3d cam2radar_transform(mat4d_vod_cam2radar_mat_);

    // Parse pose JSON files
    std::vector<std::filesystem::path> pose_files;
    c_vod_parser_.GetPoseJsonFilesWithSeq(pose_directory_path, scenario_numbers, pose_files);

    std::vector<Eigen::Affine3d> gt_odom2radar_transforms;
    c_vod_parser_.ParsePoseJsonFiles(pose_files, cam2radar_transform, gt_odom2radar_transforms);
    std::cout << "JSON File Read Complete" << std::endl;

    // Process .bin files from sequence numbers
    std::vector<pcl::PointCloud<RadarPoint>::Ptr> clouds;
    std::vector<Eigen::Affine3d> estimate_odom2radar_transforms;

    int idx = 0;
    for (const auto& seq : scenario_numbers) {
        std::string bin_file_path = radar_directory_path + seq + ".bin";
        std::ifstream file(bin_file_path, std::ios::binary);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << bin_file_path << std::endl;
            continue;
        }

        // Convert the .bin file to a point cloud
        pcl::PointCloud<RadarPoint>::Ptr cloud(new pcl::PointCloud<RadarPoint>);
        c_vod_parser_.ConvertBin2PointCloud(file, cloud, std::stoi(seq), i_point_cloud_);
        // Publish for visualization
        rospub_vod_radar_points_->publish(i_point_cloud_);
        clouds.push_back(cloud);

        // Publish ground truth poses
        if (gt_odom2radar_transforms.size() > 0) {
            Eigen::Affine3d odom2radar = gt_odom2radar_transforms[idx++];
            PubVodGtPose(odom2radar, i_point_cloud_.header);
        }

        // Run odometry for each .bin file
        b_radar_scan_exist_ = true;
        RunOdometry();
        estimate_odom2radar_transforms.push_back(affine3d_result_tf_);

        // ros spin 10hz
        if (rclcpp::ok()) {
            rclcpp::Rate loop_rate(i_vod_frame_rate_);
            loop_rate.sleep();
        }

        file.close();
    }

    // Save estimated poses to a file
    std::string curr_folder_seq = vod_eval_folder_path + "seq" + std::to_string(scenario_num);
    SavePosesToFile(estimate_odom2radar_transforms, curr_folder_seq + "_estimate_poses.txt");
    SavePosesToFile(gt_odom2radar_transforms, curr_folder_seq + "_gt_poses.txt");
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::SavePosesToFile(const std::vector<Eigen::Affine3d>& poses,
                                                      const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    for (const auto& pose : poses) {
        Eigen::Matrix4d matrix = pose.matrix();
        // Write the first three rows of the 4x4 matrix
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                file << matrix(row, col);
                if (row != 2 || col != 3) {
                    file << " ";
                }
            }
        }
        file << "\n";
    }

    file.close();
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::PubVodGtPose(const Eigen::Affine3d& vod_gt_pose,
                                                   const std_msgs::msg::Header& point_cloud_header) {
    geometry_msgs::msg::Pose vod_gt_pose_msg;
    vod_gt_pose_msg.position.x = vod_gt_pose.translation().x();
    vod_gt_pose_msg.position.y = vod_gt_pose.translation().y();
    vod_gt_pose_msg.position.z = vod_gt_pose.translation().z();

    // Convert rotation matrix to quaternion
    Eigen::Quaterniond quat(vod_gt_pose.rotation());
    vod_gt_pose_msg.orientation.x = quat.x();
    vod_gt_pose_msg.orientation.y = quat.y();
    vod_gt_pose_msg.orientation.z = quat.z();
    vod_gt_pose_msg.orientation.w = quat.w();

    vod_gt_arr_.header.frame_id = str_origin_;
    vod_gt_arr_.header.stamp = point_cloud_header.stamp;
    vod_gt_arr_.poses.push_back(vod_gt_pose_msg);

    // For rviz visualization
    rospub_gt_odom_pose_->publish(vod_gt_arr_);
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::EigenPointRCSToPointCloud(
        const std::vector<std::pair<Eigen::Vector3d, double>>& vec_point_rcs_pairs,
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_point_cloud) {
    output_point_cloud->points.clear();

    for (size_t idx = 0; idx < vec_point_rcs_pairs.size(); idx++) {
        pcl::PointXYZI pclPoint;
        pclPoint.x = vec_point_rcs_pairs[idx].first(0);
        pclPoint.y = vec_point_rcs_pairs[idx].first(1);
        pclPoint.z = vec_point_rcs_pairs[idx].first(2);
        pclPoint.intensity = vec_point_rcs_pairs[idx].second;

        output_point_cloud->points.push_back(pclPoint);
    }
    output_point_cloud->width = static_cast<uint32_t>(output_point_cloud->points.size());
    output_point_cloud->height = 1;
    output_point_cloud->is_dense = true;

    // Publish
    sensor_msgs::msg::PointCloud2 power_filtered_pc2;
    pcl::toROSMsg(*output_point_cloud, power_filtered_pc2);
    power_filtered_pc2.header.frame_id = i_point_cloud_.header.frame_id;
    power_filtered_pc2.header.stamp = i_point_cloud_.header.stamp;

    rospub_rcs_filtered_points_->publish(power_filtered_pc2);
}

template <typename pointT>
void ImagingRadarOdometryRun<pointT>::EigenToPointCloud(
        const std::vector<Eigen::Vector3d>& vec_point_cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_point_cloud) {
    output_point_cloud->points.clear();

    for (auto& point : vec_point_cloud) {
        pcl::PointXYZI pclPoint;
        pclPoint.x = point(0);
        pclPoint.y = point(1);
        pclPoint.z = point(2);
        pclPoint.intensity = 1.0; // dummy

        output_point_cloud->points.push_back(pclPoint);
    }
    output_point_cloud->width = static_cast<uint32_t>(output_point_cloud->points.size());
    output_point_cloud->height = 1;
    output_point_cloud->is_dense = true;
}
