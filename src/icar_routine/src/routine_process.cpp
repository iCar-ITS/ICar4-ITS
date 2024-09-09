#include "icar_routine/routine.h"

void process_marker()
{
    {
        geometry_msgs::Point p;

        static const float tyre_diameter = icar_tyre_rim * 0.0254 + icar_tyre_width * icar_tyre_aspect_ratio * 0.00002;
        static const float tyre_width = icar_tyre_width * 0.001;

        _marker.cube("body_link", "body", 1,
                     p, tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0),
                     1.0, 1.0, 1.0, 0.5,
                     icar_body_length, icar_body_width, icar_body_height);

        p.y = icar_body_width / 2 - tyre_width / 2;
        _marker.cylinder("front_axle_link", "tyre", 1,
                         p, tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0),
                         0.0, 0.0, 0.0, 0.5,
                         tyre_diameter, tyre_diameter, tyre_width);
        _marker.cylinder("rear_axle_link", "tyre", 2,
                         p, tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0),
                         0.0, 0.0, 0.0, 0.5,
                         tyre_diameter, tyre_diameter, tyre_width);

        p.y = -icar_body_width / 2 + tyre_width / 2;
        _marker.cylinder("front_axle_link", "tyre", 3,
                         p, tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0),
                         0.0, 0.0, 0.0, 0.5,
                         tyre_diameter, tyre_diameter, tyre_width);
        _marker.cylinder("rear_axle_link", "tyre", 4,
                         p, tf::createQuaternionMsgFromRollPitchYaw(M_PI_2, 0, 0),
                         0.0, 0.0, 0.0, 0.5,
                         tyre_diameter, tyre_diameter, tyre_width);
    }

    {
        static std::vector<geometry_msgs::Point> ps;
        static geometry_msgs::Point p;

        static geometry_msgs::Pose2D last_pose_pose2d;

        if (RADIUS(pose_pose2d, last_pose_pose2d) > 0.1)
        {
            if (ps.size() > 25000)
                ps.erase(ps.begin());

            p.x = last_pose_pose2d.x = pose_pose2d.x;
            p.y = last_pose_pose2d.y = pose_pose2d.y;
            ps.push_back(p);

            _marker.line_strip("map", "pose", 1, ps, 0.84, 0.07, 0.33, 0.50, 0.05);
        }
    }

    {
        _marker.line_strip("map", "route", 1, route_active, 0.98, 0.29, 0.16, 0.50, 0.05);
        _marker.line_strip("map", "route", 2, route_limit_kanan, 0.98, 0.58, 0.32, 0.50, 0.05);
        _marker.line_strip("map", "route", 3, route_limit_kiri, 0.98, 0.58, 0.32, 0.50, 0.05);
    }

    {
        std::vector<geometry_msgs::Point> ps;
        geometry_msgs::Point p;

        float angle_start = -M_PI_2;
        float angle_stop = -M_PI_2 + (10 / pp_active.icrRadius);
        float angle_step = (angle_stop - angle_start) / 100;

        ps.clear();
        for (int i = 0; i < 101; i += 2)
        {
            p.x = pp_active.lookAheadDistance * cos(M_PI / 50 * i);
            p.y = pp_active.lookAheadDistance * sin(M_PI / 50 * i);
            ps.push_back(p);
        }
        _marker.line_strip("base_link", "pp", 1, ps, 0.25, 0.32, 0.23, 0.50, 0.1);

        p.x = pp_active.goalPositionX;
        p.y = pp_active.goalPositionY;
        _marker.sphere("map", "pp", 2, p, tf::createQuaternionMsgFromYaw(0), 0.38, 0.60, 0.40, 1.00, 0.5, 0.5, 0.5);

        p.x = pp_active.icrPositionX;
        p.y = pp_active.icrPositionY;
        _marker.sphere("map", "pp", 3, p, tf::createQuaternionMsgFromYaw(0), 0.62, 0.75, 0.55, 1.00, 0.5, 0.5, 0.5);

        if (fabsf(pp_active.icrRadius) > __FLT_EPSILON__)
        {
            ps.clear();
            for (int i = 0; i < 101; i++)
            {
                p.x = pp_active.icrRadius * cosf(angle_start + angle_step * i);
                p.y = pp_active.icrRadius * sinf(angle_start + angle_step * i) + pp_active.icrRadius;
                ps.push_back(p);
            }
            _marker.line_strip("base_link", "pp", 4, ps, 0.93, 0.95, 0.84, 0.50, 0.05);
        }

        if (route_active.size() > 0)
            _marker.sphere("map", "pp", 5, route_active[pp_active.indexNearest], tf::createQuaternionMsgFromYaw(0), 1.0, 1.0, 0.0, 0.5, 0.5, 0.5, 0.5);
    }

    {
        std::vector<geometry_msgs::Point> ps;
        geometry_msgs::Point p;

        static const float angle_start = -M_PI;
        static const float angle_stop = M_PI;
        static const float angle_step = (angle_stop - angle_start) / 100;

        ps.clear();
        for (int i = 0; i < actions_main.size(); i++)
        {
            for (int j = 1; j < 101; j++)
            {
                p.x = actions_main[i].x + actions_main[i].radius * cosf(angle_start + angle_step * j);
                p.y = actions_main[i].y + actions_main[i].radius * sinf(angle_start + angle_step * j);
                ps.push_back(p);
                p.x = actions_main[i].x + actions_main[i].radius * cosf(angle_start + angle_step * (j - 1));
                p.y = actions_main[i].y + actions_main[i].radius * sinf(angle_start + angle_step * (j - 1));
                ps.push_back(p);
            }
        }
        _marker.line_list("map", "actions", 1, ps, 0.68, 0.48, 0.91, 1.0, 0.05);

        ps.clear();
        for (int i = 0; i < actions_accessory.size(); i++)
        {
            for (int j = 1; j < 101; j++)
            {
                p.x = actions_accessory[i].x + actions_accessory[i].radius * cosf(angle_start + angle_step * j);
                p.y = actions_accessory[i].y + actions_accessory[i].radius * sinf(angle_start + angle_step * j);
                ps.push_back(p);
                p.x = actions_accessory[i].x + actions_accessory[i].radius * cosf(angle_start + angle_step * (j - 1));
                p.y = actions_accessory[i].y + actions_accessory[i].radius * sinf(angle_start + angle_step * (j - 1));
                ps.push_back(p);
            }
        }
        _marker.line_list("map", "actions", 2, ps, 0.24, 0.33, 0.67, 1.0, 0.05);
    }

    {
        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = icar_body_height;

        std::stringstream information;
        information << "index: " << std::fixed << std::setprecision(0) << pp_active.indexNearest << std::endl;
        information << "x: " << std::fixed << std::setprecision(2) << pose_pose2d.x << " m" << std::endl;
        information << "y: " << std::fixed << std::setprecision(2) << pose_pose2d.y << " m" << std::endl;
        information << "yaw: " << std::fixed << std::setprecision(2) << pose_pose2d.theta * 180 / M_PI << " deg" << std::endl;

        _marker.text_view_facing("body_link", "information", 1, p, tf::createQuaternionMsgFromYaw(0), 1.0, 1.0, 1.0, 1.0, 0.5, information.str());
    }
}

void process_metric()
{
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();

    static float last_metric_velocity = 0.0;
    static float last_metric_acceleration = 0.0;

    /* Calculate delta value of encoder per 0.02 second. */
    int16_t d_encoder_kiri;
    int16_t d_encoder_kanan;
    encoder_2_d_encoder(encoder_kiri, encoder_kanan, &d_encoder_kiri, &d_encoder_kanan);

    /* Calculate delta value of gyroscope per 0.02 second. */
    float d_gyroscope;
    gyroscope_2_d_gyroscope(gyroscope, &d_gyroscope);

    /* Calculate delta value of distance per 0.02 second. */
    static float d_distance;
    d_distance += (d_encoder_kiri + d_encoder_kanan) / 2 * odometry_to_meter;

    if (current_time - last_time > ros::Duration(0.1))
    {
        last_time = current_time;
        last_metric_velocity = metric_velocity;
        last_metric_acceleration = metric_acceleration;

        metric_velocity = 0.85 * metric_velocity + 0.15 * d_distance / 0.1;
        metric_acceleration = (metric_velocity - last_metric_velocity) / 0.1;
        metric_jerk = (metric_acceleration - last_metric_acceleration) / 0.1;

        d_distance = 0.0;
    }
}

void process_ui()
{
    ui_input_data.throttle = action_throttle;
    ui_input_data.brake = action_brake;
    ui_input_data.velocity_output = fabsf(metric_velocity) * 3.6;
    ui_input_data.velocity_input = fabsf(action_velocity) * 3.6;
    ui_input_data.steering_output = steering_position * -180.0 / M_PI;
    ui_input_data.steering_input = action_steering * -180.0 / M_PI;
    pub_ui_input_data.publish(ui_input_data);
}

//------------------------------------------------------------------------------
//==============================================================================

void process_record_and_load()
{
    static std::ofstream route_output;

    switch (status_algoritma_record)
    {
    case 10:
    {
        boost::posix_time::ptime time_ptime = boost::posix_time::microsec_clock::local_time();
        std::string time_string = boost::posix_time::to_simple_string(time_ptime);

        // -----------------------------

        route_output.open(route_path + "/" + time_string.substr(0, 20) + "_main.csv");
        route_output << "x,y,radius,action_type,obstacle_status,laser_scan_distance,look_ahead_distance,target_velocity,comment" << std::endl;
        route_output.close();
        route_output.open(route_path + "/" + time_string.substr(0, 20) + "_accessory.csv");
        route_output << "x,y,radius,action_type,leftsignal,rightsignal,headlight,r,g,b,comment" << std::endl;
        route_output.close();

        // -----------------------------

        route_output.open(route_path + "/" + time_string.substr(0, 20) + ".csv");
        route_output << "x,y" << std::endl;

        // -----------------------------

        _log.warn("Record route: start");
        status_algoritma_record = 11;
        break;
    }

    case 11:
    {
        static geometry_msgs::Pose2D last_pose_pose2d;

        if (RADIUS(pose_pose2d, last_pose_pose2d) > 0.1)
        {
            route_output << pose_pose2d.x << "," << pose_pose2d.y << std::endl;

            last_pose_pose2d.x = pose_pose2d.x;
            last_pose_pose2d.y = pose_pose2d.y;
        }

        break;
    }

    case 20:
    {
        route_output.close();

        // -----------------------------

        _log.warn("Record route: stop");
        status_algoritma_record = 0;
        break;
    }
    }

    // =================================

    switch (status_algoritma_load)
    {
    case 10:
    {
        std::vector<std::string> files;

        // -----------------------------

        for (boost::filesystem::directory_iterator itr(route_path); itr != boost::filesystem::directory_iterator(); ++itr)
        {
            if (boost::filesystem::is_regular_file(itr->path()))
            {
                std::string filename = itr->path().filename().string();
                if (filename.find(".csv") != std::string::npos)
                    files.push_back(filename);
            }
        }

        for (int i = 0; i < files.size(); i++)
        {
            if (files[i].find("_main.csv") != std::string::npos)
                files.erase(files.begin() + i--);
            else if (files[i].find("_accessory.csv") != std::string::npos)
                files.erase(files.begin() + i--);
        }

        for (int i = 0; i < files.size(); i++)
            files[i] = files[i].substr(0, files[i].find(".csv"));

        // -----------------------------

        ui_input_data.route_option = "";
        for (int i = 0; i < files.size(); i++)
            ui_input_data.route_option += files[i] + "\n";
        ui_input_data.route_option.erase(ui_input_data.route_option.end() - 1);

        // -----------------------------

        _log.warn("Load route: refresh");
        status_algoritma_load = 0;
        break;
    }

    case 20:
    {
        std::ofstream file_output;
        std::ifstream file_input;

        // -----------------------------

        if (!boost::filesystem::exists(route_path + "/default.csv"))
        {
            file_output.open(route_path + "/default.csv");
            file_output << "x,y" << std::endl;
            file_output.close();
        }
        if (!boost::filesystem::exists(route_path + "/default_main.csv"))
        {
            file_output.open(route_path + "/default_main.csv");
            file_output << "x,y,radius,action_type,obstacle_status,laser_scan_distance,look_ahead_distance,target_velocity,comment" << std::endl;
            file_output.close();
        }
        if (!boost::filesystem::exists(route_path + "/default_accessory.csv"))
        {
            file_output.open(route_path + "/default_accessory.csv");
            file_output << "x,y,radius,action_type,leftsignal,rightsignal,headlight,r,g,b,comment" << std::endl;
            file_output.close();
        }
        if (ui_output_data.route_option != "default")
        {
            boost::filesystem::copy_file(route_path + "/" + ui_output_data.route_option + ".csv", route_path + "/default.csv", boost::filesystem::copy_option::overwrite_if_exists);
            boost::filesystem::copy_file(route_path + "/" + ui_output_data.route_option + "_main.csv", route_path + "/default_main.csv", boost::filesystem::copy_option::overwrite_if_exists);
            boost::filesystem::copy_file(route_path + "/" + ui_output_data.route_option + "_accessory.csv", route_path + "/default_accessory.csv", boost::filesystem::copy_option::overwrite_if_exists);
        }

        // -----------------------------

        route_active.clear();
        file_input.open(route_path + "/" + ui_output_data.route_option + ".csv");
        file_input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (file_input.good())
        {
            if (file_input.peek() == EOF)
                break;

            geometry_msgs::Point p;
            file_input >> p.x;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> p.y;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            route_active.push_back(p);
        }
        file_input.close();

        // -----------------------------

        route_limit_kanan.clear();
        route_limit_kiri.clear();
        for (int i = 1; i < route_active.size(); i++)
        {
            geometry_msgs::Point origin = route_active[i - 1];
            geometry_msgs::Point target = route_active[i];

            // Rotate 90 degree and -90 degree
            double x = target.x - origin.x;
            double y = target.y - origin.y;
            double x_kiri = -y;
            double y_kiri = x;
            double x_kanan = y;
            double y_kanan = -x;

            // Normalize and multiply by some value
            double magnitude_kanan = sqrt(pow(x_kanan, 2) + pow(y_kanan, 2));
            double magnitude_kiri = sqrt(pow(x_kiri, 2) + pow(y_kiri, 2));
            x_kanan = (x_kanan / magnitude_kanan) * 2.0;
            y_kanan = (y_kanan / magnitude_kanan) * 2.0;
            x_kiri = (x_kiri / magnitude_kiri) * 0.5;
            y_kiri = (y_kiri / magnitude_kiri) * 0.5;

            // Add to vector
            geometry_msgs::Point p_kanan;
            p_kanan.x = target.x + x_kanan;
            p_kanan.y = target.y + y_kanan;
            route_limit_kanan.push_back(p_kanan);
            geometry_msgs::Point p_kiri;
            p_kiri.x = target.x + x_kiri;
            p_kiri.y = target.y + y_kiri;
            route_limit_kiri.push_back(p_kiri);
        }

        // -----------------------------

        actions_main.clear();
        file_input.open(route_path + "/" + ui_output_data.route_option + "_main.csv");
        file_input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (file_input.good())
        {
            if (file_input.peek() == EOF)
                break;

            action_main action;
            file_input >> action.x;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.y;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.radius;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.action_type;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.obstacle_status;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.laser_scan_distance;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.look_ahead_distance;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.target_velocity;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.comment;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            actions_main.push_back(action);
        }
        file_input.close();

        // -----------------------------

        actions_accessory.clear();
        file_input.open(route_path + "/" + ui_output_data.route_option + "_accessory.csv");
        file_input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (file_input.good())
        {
            if (file_input.peek() == EOF)
                break;

            action_accessory action;
            file_input >> action.x;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.y;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.radius;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.action_type;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.leftsignal;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.rightsignal;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.headlight;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.r;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.g;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.b;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file_input >> action.comment;
            file_input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            actions_accessory.push_back(action);
        }
        file_input.close();

        // -----------------------------

        _log.warn("Load route: open");
        status_algoritma_load = 0;
        break;
    }
    }
}

void process_mission()
{
    const static float default_look_ahead_distance = 7.5;  // 14.0m
    const static float default_target_velocity = 2.5;      // 3.5m/s
    const static float default_laser_scan_distance = 10.0; // 14.0m

    static float prev_active_look_ahead_distance = default_look_ahead_distance;
    static float prev_active_target_velocity = default_target_velocity;
    static float prev_active_laser_scan_distance = default_laser_scan_distance;
    static bool prev_obstacle_status_steering;
    static bool prev_obstacle_status_velocity;

    static ros::Time time_now;
    static ros::Time time_last;
    time_last = time_now;
    time_now = ros::Time::now();
    ros::Duration dt = time_now - time_last;

    //----------------------------------

    static ros::Time time_action;
    static float active_look_ahead_distance = default_look_ahead_distance;
    static float active_target_velocity = default_target_velocity;
    static float active_laser_scan_distance = default_laser_scan_distance;

    //----------------------------------

    static float active_look_ahead_distance_buffer;
    if (active_look_ahead_distance_buffer < active_look_ahead_distance)
    {
        active_look_ahead_distance_buffer += 1.0 * dt.toSec();
        if (active_look_ahead_distance_buffer > active_look_ahead_distance)
            active_look_ahead_distance_buffer = active_look_ahead_distance;
    }
    else if (active_look_ahead_distance_buffer > active_look_ahead_distance)
    {
        active_look_ahead_distance_buffer -= 1.0 * dt.toSec();
        if (active_look_ahead_distance_buffer < active_look_ahead_distance)
            active_look_ahead_distance_buffer = active_look_ahead_distance;
    }

    pp_active.setLookAheadDistance(active_look_ahead_distance_buffer);
    pp_limit_kanan.setLookAheadDistance(1.38 * icar_wheelbase);
    pp_limit_kiri.setLookAheadDistance(2.62 * icar_wheelbase);
    float steering_active = pp_active.steeringAngle;
    float steering_limit_kanan = pp_limit_kanan.steeringAngle;
    float steering_limit_kiri = pp_limit_kiri.steeringAngle;

    //----------------------------------

    static float active_laser_scan_distance_buffer;
    if (active_laser_scan_distance_buffer < active_laser_scan_distance)
    {
        active_laser_scan_distance_buffer += 1.0 * dt.toSec();
        if (active_laser_scan_distance_buffer > active_laser_scan_distance)
            active_laser_scan_distance_buffer = active_laser_scan_distance;
    }
    else if (active_laser_scan_distance_buffer > active_laser_scan_distance)
    {
        active_laser_scan_distance_buffer -= 1.0 * dt.toSec();
        if (active_laser_scan_distance_buffer < active_laser_scan_distance)
            active_laser_scan_distance_buffer = active_laser_scan_distance;
    }

    obstacle_1_parameter.laser_scan_distance = active_laser_scan_distance_buffer;

    //----------------------------------

    float steering_after_obstacle = (obstacle_1_data.output_steering_to_right * -0.81 + obstacle_1_data.output_steering_to_left * 0.19) * (1 - obstacle_2_data.output_blindspot) +
                                    steering_active;
    float velocity_after_obstacle = (1 - obstacle_1_data.output_velocity_to_right * 1.0) *
                                    (1 - obstacle_1_data.output_velocity_to_left * 0.0) *
                                    (1 - obstacle_1_data.output_velocity_following * 1.0) *
                                    (1 - obstacle_1_data.output_emergency * 1.0) *
                                    active_target_velocity;

    //----------------------------------

    float slow_acceleration = (obstacle_1_data.output_steering_to_right * 1.0 +
                               obstacle_1_data.output_steering_to_left * 0.0 +
                               obstacle_1_data.output_velocity_following * 1.0 +
                               obstacle_1_data.output_emergency * 1.0) /
                              (1.0 + 0.0 + 1.0 + 1.0);

    float slow_deceleration = 0.0;

    //----------------------------------

    static unsigned int prev_status_algoritma_mission = 0;
    if (status_algoritma_mission != prev_status_algoritma_mission)
    {
        _log.debug("Status Algoritma: %d | Look Ahead: %.2fm | Target Velocity: %.2fm/s | Laser Scan: %.2fm | Obstacle Steering: %d | Obstacle Velocity: %d",
                   status_algoritma_mission,
                   active_look_ahead_distance,
                   active_target_velocity,
                   active_laser_scan_distance,
                   obstacle_1_parameter.obstacle_status_steering,
                   obstacle_1_parameter.obstacle_status_velocity);

        play_sound(1, icar_ui::ui_output_sound::ACTION_PLAY_ONCE, "notification.wav");
    }
    prev_status_algoritma_mission = status_algoritma_mission;

    //----------------------------------

    switch (status_algoritma_mission)
    {
    case 0:
    {
        jalan_maju_setir_otomatis(0, steering_active);

        //----------------------------------

        accessory_bitwise_op(accessory, 0, 1); // B
        accessory_bitwise_op(accessory, 1, 1); // G
        accessory_bitwise_op(accessory, 2, 1); // R
        accessory_bitwise_op(accessory, 3, 1); // Headlight

        break;
    }

    case 1:
    {
        int action_index = is_inside_action_main(0);
        if (action_index != -1)
        {
            switch (actions_main[action_index].action_type)
            {
            case 0:
                status_algoritma_mission = 1;
                break;
            case 1:
                status_algoritma_mission = 4;
                prev_active_look_ahead_distance = active_look_ahead_distance;
                prev_active_target_velocity = active_target_velocity;
                prev_active_laser_scan_distance = active_laser_scan_distance;
                prev_obstacle_status_steering = obstacle_1_parameter.obstacle_status_steering;
                prev_obstacle_status_velocity = obstacle_1_parameter.obstacle_status_velocity;
                break;
            case 2:
                status_algoritma_mission = 2;
                break;
            default:
                status_algoritma_mission = 1;
                break;
            }

            active_look_ahead_distance = actions_main[action_index].look_ahead_distance;
            active_target_velocity = actions_main[action_index].target_velocity;
            active_laser_scan_distance = actions_main[action_index].laser_scan_distance;
            obstacle_1_parameter.obstacle_status_steering = (actions_main[action_index].obstacle_status & 0x01) >> 0;
            obstacle_1_parameter.obstacle_status_velocity = (actions_main[action_index].obstacle_status & 0x02) >> 1;
            time_action = ros::Time::now();
        }

        // -----------------------------

        jalan_maju_setir_otomatis(velocity_after_obstacle,
                                  fmaxf(steering_limit_kanan, fminf(steering_limit_kiri, steering_after_obstacle)),
                                  slow_acceleration,
                                  slow_deceleration,
                                  obstacle_1_data.output_emergency);

        // -----------------------------

        accessory_bitwise_op(accessory, 0, 0); // B
        accessory_bitwise_op(accessory, 1, 0); // G
        accessory_bitwise_op(accessory, 2, 1); // R

        break;
    }

    case 2:
    {
        // --------------------------------------------------------
        // This state is to make car stop for 30 seconds.
        // If car stop for 30 seconds, then car will go to state 3.
        // --------------------------------------------------------

        accessory_bitwise_op(accessory, 4, 1); // Right sign
        accessory_bitwise_op(accessory, 5, 1); // Left sign

        if (ros::Time::now() - time_action > ros::Duration(30.0))
        {
            status_algoritma_mission = 3;
            accessory_bitwise_op(accessory, 4, 0); // Right sign
            accessory_bitwise_op(accessory, 5, 0); // Left sign
        }

        // -----------------------------

        bool is_stopped;
        bool is_emergency;
        jalan_maju_setir_otomatis(0, steering_active, 0, 0, 0, &is_stopped, &is_emergency);

        // -----------------------------

        if (is_emergency)
        {
            accessory_bitwise_op(accessory, 0, 0); // B
            accessory_bitwise_op(accessory, 1, 1); // G
            accessory_bitwise_op(accessory, 2, 0); // R
        }
        else
        {
            accessory_bitwise_op(accessory, 0, 0); // B
            accessory_bitwise_op(accessory, 1, 1); // G
            accessory_bitwise_op(accessory, 2, 1); // R
        }

        break;
    }

    case 3:
    {
        // -----------------------------------------------------------
        // This state is to make car move outside stop area.
        // If car move outside stop area, then car will go to state 1.
        // -----------------------------------------------------------

        if (is_inside_action_main(1.0) == -1)
        {
            status_algoritma_mission = 1;
        }

        // -----------------------------

        jalan_maju_setir_otomatis(default_target_velocity,
                                  fmaxf(steering_limit_kanan, fminf(steering_limit_kiri, steering_after_obstacle)),
                                  slow_acceleration,
                                  slow_deceleration,
                                  obstacle_1_data.output_emergency);

        // -----------------------------

        accessory_bitwise_op(accessory, 0, 0); // B
        accessory_bitwise_op(accessory, 1, 0); // G
        accessory_bitwise_op(accessory, 2, 1); // R

        break;
    }

    case 4:
    {
        // -------------------------------------------------------------
        // This state is to make car using defined navigation parameter
        // while inside area. The parameter will be restored to previous
        // parameter when car move outside defined navigation area.
        // -------------------------------------------------------------

        if (is_inside_action_main(1.0) == -1)
        {
            status_algoritma_mission = 1;
            active_look_ahead_distance = prev_active_look_ahead_distance;
            active_target_velocity = prev_active_target_velocity;
            active_laser_scan_distance = prev_active_laser_scan_distance;
            obstacle_1_parameter.obstacle_status_steering = prev_obstacle_status_steering;
            obstacle_1_parameter.obstacle_status_velocity = prev_obstacle_status_velocity;
        }

        // -----------------------------

        jalan_maju_setir_otomatis(velocity_after_obstacle,
                                  fmaxf(steering_limit_kanan, fminf(steering_limit_kiri, steering_after_obstacle)),
                                  slow_acceleration,
                                  obstacle_1_data.output_emergency);

        // -----------------------------

        accessory_bitwise_op(accessory, 0, 0); // B
        accessory_bitwise_op(accessory, 1, 0); // G
        accessory_bitwise_op(accessory, 2, 1); // R

        break;
    }
    }
}

void process_accessory()
{
    static uint8_t prev_active_accessory = accessory;

    static ros::Time time_now;
    static ros::Time time_last;
    time_last = time_now;
    time_now = ros::Time::now();
    ros::Duration dt = time_now - time_last;

    //----------------------------------

    static unsigned int prev_status_algoritma_accessory = 0;
    if (status_algoritma_accessory != prev_status_algoritma_accessory)
    {
        _log.debug("Status Algoritma: %d | Left: %d | Right:%d | Headlight: %d | RGB: %d%d%d",
                   status_algoritma_accessory,
                   (accessory & 0b100000) >> 5,
                   (accessory & 0b010000) >> 4,
                   (accessory & 0b001000) >> 3,
                   (accessory & 0b000100) >> 2,
                   (accessory & 0b000010) >> 1,
                   (accessory & 0b000001) >> 0);
    }
    prev_status_algoritma_accessory = status_algoritma_accessory;

    //----------------------------------

    switch (status_algoritma_accessory)
    {
    case 0:
    {
        int action_index = is_inside_action_accessory(0);
        if (action_index != -1)
        {
            switch (actions_accessory[action_index].action_type)
            {
            case 0:
                status_algoritma_accessory = 0;
                break;
            case 1:
                status_algoritma_accessory = 1;
                prev_active_accessory = accessory;
                break;
            default:
                status_algoritma_accessory = 0;
                break;
            }

            accessory_bitwise_op(accessory, 4, actions_accessory[action_index].rightsignal);
            accessory_bitwise_op(accessory, 5, actions_accessory[action_index].leftsignal);
        }
        break;
    }

    case 1:
    {
        // -------------------------------------------------------------
        // This state is to make car set accessory to defined parameter
        // while inside area. The parameter will be restored to previous
        // parameter when car move outside defined navigation area.
        // -------------------------------------------------------------

        if (is_inside_action_accessory(1.0) == -1)
        {
            status_algoritma_accessory = 0;
            accessory = prev_active_accessory;
        }

        break;
    }
    }
}

void process_test()
{
}

//------------------------------------------------------------------------------
//==============================================================================

int is_inside_action_main(float radius_offset)
{
    int result = -1;

    for (int i = 0; i < actions_main.size(); i++)
    {
        float dx = actions_main[i].x - pose_pose2d.x;
        float dy = actions_main[i].y - pose_pose2d.y;
        float distance = sqrt(dx * dx + dy * dy);

        if (distance < actions_main[i].radius + radius_offset)
        {
            result = i;
            break;
        }
    }

    return result;
}

int is_inside_action_accessory(float radius_offset)
{
    int result = -1;

    for (int i = 0; i < actions_accessory.size(); i++)
    {
        float dx = actions_accessory[i].x - pose_pose2d.x;
        float dy = actions_accessory[i].y - pose_pose2d.y;
        float distance = sqrt(dx * dx + dy * dy);

        if (distance < actions_accessory[i].radius + radius_offset)
        {
            result = i;
            break;
        }
    }

    return result;
}

//------------------------------------------------------------------------------
//==============================================================================

/**
 * @brief Read button code.
 *
 * @return int Button code.
 */
int button_read()
{
    int button_code = 0;

    if (ui_output_data.button_code != 0)
    {
        button_code = ui_output_data.button_code;
        ui_output_data.button_code = 0;
    }

    return button_code;
}

/**
 * It publishes a message to the topic /ui_output_sound.
 *
 * @param id the id of the sound
 * @param action the action of the sound
 * @param filename the name of the sound file to be played.
 */
void play_sound(uint8_t id, uint8_t action, std::string filename)
{
    icar_ui::ui_output_sound msg_ui_output_sound;
    msg_ui_output_sound.id = id;
    msg_ui_output_sound.action = action;
    msg_ui_output_sound.filename = filename;
    pub_ui_output_sound.publish(msg_ui_output_sound);
}

/**
 * @brief Convert encoder value to delta encoder value.
 *
 * @param encoder_kiri Value of left encoder.
 * @param encoder_kanan Value of right encoder.
 * @param d_encoder_kiri Delta value of left encoder.
 * @param d_encoder_kanan Delta value of right encoder.
 */
void encoder_2_d_encoder(uint16_t encoder_kiri, uint16_t encoder_kanan, int16_t *d_encoder_kiri, int16_t *d_encoder_kanan)
{
    static uint16_t last_encoder_kiri;
    static uint16_t last_encoder_kanan;

    /* To prevent the first value of encoder to be used as the last value. */
    if (last_encoder_kiri == 0 || last_encoder_kanan == 0)
    {
        last_encoder_kiri = encoder_kiri;
        last_encoder_kanan = encoder_kanan;
        *d_encoder_kiri = 0;
        *d_encoder_kanan = 0;
        return;
    }

    /* To prevent the encoder value to be overflowed. */
    if (encoder_kiri < 1024 && last_encoder_kiri > 3072)
        *d_encoder_kiri = encoder_kiri + 4096 - last_encoder_kiri;
    else if (encoder_kiri > 3072 && last_encoder_kiri < 1024)
        *d_encoder_kiri = encoder_kiri - 4096 - last_encoder_kiri;
    else
        *d_encoder_kiri = encoder_kiri - last_encoder_kiri;

    if (encoder_kanan < 1024 && last_encoder_kanan > 3072)
        *d_encoder_kanan = encoder_kanan + 4096 - last_encoder_kanan;
    else if (encoder_kanan > 3072 && last_encoder_kanan < 1024)
        *d_encoder_kanan = encoder_kanan - 4096 - last_encoder_kanan;
    else
        *d_encoder_kanan = encoder_kanan - last_encoder_kanan;

    last_encoder_kiri = encoder_kiri;
    last_encoder_kanan = encoder_kanan;
}

/**
 * @brief Convert gyroscope value to delta gyroscope value.
 *
 * @param gyroscope Value of gyroscope.
 * @param d_gyroscope Delta value of gyroscope.
 */
void gyroscope_2_d_gyroscope(float gyroscope, float *d_gyroscope)
{
    static float last_gyroscope;

    /* To prevent the first value of gyroscope to be used as the last value. */
    if (last_gyroscope == 0)
    {
        last_gyroscope = gyroscope;
        *d_gyroscope = 0;
        return;
    }

    /* To prevent the gyroscope value to be overflowed. */
    if (gyroscope < -90 && last_gyroscope > 90)
        *d_gyroscope = gyroscope + 360 - last_gyroscope;
    else if (gyroscope > 90 && last_gyroscope < -90)
        *d_gyroscope = gyroscope - 360 - last_gyroscope;
    else
        *d_gyroscope = gyroscope - last_gyroscope;

    last_gyroscope = gyroscope;
}

/**
 * The function performs a bitwise operation on a given bit of a uint8_t variable based on a provided
 * bit value.
 *
 * @param accessory an 8-bit unsigned integer representing a set of accessory bits.
 * @param bit_index The index of the bit in the accessory variable that needs to be modified.
 * @param bit_value The value of the bit to be set or cleared. It can be either 0 or 1. If it is -1,
 * the function returns without doing anything.
 *
 * @return If the `bit_value` parameter is equal to -1, the function will return without performing any
 * operations. Therefore, nothing is being returned explicitly.
 */
void accessory_bitwise_op(uint8_t &accessory, uint8_t bit_index, int8_t bit_value)
{
    if (bit_value != 0 && bit_value != 1)
        return;

    if (bit_value == 0)
        accessory &= ~(1 << bit_index);
    else if (bit_value == 1)
        accessory |= (1 << bit_index);
}