#include <ardrone.h>


using namespace std;

Ardrone::Ardrone()
{
    updateOdometry();
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, true);
    drone_navdata_sub = nh.subscribe("/ardrone/navdata", 1000, &Ardrone::navdataCallback, this);
    drone_imu_sub = nh.subscribe("/ardrone/imu", 1000, &Ardrone::imudataCallback, this);

    // Set up reading PID controller parameters
    dynamic_reconfigure::Server<ardrone_project::DynamicConfig> srv;
    dynamic_reconfigure::Server<ardrone_project::DynamicConfig>::CallbackType f;
    f = boost::bind(&Ardrone::DynConfCallback, this, _1, _2);
    srv.setCallback(f);

}


Ardrone::~Ardrone()
{

}


//===============================
// basic ardrone publishers
void Ardrone::reset()
{
    ROS_INFO_STREAM("Ardrone is resetting...");

    reset_pub = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1000, true);
    while(reset_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    std_msgs::Empty empty;
    reset_pub.publish(empty);
}
//====
void Ardrone::takeoff()
{
    ROS_INFO_STREAM("Ardrone is taking off...");

    takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1000, true);
    while(takeoff_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    std_msgs::Empty empty;
    takeoff_pub.publish(empty);
}
//====
void Ardrone::land()
{
    ROS_INFO_STREAM("Ardrone is landing...");

    land_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 1000, true);
    while(land_pub.getNumSubscribers() < 1) {
        sleep(0.01);
    }
    std_msgs::Empty empty;
    land_pub.publish(empty);
}
//====
void Ardrone::hover()
{

    ROS_INFO_STREAM("Ardrone is hovering...");

    hover_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000, true);
    geometry_msgs::Twist velocity;

    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;

    hover_pub.publish(velocity);
}

//===============================
// receiving data from Ardrone

void Ardrone::updateOdometry()
{
    dronepose_sub = nh.subscribe("/ground_truth/state", 1000, &Ardrone::odometryCallback, this);
    //dronepose_sub = nh.subscribe("/ardrone/odometry", 1000, &Ardrone::odometryCallback, this);
}

//====
void Ardrone::odometryCallback(const nav_msgs::Odometry &msg)
{
    //ROS_INFO_STREAM("q1 = " << x << " q2 = " << y <<" q3 = "<< z << " q4 = " << w << "\n" << "x = " << currentX <<" y = " << currentY << " z =  " << currentZ);

    // drone position [m]
    currentX = msg.pose.pose.position.x;
    currentY = msg.pose.pose.position.y;
    currentZ = msg.pose.pose.position.z;
    //ROS_INFO_STREAM("x = " << currentX <<" y = " << currentY << " z =  " << currentZ);

}
//====
ArdroneOdometry::ArdronePosition Ardrone::getArdronePos()
{
    updateOdometry();

    ArdroneOdometry::ArdronePosition position;
    position.posX = currentX;
    position.posY = currentY;
    position.posZ = currentZ;

    return position;
}
//====
ArdroneOdometry::ArdroneEulerAngle Ardrone::getEulerAng()
{

    ArdroneOdometry::ArdroneEulerAngle angle;

    angle.rotX = rotXaxis;
    angle.rotY = rotYaxis;
    angle.rotZ = rotZaxis;


    return angle;
}
//====
void Ardrone::navdataCallback(const ardrone_autonomy::Navdata &navdata)
{
    // get linear velocity and convert mm/s to m/s
    drone.vel_x = navdata.vx / 1000.0;
    drone.vel_y = navdata.vy / 1000.0;
    drone.vel_z = navdata.vz / 1000.0;

    rotXaxis = navdata.rotX;
    rotYaxis = navdata.rotY;
    rotZaxis = navdata.rotZ;

    // get ardrone state
    ardrone_state = navdata.state;

}

//====
void Ardrone::imudataCallback(const sensor_msgs::Imu &imu)
{
    // angular drone velocity velocity [rad/sec]
    drone.vel_roll = imu.angular_velocity.x;
    drone.vel_pitch = imu.angular_velocity.y;
    drone.vel_yaw = imu.angular_velocity.z;

}

//===============================
// init PID errors
void Ardrone::updatePIDControl(double targetX, double targetY, double targetZ)
{

    Ardrone ardrone;

    const int rate = 20;
    ros::Rate loop_rate(rate);


    // Main ROS loop
    while (ros::ok()) {

        // get pose of drone
        updateOdometry();

        // pose error
        double pose_error[4];

        // derivative_error
        double derivative_error[4];

        double old_pose_error[4];

        double integ_pose_error[4];

        double current_x = ardrone.getArdronePos().posX;
        double current_y = ardrone.getArdronePos().posY;
        double current_z = ardrone.getArdronePos().posZ;
        double current_yaw = ardrone.getEulerAng().rotZ * RAD2GRAD;//[rad]
        current_yaw = stabilizeAngle(current_yaw);

        // error x,y,z
        pose_error[0] = targetX - current_x;
        pose_error[1] = targetY - current_y;
        pose_error[2] = targetZ - current_z;

        // calculate target's yaw [rad]
        double targetYaw = atan2((targetY - current_y), (targetX - current_x));
        targetYaw = stabilizeAngle(targetYaw);
        targetYaw = stabilizeAngle(targetYaw);
        // error yaw [rad]
        pose_error[3] = targetYaw - current_yaw;
        pose_error[3] = stabilizeAngle(pose_error[3]);
        pose_error[3] = stabilizeAngle(pose_error[3]);

        integ_pose_error[0] = (pose_error[0] + old_pose_error[0]) * 0.001 /2;
        integ_pose_error[1] = (pose_error[1] + old_pose_error[1]) * 0.001 /2;
        integ_pose_error[2] = (pose_error[2] + old_pose_error[2]) * 0.001 /2;

        integ_pose_error[0] = relu(integ_pose_error[0], 0.3);
        integ_pose_error[1] = relu(integ_pose_error[0], 0.3);
        integ_pose_error[2] = relu(integ_pose_error[0], 0.3);
        //integ_pose_error[3] = relu(integ_pose_error[0], 0.2);

        old_pose_error[0] = pose_error[0];
        old_pose_error[1] = pose_error[1];
        old_pose_error[2] = pose_error[2];
        old_pose_error[3] = pose_error[3];

        // differntial of pose and yaw
        derivative_error[0] = -(drone.vel_x);
        derivative_error[1] = -(drone.vel_y);
        derivative_error[2] = -(drone.vel_z);
        derivative_error[3] = -(drone.vel_yaw);

        double error_r = 0.2;

        double length = sqrt((pose_error[0]*pose_error[0]) + (pose_error[1]*pose_error[1]) + (pose_error[2]*pose_error[2]));

        if (length < error_r) return;

        double time = ros::Time::now().toSec() - old_t.toSec();


        xValue.optimizeFunction(pose_error[0], integ_pose_error[0], derivative_error[0], "x", time);
        yValue.optimizeFunction(pose_error[1], integ_pose_error[1], derivative_error[1], "y", time);
        zValue.optimizeFunction(pose_error[2], integ_pose_error[2], derivative_error[2], "z", time);
        yawValue.optimizeFunction(pose_error[3], 0.0, derivative_error[3], "yaw" , time);
        calcPIDControl(pose_error, derivative_error,integ_pose_error, current_yaw);

        ros::spinOnce();
        loop_rate.sleep();

        if(!(isnan(xValue.proportional_gain)) && (xValue.proportional_gain != 0.0)){
            p_coeff_x = xValue.proportional_gain;
            i_coeff_x = xValue.integral_gain;
            d_coeff_x = xValue.derivative_gain;
        }
        if(!(isnan(yValue.proportional_gain)) && (yValue.proportional_gain != 0.0)){
            p_coeff_y = yValue.proportional_gain;
            i_coeff_y = yValue.integral_gain;
            d_coeff_y = yValue.derivative_gain;
        }
        if(!(isnan(zValue.proportional_gain)) && (zValue.proportional_gain != 0.0)){
            p_coeff_z = zValue.proportional_gain;
            i_coeff_z = zValue.integral_gain;
            d_coeff_z = zValue.derivative_gain;
        }
        if(!(isnan(yawValue.proportional_gain)) && (yawValue.proportional_gain != 0.0)){
            p_coeff_yaw = yawValue.proportional_gain;
            d_coeff_yaw = yawValue.derivative_gain;
        }

        // show info
        ROS_INFO("--------------------");
        ROS_INFO("Ardrone");
        ROS_INFO("Position:        %0.4f    %0.4f    %0.4f",current_x, current_y, current_z );
        ROS_INFO("Target:          %0.4f    %0.4f    %0.4f" , targetX, targetY, targetZ );
        ROS_INFO("===");
        ROS_INFO("Error");
        ROS_INFO("   Pose:         %0.4f    %0.4f    %0.4f", pose_error[0], pose_error[1], pose_error[2]);
        ROS_INFO("   Yaw:          %0.4f",pose_error[3]);
        ROS_INFO("   Length:       %0.4f",length);
        ROS_INFO("===");
        ROS_INFO("SGD coeffs");
        ROS_INFO("   X:            %0.4f   %0.4f   %0.4f",xValue.proportional_gain, xValue.integral_gain, xValue.derivative_gain);
        ROS_INFO("   Y:            %0.4f   %0.4f   %0.4f",yValue.proportional_gain, yValue.integral_gain, yValue.derivative_gain);
        ROS_INFO("   Z:            %0.4f   %0.4f   %0.4f",zValue.proportional_gain, zValue.integral_gain, zValue.derivative_gain);
        //ROS_INFO("   Yaw:          %0.4f   %0.4f   %0.4f",yawValue.proportional_gain,0.000, yawValue.derivative_gain);
        ROS_INFO("===");
        //ROS_INFO("Default coeffs x:   %0.3f   %0.3f   %0.3f",p_coeff_x, i_coeff_x, d_coeff_x);
        //ROS_INFO("Default coeffs y:   %0.3f   %0.3f   %0.3f",p_coeff_y, i_coeff_y, d_coeff_y);
        //ROS_INFO("Default coeffs z:   %0.3f   %0.3f   %0.3f",p_coeff_z, i_coeff_z, d_coeff_z);
        //ROS_INFO("Default coeffs yaw: %0.3f   %0.3f   %0.3f",p_coeff_yaw,0.000, d_coeff_yaw);


        std::ofstream out ("/home/user/catkin_ws/src/ardrone_project/doc/out.txt", std::ios::app);
        if (out.is_open()){
        out << current_x << " " << current_y << " " << current_z << " " << current_yaw << " " << time << std::endl;
        }
        std::ofstream err ("/home/user/catkin_ws/src/ardrone_project/doc/errors.txt", std::ios::app);
        if (err.is_open()){
        err << pose_error[0] << " " << pose_error[1] << " " << pose_error[2] << " " << pose_error[3] << " " << time << std::endl;
        }
        /**
         std::ofstream sgd_y ("sgd_y.txt", std::ios::app);
        if (sgd_y.is_open()){
        sgd_y << yValue.coefficients[0] << " " << yValue.coefficients[1] << " " << yValue.coefficients[2] << " " << time <<std::endl;
        }
         std::ofstream sgd_z ("sgd_z.txt", std::ios::app);
        if (sgd_z.is_open()){
        sgd_z << zValue.coefficients[0] << " " << zValue.coefficients[1] << " " << zValue.coefficients[2] << " " << time <<std::endl;
        }
        */

        counter+=1;
    }
}



void Ardrone::DynConfCallback(ardrone_project::DynamicConfig &config, uint32_t level)
{

    p_coeff_x = config.Kp_x;
    p_coeff_y = config.Kp_y;
    p_coeff_z = config.Kp_z;
    p_coeff_yaw = config.Kp_yaw;

    i_coeff_x = config.Ki_x;
    i_coeff_y = config.Ki_y;
    i_coeff_z = config.Ki_z;

    d_coeff_x = config.Kd_x;
    d_coeff_y = config.Kd_x;
    d_coeff_z = config.Kd_x;
    d_coeff_yaw = config.Kd_yaw;


}

void Ardrone::calcPIDControl(const double * pose_error, const double * diff_pose_error, const double * integ_pose_error,  double drone_yaw)
{

    if (counter <= 2)
    {
        return;
    }
    Ardrone ardrone;

    //=============================

    // proportional part
    double proportional_part[4] = { pose_error[0], pose_error[1], pose_error[2], pose_error[3] };

    // convert global into local: rotation of axis
    // x' coordinate
    proportional_part[0] = cos(drone_yaw) * pose_error[0] + sin(drone_yaw) * pose_error[1];
    // y' coordinate
    proportional_part[1] = cos(drone_yaw) * pose_error[1] - sin(drone_yaw) * pose_error[0];

    //====

    // derivative part
    double derivative_part[4] = { diff_pose_error[0], diff_pose_error[1], diff_pose_error[2], diff_pose_error[3] };

    // convert global into local: rotation of axis
    // (x' coordinate)'
    derivative_part[0] = cos(drone_yaw) * diff_pose_error[1] - sin(drone_yaw) * diff_pose_error[0];
    // (y' coordinate)'
    derivative_part[1] = -cos(drone_yaw) * diff_pose_error[0] - sin(drone_yaw) * diff_pose_error[1];

    //=============================
    double integral_part[4] = { integ_pose_error[0], integ_pose_error[1], integ_pose_error[2], integ_pose_error[3] };

    integral_part[0] = sin(drone_yaw) * integ_pose_error[0] - cos(drone_yaw) * pose_error[1];
    integral_part[1] = sin(drone_yaw) * integ_pose_error[1] + cos(drone_yaw) * pose_error[0];


    // calculate control

    // x linear control

    double x_linear_control =  p_coeff_x * proportional_part[0]
                            +  i_coeff_x * integral_part[0]
                            +  d_coeff_x * derivative_part[0];

    x_linear_control = relu(x_linear_control, 1.0);

    control.setXVelocity(x_linear_control);

    // y linear control

    double y_linear_control =  p_coeff_y * proportional_part[1]
                            +  i_coeff_y * integral_part[1]
                            +  d_coeff_y * derivative_part[1];

    y_linear_control = relu(y_linear_control, 1.0);

    control.setYVelocity(y_linear_control);

    // z linear control

    double z_linear_control =  p_coeff_z * proportional_part[2]
                            +  i_coeff_z * integral_part[2]
                            +  d_coeff_z * derivative_part[2];

    z_linear_control = relu(z_linear_control, 1.0);

    control.setZVelocity(z_linear_control);

    // yaw_control
    double yaw_control  = p_coeff_yaw * proportional_part[3]
                        - d_coeff_yaw * derivative_part[3];

    yaw_control = relu(yaw_control, 1.0);

    //control.setYaw(yaw_control);

    // --- Publication of control commands

    vel_pub.publish(control.getCmdVel());


}

double Ardrone::stabilizeAngle(double &angle)
{
    while(angle >= M_PI)
        angle -= 2*M_PI;
    while(angle < -M_PI)
        angle += 2*M_PI;
    return angle;
}

double Ardrone::relu(double &inputValue, double bound)
{
    if (inputValue > bound)
        inputValue = bound;
    if (inputValue < -bound)
        inputValue = - bound;
    return inputValue;

}
int Ardrone::sign(double value) {

    return value/std::abs(value);

}
