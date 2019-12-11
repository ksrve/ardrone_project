#ifndef ARDRONE_H
#define ARDRONE_H

#include <ros/ros.h>


#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include "ardrone_project/DynamicConfig.h"

#include <string>
#include <iostream>

#include <numeric>
#include <float.h>
#include <vector>
#include <math.h>
#include <cmath>
#include <fstream>

#include <sstream>

// Data drom Ardrone
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ardrone_autonomy/Navdata.h>

#define GRAD2RAD 180/M_PI;
#define RAD2GRAD M_PI/180;

using namespace std;


class ArdroneOdometry
{
public:
    //current ardrone position
    struct ArdronePosition
    {
        double posX;
        double posY;
        double posZ;
    };

    //current ardrone orientation
    struct ArdroneEulerAngle
    {
        double rotX;// roll
        double rotY;// pitch
        double rotZ;// yaw
    };
    // for derivative part

        double vel_x;
        double vel_y;
        double vel_z;

    //struct ArdroneImu
    //{
        double vel_roll;
        double vel_pitch;
        double vel_yaw;
    //};

};

class StochasticGradientDescent
{
public:

    // Errors
    double proportional_error = 0;
    double integral_error = 0;
    double derivative_error = 0;

    // PID parameters
    double proportional_gain = 0;
    double integral_gain = 0;
    double derivative_gain = 0;

    // SGD parameters
    const int amount_of_training_examples = 25;// for point = 25; for trajectory = 10
    int steps_counter = 0;

    std::vector<double> sgd_error;
    std::vector<std::vector<double>> sgd_error_terms;

    // Vector coefficients
    std::vector<double> coefficients{proportional_gain, integral_gain, derivative_gain};

    double error;


    void updateErrors(double proportional_error, double integral_error, double derivative_error)
    {

        // Push back proportional_error
        sgd_error.push_back(proportional_error);

        // Vector of errors
        std::vector<double> error_values{proportional_error, integral_error, derivative_error};

        // Push the vector into the vector of vectors
        sgd_error_terms.push_back(error_values);
    }

    void SGDOptimization(std::string type, double time)
    {

        double learning_rate = 0.001;// for point = 0.0003; for trajectory = 0.01
        int epochs = 50;// for point = 100; for trajectory = 50

        for(std::size_t epoch = 0; epoch < epochs; epoch++)
        {
            double sum_of_sq_error = 0.0;
            // From 2 because of Gazebo
            for(std::size_t i = 2; i < sgd_error.size(); i++)
            {
              double predicted_error =  coefficients[0] * sgd_error_terms[i][0] +
                                        coefficients[1] * sgd_error_terms[i][1] +
                                        coefficients[2] * sgd_error_terms[i][2];

              error = predicted_error - sgd_error[i];

              // Add up the sum of the squared errors
            sum_of_sq_error += error * error;

             // If the error has converged, break, to save computation time
                if(sum_of_sq_error < 0.00001)
                {
                    break;
                }

              for(std::size_t coeff_index = 0; coeff_index < coefficients.size(); coeff_index++)
              {
                coefficients[coeff_index] = coefficients[coeff_index] - learning_rate * error * sgd_error_terms[i][coeff_index];
            }
            std::string name_file = "/home/user/catkin_ws/src/ardrone_project/doc/sgd_" + type + ".txt";
                std::ofstream sgd (name_file, std::ios::app);
                if (sgd.is_open()){
                sgd << coefficients[0] << " " << coefficients[1] << " " << coefficients[2] << " " << time << std::endl;

            }
        }
    }

        proportional_gain = abs(coefficients[0]);
        integral_gain = abs(coefficients[1]);
        derivative_gain = abs(coefficients[2]);
    }

      // PID optimizer
      void optimizeFunction(double proportional_error, double integral_error, double derivative_error, std::string type, double time)
      {

        updateErrors(proportional_error, integral_error, derivative_error);

        if(steps_counter == amount_of_training_examples)
        {
          SGDOptimization(type, time);

          steps_counter = 0;
          sgd_error.clear();
          sgd_error_terms.clear();

        }
        // Increment the counter
        steps_counter += 1;
      }

};

class ControlArdrone
{

private:
    geometry_msgs::Twist cmd;
public:
    // get cmd_vel parametrs
    const geometry_msgs::Twist &getCmdVel()
    {
        return cmd;
    }
    // set cmd_vel parametrs
    void setXVelocity(const double &vel)
    {
        cmd.linear.x = vel;
    }
    void setYVelocity(const double &vel)
    {
        cmd.linear.y = vel;
    }
    void setZVelocity(const double &vel)
    {
        cmd.linear.z = vel;
    }
    void setYaw(double yaw)
    {
        cmd.angular.z = yaw;
    }
};



class Ardrone
{

public:


    Ardrone();

    ~Ardrone();

    //basic Ardrone publishers
    void reset();
    void takeoff();
    void hover();
    void land();

    //init PID
     void updatePIDControl(double targetX, double targetY, double targetZ);


    //get Ardrone position: x,y,z
    ArdroneOdometry::ArdronePosition getArdronePos();
    //get Ardrone Euler angles: roll, pitch, yaw
    ArdroneOdometry::ArdroneEulerAngle getEulerAng();

private:


	ros::NodeHandle nh;

    //ROS Subscribers
    ros::Subscriber dronepose_sub;
    ros::Subscriber drone_navdata_sub;
    ros::Subscriber drone_imu_sub;

    //ROS Publishers
	ros::Publisher vel_pub;
    ros::Publisher takeoff_pub;
    ros::Publisher reset_pub;
    ros::Publisher land_pub;
    ros::Publisher hover_pub;


    ControlArdrone control;

    ArdroneOdometry drone;

    StochasticGradientDescent xValue;
    StochasticGradientDescent yValue;
    StochasticGradientDescent zValue;
    StochasticGradientDescent yawValue;


    // get Ardrone Navdata
    void navdataCallback(const ardrone_autonomy::Navdata &navdata);

    // get Ardrone Imu
    void imudataCallback(const sensor_msgs::Imu &imu);

    //get Ardrone odometry
    void odometryCallback(const nav_msgs::Odometry &msg);

    //main part of PID
    void calcPIDControl(const double * new_error, const double * diff_error, const double * integ_pose_error, double drone_yaw);

    void DynConfCallback(ardrone_project::DynamicConfig &config, uint32_t level);

	// update Ardrone odometry
	void updateOdometry();

	// other functions
	double stabilizeAngle(double &angle);

    double angleFrom2(double angle, double min, double sup);


    int GetMS(ros::Time stamp = ros::Time::now());


    // saturation / windup
    double relu(double &inputValue, double bound);

    int sign(double value);


    // ardrone state
    int ardrone_state;


    double x, y, z, w;
	double currentX, currentY, currentZ;

    double rotXaxis;
    double rotYaxis;
    double rotZaxis;

    double p_coeff_x;
    double p_coeff_y;
    double p_coeff_z;
    double p_coeff_yaw;

    double i_coeff_x;
    double i_coeff_y;
    double i_coeff_z;

    double d_coeff_x;
    double d_coeff_y;
    double d_coeff_z;
    double d_coeff_yaw;

    double dt;
    ros::Time old_t = ros::Time::now();
    int counter = 0;

};

#endif /* ARDRONE_H */
