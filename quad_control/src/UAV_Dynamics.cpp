#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>



int g = 9.81;
int uav_weight = 2; //KG
float thrust, tx, ty, tz;

float phi,theta,psi;

float step = 1.0/100.0;

Eigen::Matrix3f inertias_matrix;
Eigen::Vector3f torques;

Eigen::Vector3f angular_accel_body;
Eigen::Vector3f angular_vel_body;
Eigen::Vector3f attitude_dot;
Eigen::Vector3f attitude;

Eigen::Vector3f linear_accel_body; //body frame
Eigen::Vector3f linear_vel_body; //body frame
Eigen::Vector3f e3;
Eigen::Vector3f linear_pos;
Eigen::Vector3f linear_vel_inertial;
Eigen::Matrix3f rotationalmatrix;
Eigen::Matrix3f rot2;
Eigen::Vector3f force;

Eigen::Matrix3f R(float phi, float theta, float psi);
Eigen::Matrix3f R2(float phi, float theta, float psi);

void LinearControlCallback(const geometry_msgs::Vector3::ConstPtr& th){

    thrust = th->x;
}

void AttitudeControlCallback(const geometry_msgs::Vector3::ConstPtr& moments){

    tx = moments->x;
    ty = moments->y;
    tz = moments->z;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "Dynamics");
    ros::NodeHandle nh;

    ros::Publisher Dynamics_pos_Pub = nh.advertise<geometry_msgs::Vector3>("dynamics/pos",10);
    ros::Publisher Dynamics_vel_Pub = nh.advertise<geometry_msgs::Vector3>("dynamics/lin_vel",10);
    ros::Publisher Dynamics_attitude_Pub = nh.advertise<geometry_msgs::Vector3>("dynamics/attitude",10);
    ros::Publisher Dynamics_attitude_dot_Pub = nh.advertise<geometry_msgs::Vector3>("dynamics/attitude_dot",10);

    ros::Subscriber LinearControl_Sub = nh.subscribe("/posdata/angdes", 10, &LinearControlCallback);
    ros::Subscriber AttitudeControl_Sub = nh.subscribe("/torques", 10, &AttitudeControlCallback);

    ros::Rate loop_rate(100);

    inertias_matrix << 0.0411, 0, 0,
                        0, .0478, 0,
                        0, 0, .0599;
    
    torques << tx, ty, tz;

    e3 << 0, 0, 1;  

    while (ros::ok())
    {
        
    
    angular_accel_body = inertias_matrix.inverse() * (torques - angular_vel_body.cross(inertias_matrix * angular_vel_body));
    

    for (int i = 0; i < 3; i++)
    {
        angular_vel_body(i) = angular_vel_body(i) + step * angular_accel_body(i);
    }


    attitude_dot = R2(attitude(0), attitude(1), attitude(2)) * angular_vel_body;

    for (int i = 0; i < 3; i++)
    {
        attitude(i) = attitude(i) + step * attitude_dot(i);
    }

    //Linear dynamics
    
    force = thrust * e3 + R(attitude(0), attitude(1), attitude(2)).transpose() * uav_weight * g * e3;

    linear_accel_body = force/uav_weight - angular_vel_body.cross(linear_vel_body);
    
    for (int i = 0; i < 3; i++)
    {
        linear_vel_body(i) = linear_vel_body(i) + step * linear_accel_body(i);
    }
   
    linear_vel_inertial = R(attitude(0), attitude(1), attitude(2)) * linear_vel_body;
    
    for (int i = 0; i < 3; i++)
    {
        linear_pos(i) = linear_pos(i) + step * linear_vel_inertial(i);
    }


        ros::spinOnce();
        loop_rate.sleep();

      
    }
}


Eigen::Matrix3f R(float phi, float theta, float psi){
    rotationalmatrix << cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
                        cos(theta)*sin(psi), cos(psi)*cos(phi) + sin(psi)*sin(phi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi),
                        -sin(theta)        , cos(theta)*sin(phi)                             , cos(phi)*cos(theta);

    return rotationalmatrix;
}

Eigen::Matrix3f R2(float phi, float theta, float psi){
    rot2 << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
            0, cos(phi)           , -sin(phi),
            0, sin(phi)/cos(theta), cos(phi)/cos(theta);

    return rot2;
}