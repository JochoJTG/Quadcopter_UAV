#include <ros/ros.h>
#include <eigen3/Eigen/Dense>



const g = 9.81;
int uav_weight = 2 //KG
float thrust, force,tx, ty, tz;

float phi,theta,psi;

Eigen::Matrix3f intertias_matrix;
intertias_matrix << 0.0411, 0, 0
                    0, .0478, 0
                    0, 0, .0599;
Eigen::Vector3f torques;
torques << tx, ty, tz;

Eigen::Vector3f angular_accel_body;
Eigen::Vector3f angular_vel_body;
Eigen::Vector3f attitude_dot;
Eigen::Vector3f attitude;

Eigen::Vector3f linear_accel_body; //body frame
Eigen::Vector3f linear_vel_body; //body frame
Eigen::Vector3f e3;
    e3 << 0, 0, 1;
Eigen::Vector3f linear_pos;
Eigen::Vector3f linear_vel_inertial;
Eigen::Matrix3f rotationalmatrix;
Eigen::Matrix3f rot2;

void LinearControlCallback(){

    thrust = algo->x
}

void AttitudeControlCallback(){

    tx = algo->x;
    ty = algo->y;
    tz = algo->z;
}

void AngularDynamcis(){

        angular_accel_body = intertias_matrix.inverse() * (torques - angular_vel_body.cross(inertias_matrix * angular_vel_body));
    

    for (uint8_t i = 0; i < 3; i++)
    {
        angular_vel_body(i) = angular_vel_body(i) + step + angular_accel_body;
    }


    attitude_dot = R2(phi, theta, psi) * angular_vel_body

    for (uint8_t i = 0; i < 3; i++)
    {
        attitude(i) = attitude(i) + step + attitude_dot(i);
    }

}

void LinearDynamics(){

    force = thrust * e3 + R.transpose() * uav_weight * g * e3;
    
    linear_accel_body = force/uav_weight - angular_vel_body.cross(linear_vel_body);
    

    for (uint8_t i = 0; i < 3; i++)
    {
        linear_vel_body(i) = linear_vel_body(i) + step * linear_accel_body(i);
    }
   
    linear_vel_inertial = R(phi,theta,psi) * linear_vel_body;
    
    for (uint8_t i = 0; i < 3; i++)
    {
        linear_pos(i) = linear_pos(i) + step * linear_vel_inertial(i);
    }

}

void R(float phi, float theta, float psi){
    rotationalmatrix << cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta),
                        cos(theta)*sin(psi), cos(psi)*cos(phi) + sin(psi)*sin(phi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi),
                        -sin(theta)        , cos(theta)*sin(phi)                             , cos(phi)*cos(theta);
}
void R2(float phi, float theta, float psi){
    rot2 << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
            0, cos(phi)           , -sin(phi),
            0, sin(phi)/cos(theta), cos(phi)/cos(theta);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "Dynamics");
    ros::nodeHabdle nh;



    ros::Publisher DynamicsPub = nh.advertise<>("/Dynamics",10);
    ros::Subscriber LinearControl_Sub = nh.subscriber("/LinearControl", 10, &LinearControlCallback);
    ros::Subscriber AttitudeControl_Sub = nh.subscriber("/AttitudeControl", 10. &AttitudeControlCallback);


    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        
        AngularDynamcis();
        LinearDynamics();


        ros::spinOnce();
        loop_rate.sleep()

    }
    



}