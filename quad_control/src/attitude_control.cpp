#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "Quadcopter_UAV/Torque.h" 

// Declaración de variables
float phi, theta, psi, phi_punto, theta_punto, psi_punto;
float phi_d, theta_d, psi_d, phi_punto_d, theta_punto_d, psi_punto_d;
float error_phi, error_theta, error_psi;
float error_phi_punto, error_theta_punto, error_psi_punto;
float u_phi, u_theta, u_psi;

const float Jxx = 0.0411;
const float Jyy = 0.0478;
const float Jzz = 0.0599;

const float Kp_phi = 1.0;
const float Kd_phi = 3.0;
const float Kp_theta = 1.0;
const float Kd_theta = 3.0;
const float Kp_psi = 1.0;
const float Kd_psi = 3.0;

// Callbacks para los datos recibidos
void attitudeCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    phi = msg->x;
    theta = msg->y;
    psi = msg->z;
}

void attitudeDotCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    phi_punto = msg->x;
    theta_punto = msg->y;
    psi_punto = msg->z;
}

void angDesCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    phi_d = msg->x;
    theta_d = msg->y;
    psi_d = msg->z;
}

void calculateAndPublishTorques() {
    geometry_msgs::Vector3 torques;
    
    // Cálculos para los errores
    error_phi = phi_d - phi;
    error_theta = theta_d - theta;
    error_psi = psi_d - psi;

    error_phi_punto = phi_punto_d - phi_punto;
    error_theta_punto = theta_punto_d - theta_punto;
    error_psi_punto = psi_punto_d - psi_punto;

    u_phi = Kp_phi * error_phi + Kd_phi * error_phi_punto;
    u_theta = Kp_theta * error_theta + Kd_theta * error_theta_punto;
    u_psi = Kp_psi * error_psi + Kd_psi * error_psi_punto;

    // Asignación de los torques calculados al mensaje Vector3
    torques.x = Jxx * u_phi;
    torques.y = Jyy * u_theta;
    torques.z = Jzz * u_psi;

    // Publicación de los torques
    torque_pub.publish(torques);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "attitude_control");
    ros::NodeHandle nh;

    // Suscripciones
    ros::Subscriber subAttitude = nh.subscribe("dynamics/attitude", 10, &attitudeCallback);
    ros::Subscriber subAttitudeDot = nh.subscribe("dynamics/attitude_dot", 10, &attitudeDotCallback);
    ros::Subscriber subAngDes = nh.subscribe("posdata/angdes", 10, &angDesCallback);

    // Publicador para los torques usando Vector3
    ros::Publisher torque_pub = nh.advertise<geometry_msgs::Vector3>("/torques", 10);

    ros::Rate loop_rate(10); // Frecuencia de 10 Hz

    while (ros::ok()) {
        calculateAndPublishTorques(); 
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}