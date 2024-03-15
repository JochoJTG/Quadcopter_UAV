#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <iostream>

float phi theta psi phi_punto theta_punto psi_punto;
float phi_d theta_d psi_d phi_punto_d theta_punto_d psi_punto_d;
float error_phi error_theta_punto error_psi_punto;
float u_phi u_theta u_psi;

const float Jxx = 0.0411;
const float Jyy = 0.0478;
const float Jzz = 0.0599;

const float Kp_phi = 1.0;
const float Kd_phi = 3.0;
const float Kp_theta = 1.0;
const float Kd_theta = 3.0;
const float Kp_psi = 1.0;
const float Kd_psi = 3.0;

void dronInfoCallback(const Quadcopter_UAV::dronInfo::ConstPtr& msg){
    phi = msg->phi;
    theta = msg->theta;
    psi = msg->psi;
    phi_punto = msg->phi_punto;
    theta_punto = msg->theta_punto;
    psi_punto= msg->psi_punto;
}

void desiredInfoCallback(const Quadcopter_UAV::dronInfo::ConstPtr& msg){
    phi_d = msg->psi_d;
    theta_d = msg->theta_d;
    psi_d = msg->psi_d;
    phi_punto_d = msg->phi_punto_d;
    theta_punto_d = msg->theta_punto_d;
    psi_punto_d = msg->psi_punto_d;
}

Quadcopter_UAV::Torque calculateTorques() {
    Quadcopter_UAV::Torque torques;
    // Lógica para calcular los torques
    error_phi = phi_d - phi;
    error_phi_punto = phi_d - phi;

    error_theta = theta_d - theta;
    error_theta_punto = theta_d - theta;

    error_psi = psi_d - psi;
    error_psi_punto = psi_d - psi;

    u_phi = Kp_phi*error_phi + Kd_phi*error_phi_punto;
    u_theta = Kp_phi*error_theta + Kd_theta*error_theta_punto;
    u_psi = Kp_psi*error_psi + Kd_psi*error_psi_punto;

    torques.tau_phi = Jxx*(((Jzz-Jyy)/Jxx)*theta_punto*psi_punto + u_phi);
    torques.tau_theta = Jyy*(((Jxx-Jzz)/Jyy)*theta_punto*psi_punto + u_theta);
    torques.tau_psi = Jzz*(((Jyy-Jxx)/Jzz)*theta_punto*psi_punto + u_psi);

    return torques;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "attitude");
    ros::NodeHandle n;

    // Subscripciones
    ros::Subscriber subDesired = n.subscribe("/infoAngular_deseada", 10, desiredInfoCallback);
    ros::Subscriber subDron = n.subscribe("/infoAngular_obtenida", 10, dronInfoCallback);

    // Publicador para los torques
    ros::Publisher torque_pub = n.advertise<Quadcopter_UAV::Torque>("/torques", 10);

    ros::Rate loop_rate(10); // Frecuencia de 10 Hz

    while (ros::ok()) {
        Quadcopter_UAV::Torque torques = calculateTorques();
        torque_pub.publish(torques);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
