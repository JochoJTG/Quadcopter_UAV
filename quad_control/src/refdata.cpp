#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>


float hz=100;
float t;

float x=-5;
float y=0;
float z=0;

float phi=0;
float theta=0;
float psi=0;

float phi_punto_des;
float theta_punto_des;
float psi_punt_des;

float x_punt_des;
float y_punt_des;
float z_punt_des;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "refdata");
    ros:: NodeHandle nh;

    ros:: Publisher pos_pub = nh.advertise<geometry_msgs::Vector3>("refdata/pos",10);
    ros:: Publisher ang_pub = nh.advertise<geometry_msgs::Vector3>("refdata/ang",10);
    ros:: Publisher vel_pub = nh.advertise<geometry_msgs::Vector3>("refdata/vel",10);
    ros:: Publisher omega_pub = nh.advertise<geometry_msgs::Vector3>("refdata/omega",10);
    
    ros::Rate loop_rate(hz);

    
    geometry_msgs::Vector3 ref_pos;
    geometry_msgs::Vector3 ref_ang;
    geometry_msgs::Vector3 ref_vel;
    geometry_msgs::Vector3 ref_omega;

    while (ros::ok())
    {
        t=0.01+t;

        if(t<5){
            x_punt_des=0;
            y_punt_des=0;
            z_punt_des=-0.5;
            psi_punt_des=0;

        }

        if(t>=5 && t<=65){
            x_punt_des=0.5*sin(0.1*(t-5));

            y_punt_des=0.5*cos(0.1*(t-5));
            z_punt_des=0;
            psi_punt_des=0.1;
        }

        if (t>65)
        {
            t=0;
        }
        

    x= x + x_punt_des * 0.01;
    y= y+ y_punt_des * 0.01;
    z= z + z_punt_des *0.01;

    psi= psi + psi_punt_des *0.01;
    
    ref_pos.x = x;
    ref_pos.y = y;
    ref_pos.z = z;

    ref_ang.x = phi;
    ref_ang.y = theta;
    ref_ang.z = psi;

    ref_vel.x = x_punt_des;
    ref_vel.y = y_punt_des;
    ref_vel.z = z_punt_des;

    ref_omega.x = phi_punto_des;
    ref_omega.y = theta_punto_des;
    ref_omega.z = z_punt_des;

    ros::spinOnce();

    loop_rate.sleep();
    
    pos_pub.publish(ref_pos);
    ang_pub.publish(ref_ang);
    vel_pub.publish(ref_vel);
    omega_pub.publish(ref_omega);
  
    }
}
    