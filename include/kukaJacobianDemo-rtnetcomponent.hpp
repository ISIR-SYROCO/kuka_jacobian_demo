// Filename: kukaJacobianDemo-rtnetcomponent.hpp
// Copyright: 2014 ISIR-CNRS
// Author: Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr)
// Description: Orocos component to command the kuka using simple Jacobian transpose controller

#ifndef KUKA_JACOBIAN_DEMO_RTNET_COMPONENT_HPP
#define KUKA_JACOBIAN_DEMO_RTNET_COMPONENT_HPP

#include <friRTNetExampleAbstract.hpp>
#include <Eigen/Dense>

class KukaJacobianDemoRTNET : public FriRTNetExampleAbstract{
    public:
        KukaJacobianDemoRTNET(std::string const& name);

	Eigen::Vector3d Xcons, Xdes, Err;
	geometry_msgs::Pose Xmsr;
	double Kp, Kd, dT,Vmax;
	
	bool doStart();
	bool configureHook();
        void updateHook();

	void setXcons(std::vector<double> &X);
	void setGains(double KP, double KD);
	void setVmax(double Vm);
	void setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping);

};

#endif
