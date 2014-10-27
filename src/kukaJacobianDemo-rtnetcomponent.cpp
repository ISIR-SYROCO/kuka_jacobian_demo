// Filename: kukaSimpleDemo-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author: Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr)
// Description:

#include "kukaJacobianDemo-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>


#include <boost/foreach.hpp>
#include <math.h>

KukaJacobianDemoRTNET::KukaJacobianDemoRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
	this->addOperation("setJointImpedance", &KukaJacobianDemoRTNET::setJointImpedance, this, RTT::OwnThread);
	this->addOperation("setGains", &KukaJacobianDemoRTNET::setGains, this, RTT::OwnThread);
	this->addOperation("setXcons", &KukaJacobianDemoRTNET::setXcons, this, RTT::OwnThread);
	this->addOperation("setVmax", &KukaJacobianDemoRTNET::setVmax, this, RTT::OwnThread);
	Xcons(0)=0.0;
	Xcons(1)=0.0;
	Xcons(2)=0.0;
	Kp=20.0;
	Kd=1;
	
	Vmax=0.1; //mètres*dT seconds, si dT=1ms alors on a 5cm/s
}


bool KukaJacobianDemoRTNET::doStart(){
	//setting stiffness
	std::vector<double> stiff(LWRDOF, 1000.0);
	std::vector<double> damp(LWRDOF, 0.7);
	setJointImpedance(stiff, damp);
    	friStart();
	dT=this->getPeriod();
	std::cout<<"dT= "<<dT<<std::endl;
    	return true;
}

bool KukaJacobianDemoRTNET::configureHook(){
    setPeer("lwr");
    //initialize the arrays that will be send to KRL
    for(int i=0; i<16; ++i){
        fri_to_krl.intData[i]=0;
        fri_to_krl.realData[i]=0.0;
    }
	
    return true;
}


void KukaJacobianDemoRTNET::updateHook(){

  std::string fri_mode("e_fri_unkown_mode");
  bool fri_cmd_mode = false;
  RTT::FlowStatus fs_event = iport_events.read(fri_mode);
  if (fri_mode == "e_fri_cmd_mode")
    fri_cmd_mode = true;
  else if (fri_mode == "e_fri_mon_mode")
    fri_cmd_mode = false;
        
  std::vector<double> JState(LWRDOF);
  std::vector<double> JVel(LWRDOF);
  KDL::Jacobian Jac;
  RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
  RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(JVel);
  RTT::FlowStatus jacobian_fs = jacobianPort.read(Jac);
  RTT::FlowStatus cartPos_fs =  iport_cart_pos.read(Xmsr);

  if(fri_cmd_mode){
    //std::cout << "Command" << std::endl;
    if(joint_state_fs == RTT::NewData){
      Eigen::VectorXd joint_pos(LWRDOF);
      std::vector<double> joint_position_command(LWRDOF);
      for(unsigned int i = 0; i < LWRDOF; i++){
	joint_pos[i] = JState[i];
	joint_position_command[i] = JState[i];
      }


      if(joint_vel_fs == RTT::NewData){
	Eigen::VectorXd joint_vel(7);
	for(unsigned int i = 0; i < LWRDOF; i++){
	  joint_vel[i] = JVel[i];
	}

	if(jacobian_fs==RTT::NewData){
	  std::vector<double> joint_eff_command;
	  joint_eff_command.assign(LWRDOF, 0.0);

	  if(cartPos_fs==RTT::NewData){

        //Express the jacobian in the base frame instead of tool frame
        Jac.changeBase(KDL::Rotation::Quaternion(Xmsr.orientation.x, 
                                                 Xmsr.orientation.y,
                                                 Xmsr.orientation.z,
                                                 Xmsr.orientation.w));
	    Eigen::MatrixXd KukaJac(6,7);
	    KukaJac.noalias() = Jac.data;
	    //std::cout << "Jac" << std::endl;
	    //std::cout << KukaJac << std::endl;
	    //std::cout << "--" << std::endl;
	    KukaJac.transposeInPlace();
	    Eigen::VectorXd Xerr(3);
	    Xerr(0)=Xcons(0)-(double)Xmsr.position.x;
	    Xerr(1)=Xcons(1)-(double)Xmsr.position.y;
	    Xerr(2)=Xcons(2)-(double)Xmsr.position.z;

	    //discrétisation de la trajectoire
	    if(Xerr.norm()>=Vmax){
	    	for(int i=0;i<3;i++){
	    		Err(i)=(Xerr(i)/Xerr.norm())*Vmax;
	    	}
	    }else{
	        Err=Xerr;
	    }
	    //std::cout<< "Xerr = "<< Xerr(0)<<" Yerr = "<< Xerr(1)<<" Zerr = "<<Xerr(2)<<std::endl;
	    //std::cout<< " Kp = "<<Kp<< " Kd = " << Kd << std::endl;
	    //std::cout<<" Vmax = " << Vmax << std::endl;
	    for(int i=0;i<LWRDOF;i++){
	      double results=0;
	      for(int j=0;j<3;j++){
		//std::cout << "Jac " << i << " " << j << " " << KukaJac(i, j) << std::endl;
		//std::cout << "Kp " << Kp << std::endl;
		//std::cout << Err.transpose() << std::endl;
		results+=KukaJac(i,j)*Kp*Err(j);
	      }
      	      joint_eff_command[i] = results;//-(double)Kd*(double)(joint_vel[i]);
	    }
	    //std::cout << "Err" << std::endl;
	    //for(int i=0; i<3; ++i)
	    //  std::cout << Err[i] << " ";
	    //std::cout << "Xerr" << std::endl;
	    //for(int i=0; i<3; ++i)
	    //  std::cout << Xerr[i] << " ";
	    //std::cout << std::endl;

	    if(requiresControlMode(30)){
	      oport_add_joint_trq.write(joint_eff_command);
	    }
	    oport_joint_position.write(joint_position_command);

	  }else{
	    std::cout<<"Cannot read cartesian position port"<<std::endl;
	  }
	}else{
	  std::cout<<"Cannot read Jacobian Port"<<std::endl;
	}
      }else{
	std::cout<<"Cannot read Joint velocity Port"<<std::endl;
      }
    }else{
      std::cout<<"Cannot read Joint position Port"<<std::endl;
    }
  }
}

void KukaJacobianDemoRTNET::setXcons(std::vector<double> &X){
	for (int i=0;i<3;i++){
		Xcons(i)=X[i];
	}
}

void KukaJacobianDemoRTNET::setGains(double KP, double KD){
	Kp=KP;
	Kd=KD;
}

void KukaJacobianDemoRTNET::setVmax(double Vm){
	Vmax=Vm;
}


void KukaJacobianDemoRTNET::setJointImpedance(std::vector<double> &stiffness, std::vector<double> &damping){
	if(stiffness.size() != LWRDOF || damping.size() != LWRDOF){
		std::cout << "Wrong vector size, should be " << LWRDOF << ", " << LWRDOF << std::endl;
		return;
	}else{
		lwr_fri::FriJointImpedance joint_impedance_command;
		for(unsigned int i = 0; i < LWRDOF; i++){
			joint_impedance_command.stiffness[i] = stiffness[i];
			joint_impedance_command.damping[i] = damping[i];
		}

		oport_joint_impedance.write(joint_impedance_command);
	}
}

/*
* Using this macro, only one component may live
* in one library *and* you may *not* link this library
* with another component library. Use
* ORO_CREATE_COMPONENT_TYPE()
* ORO_LIST_COMPONENT_TYPE(kuka_jacobian_demo)
* In case you want to link with another library that
* already contains components.
*
* If you have put your component class
* in a namespace, don't forget to add it here too:
*/
ORO_CREATE_COMPONENT(KukaJacobianDemoRTNET)
