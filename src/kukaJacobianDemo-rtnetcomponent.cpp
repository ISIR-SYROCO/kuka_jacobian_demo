// Filename: kukaSimpleDemo-rtnetcomponent.cpp
// Copyright: 2014 ISIR-CNRS
// Author: Sovan Hak, Guillaume Hamon (hak@isir.upmc.fr, hamon@isir.upmc.fr)
// Description:

#include "kukaJacobianDemo-rtnetcomponent.hpp"
#include <rtt/Component.hpp>
#include <iostream>

KukaJacobianDemoRTNET::KukaJacobianDemoRTNET(std::string const& name) : FriRTNetExampleAbstract(name){
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
    RTT::FlowStatus joint_state_fs = iport_msr_joint_pos.read(JState);
    RTT::FlowStatus joint_vel_fs = iport_msr_joint_vel.read(JVel);

    if(joint_state_fs == RTT::NewData){
        Eigen::VectorXd joint_pos(LWRDOF);
        
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_pos[i] = JState[i];
            joint_position_command[i] = JState[i];
        }
        
    }
    
    if(joint_vel_fs == RTT::NewData){
        Eigen::VectorXd joint_vel(7);
        for(unsigned int i = 0; i < LWRDOF; i++){
            joint_vel[i] = JVel[i];
        }
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
