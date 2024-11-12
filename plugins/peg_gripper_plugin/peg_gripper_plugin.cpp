//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2024, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <hishida3@jhu.edu>
    \author    Hisashi Ishida
    \date      09.18.2024
    
*/
//==============================================================================

#include "peg_gripper_plugin.h"

using namespace std;

afPegGripperPlugin::afPegGripperPlugin(){
    cout << "/*********************************************" << endl;
    cout << "/* AMBF PegGripper Plugin" << endl;
    cout << "/*********************************************" << endl;
}

int afPegGripperPlugin::init(int argc, char** argv, const afWorldPtr a_afWorld){
    p_opt::options_description cmd_opts("AMBF_TF_Plugin Command Line Options");
    cmd_opts.add_options()
            ("info", "Show Info");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if(var_map.count("info")){
        std::cout<< cmd_opts << std::endl;
        return -1;
    }

    // Define path
    string file_path = __FILE__;
    m_current_filepath = file_path.substr(0, file_path.rfind("/"));

    // Get pointer to World
    m_worldPtr = a_afWorld;

    // Improve the constratint
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp = 1.0;  // improve out of plane error of joints
    m_worldPtr->m_bulletWorld->getSolverInfo().m_erp2 = 1.0; // improve out of plane error of joints

    // Get the pointer to gripper (left and right)
    m_leftGripperPtr = m_worldPtr->getRigidBody("BODY tool gripper left link");
    m_rightGripperPtr = m_worldPtr->getRigidBody("BODY tool gripper right link");

    // Get pointer for the joint
    m_leftGripperJointPtr = m_worldPtr->getJoint("JOINT tool yaw link-tool gripper left link");
    m_rightGripperJointPtr = m_worldPtr->getJoint("JOINT tool yaw link-tool gripper right link");

    // Get the pointer to the peg
    m_peg1Ptr = m_worldPtr->getRigidBody("BODY Peg_1");
    m_peg2Ptr = m_worldPtr->getRigidBody("BODY Peg_2");
    m_peg3Ptr = m_worldPtr->getRigidBody("BODY Peg_3");

    m_pegs.push_back(m_peg1Ptr);
    m_pegs.push_back(m_peg2Ptr);
    m_pegs.push_back(m_peg3Ptr);

    // Set fixed transformation from the gripper to Peg Manually
    m_gripper2peg.setLocalPos(cVector3d(0.006383, 0.023474, -0.003079));
    cQuaternion qrot(-0.3535,-0.612262,0.3535, 0.612262); //(w,x,y,z)
    cMatrix3d rot; //(w,x,y,z)
    qrot.toRotMat(rot);
    m_gripper2peg.setLocalRot(rot);

    // Store Goal locations
    afRigidBodyPtr goalPollR1Ptr = m_worldPtr->getRigidBody("BODY Poll_R_1");
    afRigidBodyPtr goalPollR2Ptr = m_worldPtr->getRigidBody("BODY Poll_R_2");
    afRigidBodyPtr goalPollR3Ptr = m_worldPtr->getRigidBody("BODY Poll_R_3");  

    m_GoalPtrList.push_back(goalPollR1Ptr);
    m_GoalPtrList.push_back(goalPollR2Ptr);
    m_GoalPtrList.push_back(goalPollR3Ptr);

    return 1;
}

void afPegGripperPlugin::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){ 
}

void afPegGripperPlugin::graphicsUpdate(){

    if (!m_gripperClosed){
        for (afRigidBodyPtr goalPtr: m_GoalPtrList){
            if ((goalPtr->getLocalPos() - m_leftGripperJointPtr->getLocalPos()).length() < 0.002){
                m_activePeg->setVisibleFlag(false);
            }
        }
    }

    else{
            m_activePeg->setVisibleFlag(true);
    }
}

void afPegGripperPlugin::physicsUpdate(double dt){
    // Check if the gripper is been closed or not
    if (m_leftGripperJointPtr->getPosition() < 0.2 &&  m_rightGripperJointPtr->getPosition() < 0.2){
        m_gripperClosed = true;

        // Get closest peg from the tool
        double minDistance = 1000;
        for (afRigidBodyPtr pegPtr:m_pegs){
            if (minDistance > (pegPtr->getLocalPos() - m_leftGripperJointPtr->getLocalPos()).length()){
                minDistance = (pegPtr->getLocalPos() - m_leftGripperJointPtr->getLocalPos()).length();
                m_activePeg = pegPtr;
            }
        }

        // Check if there is close enough peg
        // if (m_activePeg && minDistance < 1.0){
        if (m_activePeg){

            // Apply the relative transformation from the gripper
            btTransform command;
            btTransform parentTransform;
            m_leftGripperPtr->m_bulletRigidBody->getMotionState()->getWorldTransform(parentTransform);
            command = parentTransform * to_btTransform(m_gripper2peg);

            // Apply transformation to
            m_activePeg->m_bulletRigidBody->getMotionState()->setWorldTransform(command);
            m_activePeg->m_bulletRigidBody->setWorldTransform(command);
        }
    }
    else{
        m_gripperClosed = false;
    }
}

void afPegGripperPlugin::reset(){
    cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool afPegGripperPlugin::close(){
    return -1;
}
