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
*/
//==============================================================================
// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <afConversions.h>
#include <boost/program_options.hpp>
#include <ambf_server/RosComBase.h>


namespace boost
{
    namespace program_options
    {
        class variables_map;
    }
}

namespace p_opt = boost::program_options;

using namespace std;
using namespace ambf;

class afPegGripperPlugin : public afSimulatorPlugin
{
public:
    afPegGripperPlugin();
    virtual int init(int argc, char **argv, const afWorldPtr a_afWorld) override;
    virtual void keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

    void peg_visibility_callback(const std_msgs::Bool::ConstPtr &msg);

protected:
    // private:
    // Pointer to the world
    afWorldPtr m_worldPtr;

    // Pointer for left/right gripper
    afRigidBodyPtr m_leftGripperPtr, m_rightGripperPtr;
    afJointPtr m_leftGripperJointPtr, m_rightGripperJointPtr;

    // Flag for gripper close
    bool m_gripperClosed = false;

    // Pointer for peg
    afRigidBodyPtr m_peg1Ptr, m_peg2Ptr, m_peg3Ptr;
    afRigidBodyPtr m_activePeg = nullptr;

    vector<afRigidBodyPtr> m_pegs;
    vector<afRigidBodyPtr> m_GoalPtrList;

    // Transform for gripper to peg while holding
    cTransform m_gripper2peg;

    // Path
    string m_current_filepath;

    // ROS
    ros::NodeHandle *ros_node_handle;
    ros::Subscriber peg_visibility_subscriber;
    bool m_flag_visible = false;


};

AF_REGISTER_SIMULATOR_PLUGIN(afPegGripperPlugin)
