//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2022, AMBF
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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar

    \author    <pkunjam1@jhu.edu>
    \author    Punit Kunjam
*/
//==============================================================================

#include "keyboard_shortcuts.h"
#include <boost/program_options.hpp>
#include <ambf_server/RosComBase.h>

using namespace std;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace p_opt = boost::program_options;

KeyboardPlugin::KeyboardPlugin()
{
}

int KeyboardPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld)
{

    // Get world and cameras 
    m_worldPtr = a_afWorld;
    m_cameraL = findAndAppendCamera("cameraL");
    m_cameraR = findAndAppendCamera("cameraR");

    if (m_cameras.size() == 0)
    {
        cerr << "ERROR! NO CAMERAS FOUND. " << endl;
        return -1;
    }
    
    cVector3d p = m_cameraR->getLocalPos();
    cout << "cameraR pose: " << p.x() <<  " " << p.y() << " " << p.z() << endl;

    initialize_ros_publishers();


    return 1;
}

void KeyboardPlugin::graphicsUpdate()
{
}

void KeyboardPlugin::physicsUpdate(double dt)
{
}

afCameraPtr KeyboardPlugin::findAndAppendCamera(string cam_name)
{
    afCameraPtr cam = m_worldPtr->getCamera(cam_name);
    if (cam)
    {
        cerr << "INFO! GOT CAMERA: " << cam->getName() << endl;
        m_cameras[cam_name] = cam;
    }
    else
    {
        cerr << "WARNING! CAMERA NOT FOUND " << cam_name << endl;
    }
    return cam;
}

void KeyboardPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    if (a_mods == GLFW_MOD_CONTROL)
    {
        if (a_key == GLFW_KEY_A)
        {
            toggle_ar();
        }
        else if (a_key == GLFW_KEY_C)
        {
            toggle_comm_loss();
        }
    }
    else if (a_mods == GLFW_MOD_ALT)
    {
    }
    else
    {
        // Increase disparity of cameras
        if (a_key == GLFW_KEY_LEFT_BRACKET) // [
        {
            cout << "Move right cam to the left" << endl;
            increaseCameraDisparity(-0.0005); 
        }
        // Decrease disparity of small window
        else if (a_key == GLFW_KEY_RIGHT_BRACKET) // ]
        {
            cout << "Move right cm to the right" << endl;
            increaseCameraDisparity(0.0005);
        }
    }
}

void KeyboardPlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes)
{
}

void KeyboardPlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos)
{
}

void KeyboardPlugin::reset()
{
    std::cerr << "INFO! PLUGIN RESET CALLED" << endl;
}

bool KeyboardPlugin::close()
{
    return true;
}

void KeyboardPlugin::toggle_comm_loss()
{
    comm_loss_status = !comm_loss_status; 

    std_msgs::Bool msg;
    msg.data = comm_loss_status;
    m_comm_loss_pub.publish(msg);
    cout << "Communication Loss: " << comm_loss_status << endl;
}
void KeyboardPlugin::toggle_ar()
{
    ar_activate_status = !ar_activate_status;

    std_msgs::Bool msg;
    msg.data = ar_activate_status;
    m_ar_activate_pub.publish(msg);
    cout << "AR Activate: " << ar_activate_status << endl;
}

void KeyboardPlugin::initialize_ros_publishers()
{
    m_rosNode = afROSNode::getNode();
    m_comm_loss_pub = m_rosNode->advertise<std_msgs::Bool>("/communication_loss", 1);
    m_ar_activate_pub = m_rosNode->advertise<std_msgs::Bool>("/ar_activate", 1);

    cout << "INFO! Initialize comm_loss_status to: " << comm_loss_status << endl;
    cout << "INFO! Initialize ar_activate_status to: " << ar_activate_status << endl;

    std_msgs::Bool msg_comm;
    msg_comm.data = comm_loss_status;
    m_comm_loss_pub.publish(msg_comm);

    std_msgs::Bool msg_ar;
    msg_ar.data = ar_activate_status;
    m_ar_activate_pub.publish(msg_ar);

}

// Positive delta move the camera away from the left cam.
// Negative delta move the camera towards the left cam.
void KeyboardPlugin::increaseCameraDisparity(float delta)
{
    cVector3d p = m_cameraR->getLocalPos();
    p.x(p.x() + delta);
    m_cameraR->setLocalPos(p);

    cVector3d p2 = m_cameraR->getLocalPos();
    cout << "cameraR pose: " << p2.x() <<  " " << p2.y() << " " << p2.z() << endl;
}
