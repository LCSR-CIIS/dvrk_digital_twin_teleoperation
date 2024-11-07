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
    p_opt::options_description cmd_opts("drilling_simulator Command Line Options");
    cmd_opts.add_options()("info", "Show Info")
                          ("nt", p_opt::value<int>()->default_value(8), "Number Tool Cursors to Load. Default 8")
                          ("ds", p_opt::value<float>()->default_value(0.026), "Offset between shaft tool cursors. Default 0.026")
                          ("vm", p_opt::value<string>()->default_value("00ShinyWhite.jpg"), "Volume's Matcap Filename (Should be placed in the ./resources/matcap/ folder)")
                          ("dm", p_opt::value<string>()->default_value("dark_metal_brushed.jpg"), "Drill's Matcap Filename (Should be placed in ./resources/matcap/ folder)")
                          ("fp", p_opt::value<string>()->default_value("/dev/input/js0"), "Footpedal joystick input file description E.g. /dev/input/js0)")
                          ("mute", p_opt::value<bool>()->default_value(false), "Mute")
                          ("gcdr", p_opt::value<double>()->default_value(30.0), "Gaze Calibration Marker Motion Duration");

    p_opt::variables_map var_map;
    p_opt::store(p_opt::command_line_parser(argc, argv).options(cmd_opts).allow_unregistered().run(), var_map);
    p_opt::notify(var_map);

    if (var_map.count("info"))
    {
        std::cout << cmd_opts << std::endl;
        return -1;
    }

    string file_path = __FILE__;
    string current_filepath = file_path.substr(0, file_path.rfind("/"));


    // string volume_matcap = var_map["vm"].as<string>();
    // string footpedal_fd = var_map["fp"].as<string>();

    // m_worldPtr = a_afWorld;

    // // Get first camera
    // m_mainCamera = findAndAppendCamera("main_camera");
    // m_cameraL = findAndAppendCamera("cameraL");
    // m_cameraR = findAndAppendCamera("cameraR");
    // m_stereoCamera = findAndAppendCamera("stereoLR");

    // if (m_cameras.size() == 0)
    // {
    //     cerr << "ERROR! NO CAMERAS FOUND. " << endl;
    //     return -1;
    // }

    // if (!m_mainCamera)
    // {
    //     cerr << "INFO! FAILED TO LOAD main_camera, taking the first camera from world " << endl;
    //     m_mainCamera = m_worldPtr->getCameras()[0];
    // }

    // // if (m_stereoCamera){
    // //     makeVRWindowFullscreen(m_stereoCamera);
    // // }

    // m_panelManager.addCamera(m_mainCamera);
    // if (m_stereoCamera)
    // {
    //     m_stereoCamera->getInternalCamera()->m_stereoOffsetW = 0.1;
    //     m_panelManager.addCamera(m_stereoCamera);
    // }

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
    }
    else if (a_mods == GLFW_MOD_ALT)
    {
    }
    else
    {
        // Increase disparity of cameras
        if (a_key == GLFW_KEY_LEFT_BRACKET) // [
        {
            cout << "Increase distance between cameras" << endl;
        }
        // Decrease disparity of small window
        else if (a_key == GLFW_KEY_RIGHT_BRACKET) // ]
        {

            cout << "Decrease distance between cameras" << endl;
        }
        // option - reduce size along X axis
        else if (a_key == GLFW_KEY_A)
        {
            cout << "Toggle AR" << endl;
        }
        else if (a_key == GLFW_KEY_C)
        {
            cout << "Toggle comm loss" << endl;
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
