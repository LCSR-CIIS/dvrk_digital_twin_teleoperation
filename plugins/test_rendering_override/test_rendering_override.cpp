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

    \author    <jbarrag3@jh.edu>
    \author    Juan Antonio Barragan

*/
//==============================================================================

#include "test_rendering_override.h"
#include <yaml-cpp/yaml.h>

using namespace std;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

string get_current_filepath()
{
    string g_current_filepath;
    string file_path = __FILE__;
    cout << "FILE PATH: " << file_path << endl;
    g_current_filepath = file_path.substr(0, file_path.rfind("/"));
    cout << g_current_filepath << endl;

    return g_current_filepath;
}

afTestCameraOverride::afTestCameraOverride()
{
    empty_world = new cWorld();
}

int afTestCameraOverride::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{

    g_current_filepath = get_current_filepath();

    // Camera config
    m_camera = (afCameraPtr)a_afObjectPtr;
    m_camera->setOverrideRendering(true); //This triggers the error.
    set_window_size_to_pub_resolution(a_objectAttribs);
    glfwSetWindowSize(m_camera->m_window, m_width, m_height);

    cerr << "INFO! LOADING TEST CAMERA OVERRIDE \n\n\n";

    return 1;
}

void afTestCameraOverride::graphicsUpdate()
{

    static int count =0;
    cout << "INFO! Graphics Update - test  " << count++ << endl;

    glfwMakeContextCurrent(m_camera->m_window); //This triggers the error.

    afRenderOptions ro;
    ro.m_updateLabels = true;

    m_camera->render(ro);
}

void afTestCameraOverride::physicsUpdate(double dt)
{
}

void afTestCameraOverride::reset()
{
}

bool afTestCameraOverride::close()
{
    return true;
}


void afTestCameraOverride::set_window_size_to_pub_resolution(const afBaseObjectAttribsPtr a_objectAttribs)
{
    YAML::Node specificationDataNode;
    // Print yaml data
    // cerr << "INFO! SPECIFICATION DATA " << a_objectAttribs->getSpecificationData().m_rawData << endl;

    specificationDataNode = YAML::Load(a_objectAttribs->getSpecificationData().m_rawData);

    YAML::Node publish_img_res_node = specificationDataNode["publish image resolution"];

    m_width = publish_img_res_node["width"].as<int>();
    m_height = publish_img_res_node["height"].as<int>();

    // cout << "Width: " << m_width << " Height: " << m_height << endl;
}
