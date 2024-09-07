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

#include "ar_ros_plugin.h"
#include <ambf_server/RosComBase.h>
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

afCameraHMD::afCameraHMD()
{
    empty_world = new cWorld();
}

// PLUGIN ENTRY POINT
int afCameraHMD::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{

    g_current_filepath = get_current_filepath();
    initilize_ros_subscribers(a_objectAttribs);

    // Camera config
    m_camera = (afCameraPtr)a_afObjectPtr;
    m_camera->setOverrideRendering(true);
    ar_world = m_camera->getInternalCamera()->getParentWorld();
    set_window_size_to_pub_resolution(a_objectAttribs);
    glfwSetWindowSize(m_camera->m_window, m_width, m_height);

    // Load shader program into m_shaderPgm
    load_bg_quad_shaders();
    if (!m_shaderPgm)
    {
        cerr << "ERROR! FAILED TO LOAD SHADER PGM \n";
        return -1;
    }

    // Set textured quad to back layer of the camera
    create_screen_filling_quad();
    m_back_layer_world = new cWorld();
    m_back_layer_world->addChild(m_screen_filling_quad);
    m_camera->getInternalCamera()->m_backLayer = m_back_layer_world;

    cerr << "INFO! LOADING AR PLUGIN \n\n\n";

    return 1;
}

void afCameraHMD::graphicsUpdate()
{
    static bool first_time = true;
    if (first_time)
    {
        // makeFullScreen();
        cout << "First rendering!" << endl;
        first_time = false;
    }

    glfwMakeContextCurrent(m_camera->m_window);
    process_and_set_ros_texture(); // todo: this does not work. Texture needs to be processed and update inside the ros callback
    updateHMDParams();

    afRenderOptions ro;
    ro.m_updateLabels = true;

    // Turn on-off the AR overlays
    if (activate_ar)
    {
        m_camera->getInternalCamera()->setParentWorld(ar_world);
    }
    else
    {
        m_camera->getInternalCamera()->setParentWorld(empty_world);
    }

    m_camera->render(ro);
    m_camera->getInternalCamera()->setParentWorld(ar_world);
}

void afCameraHMD::physicsUpdate(double dt)
{
}

void afCameraHMD::reset()
{
}

bool afCameraHMD::close()
{
    return true;
}

void afCameraHMD::updateHMDParams()
{
    GLint id = m_shaderPgm->getId();
    // cerr << "INFO! Shader ID " << id << endl;
    glUseProgram(id);
}

void afCameraHMD::makeFullScreen()
{
    const GLFWvidmode *mode = glfwGetVideoMode(m_camera->m_monitor);
    int w = 2880;
    int h = 1600;
    int x = mode->width - w;
    int y = mode->height - h;
    int xpos, ypos;
    glfwGetMonitorPos(m_camera->m_monitor, &xpos, &ypos);
    x += xpos;
    y += ypos;
    glfwSetWindowPos(m_camera->m_window, x, y);
    glfwSetWindowSize(m_camera->m_window, w, h);
    m_camera->m_width = w;
    m_camera->m_height = h;
    glfwSwapInterval(0);
    cerr << "\t Making " << m_camera->getName() << " fullscreen \n";
}

void afCameraHMD::ar_activate_callback(const std_msgs::Bool::ConstPtr &msg)
{
    activate_ar = msg->data;
}

void afCameraHMD::left_img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert");
    }

    // process_and_set_ros_texture();

    // Visualize the ros image
    // cv::imshow("Left img", img_ptr->image);
    // cv::waitKey(1);
}

// TODO: this doesn't seem to be doing anything useful if the callback process the image.
void img_ptr_deep_copy(cv_bridge::CvImagePtr &img_ptr, cv_bridge::CvImagePtr &img_ptr_copy)
{
    img_ptr_copy->image = img_ptr->image.clone();
    img_ptr_copy->encoding = img_ptr->encoding;
    img_ptr_copy->header = img_ptr->header;
}

void afCameraHMD::process_and_set_ros_texture()
{
    if (img_ptr != nullptr)
    {
        cv_bridge::CvImagePtr img_ptr_copy = boost::make_shared<cv_bridge::CvImage>();
        
        img_ptr_deep_copy(img_ptr, img_ptr_copy);

        // cv::cvtColor(img_ptr_copy->image, img_ptr_copy->image, cv::COLOR_RGBA2BGRA); // Only needed for zed mini
        cv::flip(img_ptr_copy->image, img_ptr_copy->image, 0);

        int ros_image_size = img_ptr_copy->image.cols * img_ptr_copy->image.rows * img_ptr_copy->image.elemSize();
        int texture_image_size = ros_texture->m_image->getWidth() * ros_texture->m_image->getHeight() * ros_texture->m_image->getBytesPerPixel();

        if (ros_image_size != texture_image_size)
        {
            cout << "INFO! Initilizing rosImageTexture" << endl;
            // For ZED 2i and AMBF rostopics -
            // TODO:Note img fmt should probably not be hard-coded.
            ros_texture->m_image->erase();
            ros_texture->m_image->allocate(img_ptr_copy->image.cols, img_ptr_copy->image.rows, GL_RGBA, GL_UNSIGNED_BYTE);
            ros_texture->m_image->setData(img_ptr_copy->image.data, ros_image_size);

            // Save for debugging
            // ros_texture->saveToFile("rosImageTexture_juan.png");
        }
        else
        {
            ros_texture->m_image->setData(img_ptr_copy->image.data, ros_image_size);
        }
        ros_texture->markForUpdate();
    }
}

void afCameraHMD::set_window_size_to_pub_resolution(const afBaseObjectAttribsPtr a_objectAttribs)
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

string afCameraHMD::read_rostopic_from_config(const afBaseObjectAttribsPtr a_objectAttribs)
{
    YAML::Node specificationDataNode;

    specificationDataNode = YAML::Load(a_objectAttribs->getSpecificationData().m_rawData);

    YAML::Node publish_img_res_node = specificationDataNode["ar-ros plugin config"];

    string rostopic = publish_img_res_node["rostopic"].as<string>();

    cout << "INFO! reading images from rostopic: " << rostopic << endl;

    return rostopic;
}

void afCameraHMD::initilize_ros_subscribers(const afBaseObjectAttribsPtr a_objectAttribs)
{

    ros_node_handle = afROSNode::getNode();

    string rostopic = read_rostopic_from_config(a_objectAttribs);

    img_subscriber = ros_node_handle->subscribe(rostopic, 2, &afCameraHMD::left_img_callback, this);

    ar_activate_subscriber = ros_node_handle->subscribe("/ar_activate", 2, &afCameraHMD::ar_activate_callback, this);

    // Ambf camera
    // img_subscriber = ros_node_handle->subscribe("/ambf/env/cameras/stereoL/ImageData", 2, &afCameraHMD::left_img_callback, this);
    // right_sub = ros_node_handle->subscribe("/ambf/env/cameras/stereoR/ImageData", 2, &afCameraHMD::right_img_callback, this);
    // Zed mini
    // img_subscriber = ros_node_handle->subscribe("/zedm/zed_node/left/image_rect_color", 2, &afCameraHMD::left_img_callback, this);
    // right_sub = ros_node_handle->subscribe("/zedm/zed_node/right/image_rect_color", 2, &afCameraHMD::right_img_callback, this);
}

void afCameraHMD::load_bg_quad_shaders()
{
    afShaderAttributes shaderAttribs;
    shaderAttribs.m_shaderDefined = true;
    shaderAttribs.m_vtxFilepath = g_current_filepath + "/shaders/hmd_distortion.vs";
    shaderAttribs.m_fragFilepath = g_current_filepath + "/shaders/hmd_distortion.fs";

    m_shaderPgm = afShaderUtils::createFromAttribs(&shaderAttribs, "TEST", "VR_CAM");
}

void afCameraHMD::create_screen_filling_quad()
{

    // Load an initial texture that can be displayed while the first ros image is received
    // todo:Note: Bigger initial texture -- things will not work well with this bigger texture.
    string texture_path = g_current_filepath + "/../textures/sample1300x1024.jpg";
    // Texture witht the same resolution as the zed mini
    // string texture_path = g_current_filepath + "/../textures/sample640x360.jpg";
    cImagePtr sample_img = cImage::create();
    bool success = sample_img->loadFromFile(texture_path);
    if (!success)
    {
        cerr << "ERROR! FAILED TO LOAD TEXTURE from " << texture_path << endl;
    }

    // Texture that will be updated with the ros image
    ros_texture = cTexture2d::create();
    ros_texture->setImage(sample_img);

    m_screen_filling_quad = new cMesh();
    // clang-format off
    float quad[] = {
        // positions
        -1.0f,  1.0f, 0.0f,
        -1.0f, -1.0f, 0.0f, 
         1.0f, -1.0f, 0.0f, 
        -1.0f,  1.0f, 0.0f,
         1.0f, -1.0f, 0.0f,
         1.0f,  1.0f, 0.0  };
    // clang-format on

    for (int vI = 0; vI < 2; vI++)
    {
        int off = vI * 9;
        cVector3d v0(quad[off + 0], quad[off + 1], quad[off + 2]);
        cVector3d v1(quad[off + 3], quad[off + 4], quad[off + 5]);
        cVector3d v2(quad[off + 6], quad[off + 7], quad[off + 8]);
        m_screen_filling_quad->newTriangle(v0, v1, v2);
    }
    m_screen_filling_quad->m_vertices->setTexCoord(1, 0.0, 0.0, 1.0);
    m_screen_filling_quad->m_vertices->setTexCoord(2, 1.0, 0.0, 1.0);
    m_screen_filling_quad->m_vertices->setTexCoord(0, 0.0, 1.0, 1.0);
    m_screen_filling_quad->m_vertices->setTexCoord(3, 0.0, 1.0, 1.0);
    m_screen_filling_quad->m_vertices->setTexCoord(4, 1.0, 0.0, 1.0);
    m_screen_filling_quad->m_vertices->setTexCoord(5, 1.0, 1.0, 1.0);

    m_screen_filling_quad->computeAllNormals();

    // Objects have multiple textures available in AMBF.
    // To pass multiple textures to the shader, we can use these multiple default textures.
    m_screen_filling_quad->m_texture = ros_texture;
    // m_quadMesh->m_metallicTexture = m_frameBuffer->m_imageBuffer;

    m_screen_filling_quad->setUseTexture(true);
    m_screen_filling_quad->setShaderProgram(m_shaderPgm);
    m_screen_filling_quad->setShowEnabled(true);
}