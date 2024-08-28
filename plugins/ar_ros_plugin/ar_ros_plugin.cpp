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

string g_current_filepath;

afCameraHMD::afCameraHMD()
{
}

void afCameraHMD::get_publish_img_resolution(const afBaseObjectAttribsPtr a_objectAttribs)
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

int afCameraHMD::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    ros_node_handle = afROSNode::getNode();

    // Ambf camera
    // left_sub = ros_node_handle->subscribe("/ambf/env/cameras/stereoL/ImageData", 2, &afCameraHMD::left_img_callback, this);
    // right_sub = ros_node_handle->subscribe("/ambf/env/cameras/stereoR/ImageData", 2, &afCameraHMD::right_img_callback, this);
    // Zed mini
    left_sub = ros_node_handle->subscribe("/zedm/zed_node/left/image_rect_color", 2, &afCameraHMD::left_img_callback, this);
    // right_sub = ros_node_handle->subscribe("/zedm/zed_node/right/image_rect_color", 2, &afCameraHMD::right_img_callback, this);

    m_camera = (afCameraPtr)a_afObjectPtr;

    // No need to override rendering. Camera textures will be added to the back layer of the current scene.
    m_camera->setOverrideRendering(false);
    // m_width = m_camera->m_width;
    // m_height = m_camera->m_height;
    get_publish_img_resolution(a_objectAttribs);
    glfwSetWindowSize(m_camera->m_window, m_width, m_height);

    // m_camera->getInternalCamera()->m_stereoOffsetW = 0.1;

    // m_frameBuffer = cFrameBuffer::create();
    // m_frameBuffer->setup(m_camera->getInternalCamera(), m_width * m_alias_scaling, m_height * m_alias_scaling, true, true, GL_RGBA);

    string file_path = __FILE__;
    cout << "FILE PATH: " << file_path << endl;
    g_current_filepath = file_path.substr(0, file_path.rfind("/"));
    cout << g_current_filepath << endl;

    afShaderAttributes shaderAttribs;
    shaderAttribs.m_shaderDefined = true;
    shaderAttribs.m_vtxFilepath = g_current_filepath + "/shaders/hmd_distortion.vs";
    shaderAttribs.m_fragFilepath = g_current_filepath + "/shaders/hmd_distortion.fs";

    m_shaderPgm = afShaderUtils::createFromAttribs(&shaderAttribs, "TEST", "VR_CAM");
    if (!m_shaderPgm)
    {
        cerr << "ERROR! FAILED TO LOAD SHADER PGM \n";
        return -1;
    }

    // Load sample texture
    // todo;Note: Bigger initial texture -- things will not work well with this bigger texture.
    // string texture_path = g_current_filepath + "/../textures/raw_img.jpg";

    // Texture witht the same resolution as the zed mini
    string texture_path = g_current_filepath + "/../textures/test_sample.jpg";
    cImagePtr sample_img = cImage::create();
    bool success = sample_img->loadFromFile(texture_path);
    if (!success)
    {
        cerr << "ERROR! FAILED TO LOAD TEXTURE from " << texture_path;
        return -1;
    }

    sample_texture = cTexture2d::create();
    sample_texture->setImage(sample_img);

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

    // Variables related to ROS topics
    // concat_img_ptr = boost::make_shared<cv_bridge::CvImage>();

    // Textures
    // m_rosImageTexture = cTexture2d::create();

    m_screen_filling_quad->computeAllNormals();

    // Objects have multiple textures available in AMBF.
    // To pass multiple textures to the shader, we can use these multiple default textures.
    // For this case the rosImageTexture gets assigned to m_texture and
    // the texture from the m_imageBuffer to metallicTexture.

    m_screen_filling_quad->m_texture = sample_texture;
    // m_quadMesh->m_metallicTexture = m_frameBuffer->m_imageBuffer;

    // if (m_camera->m_frameBuffer->m_imageBuffer == nullptr)
    // {
    //     throw runtime_error("Frame buffer of m_camera should be initilized in the multiview_panels plugin");
    // }
    // m_quadMesh->m_metallicTexture = m_camera->m_frameBuffer->m_imageBuffer;

    m_screen_filling_quad->setUseTexture(true);
    m_screen_filling_quad->setShaderProgram(m_shaderPgm);
    m_screen_filling_quad->setShowEnabled(true);

    m_back_layer_world = new cWorld();
    m_back_layer_world->addChild(m_screen_filling_quad);
    m_camera->getInternalCamera()->m_backLayer = m_back_layer_world;

    cerr << "INFO! LOADING AR PLUGIN \n";
    cout << "JUAN!!!!\n\n\n\n";

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

    // updateHMDParams(); // Update HMD parameters before m_frameBuffer render creates problems.

    // glfwMakeContextCurrent(m_camera->m_window); //---> ERROR messages here.
    // m_frameBuffer->renderView();
    updateHMDParams();

    afRenderOptions ro;
    ro.m_updateLabels = true;

    // set_ros_texture(); // todo-NOTE: this does not work. Texture needs to be processed and update inside the ros callback

    // IF NEW TEXTURE WAS RECEIVED ADD IT TO THE QUAD TEXTURE.

    // cWorld *cachedWorld = m_camera->getInternalCamera()->getParentWorld();

    // m_camera->getInternalCamera()->setStereoMode(C_STEREO_DISABLED);
    // m_camera->getInternalCamera()->setParentWorld(m_vrWorld);

    // static cWorld *ew = new cWorld();
    // cWorld *fl = m_camera->getInternalCamera()->m_backLayer;
    // m_camera->getInternalCamera()->m_backLayer = fl;

    // cout << "Before rendering" << endl;
    // m_camera->render(ro);
    // cout << "After rendering" << endl;

    // m_camera->getInternalCamera()->m_backLayer = fl;

    // m_camera->getInternalCamera()->setStereoMode(C_STEREO_PASSIVE_LEFT_RIGHT);
    // m_camera->getInternalCamera()->setParentWorld(cachedWorld);
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
    //    cerr << "INFO! Shader ID " << id << endl;
    glUseProgram(id);

    // m_quadMesh->m_texture which points m_rosImageTexture gets automatically assigned to texture unit 0.
    // glUniform1i(glGetUniformLocation(id, "rosImageTexture"), 0);

    // m_quadMesh->m_metallic which points to a frameBuffer gets assigned to texture unit 2.
    // glUniform1i(glGetUniformLocation(id, "frameBufferTexture"), 2);

    // glUniform2fv(glGetUniformLocation(id, "ViewportScale"), 1, m_viewport_scale);
    // glUniform3fv(glGetUniformLocation(id, "aberr"), 1, m_aberr_scale);
    // glUniform1f(glGetUniformLocation(id, "WarpScale"), m_warp_scale * m_warp_adj);
    // glUniform4fv(glGetUniformLocation(id, "HmdWarpParam"), 1, m_distortion_coeffs);
    // glUniform2fv(glGetUniformLocation(id, "LensCenterLeft"), 1, m_left_lens_center);
    // glUniform2fv(glGetUniformLocation(id, "LensCenterRight"), 1, m_right_lens_center);
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

void afCameraHMD::left_img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        left_img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert");
    }

    set_ros_texture();
    // cv::resize(cv_ptr->image,cv_ptr->image,cv::Size(cv_ptr->image.cols/2,cv_ptr->image.rows/2));

    // cv::Rect sizeRect(0, 0, left_img_ptr->image.cols - left_img_ptr->image.cols * clipsize, left_img_ptr->image.rows);
    // left_img_ptr->image = left_img_ptr->image(sizeRect);

    // cv::imshow("Left img", left_img_ptr->image);
    // cv::waitKey(1);
}

void afCameraHMD::set_ros_texture()
{
    if (left_img_ptr != nullptr)
    {
        cv::cvtColor(left_img_ptr->image, left_img_ptr->image, cv::COLOR_RGBA2BGRA);
        cv::flip(left_img_ptr->image, left_img_ptr->image, 0);

        int ros_image_size = left_img_ptr->image.cols * left_img_ptr->image.rows * left_img_ptr->image.elemSize();
        int texture_image_size = sample_texture->m_image->getWidth() * sample_texture->m_image->getHeight() * sample_texture->m_image->getBytesPerPixel();

        if (ros_image_size != texture_image_size)
        {
            cout << "INITILIZE rosImageTexture" << endl;
            // For ZED 2i and AMBF rostopics
            sample_texture->m_image->erase();
            sample_texture->m_image->allocate(left_img_ptr->image.cols, left_img_ptr->image.rows, GL_RGBA, GL_UNSIGNED_BYTE);

            sample_texture->m_image->setData(left_img_ptr->image.data, ros_image_size);
            sample_texture->saveToFile("rosImageTexture_juan.png");
        }
        else
        {
            sample_texture->m_image->setData(left_img_ptr->image.data, ros_image_size);
        }
        sample_texture->markForUpdate();

        //  cerr << "INFO! Image Sizes" << msg->width << "x" << msg->height << " - " << msg->encoding << endl;
        // m_rosImageTexture->markForUpdate();
    }
}

// void afCameraHMD::right_img_callback(const sensor_msgs::ImageConstPtr &msg)
// {

//     try
//     {
//         right_img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
//         // frame2 = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_COLOR);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         ROS_ERROR("Could not convert");
//     }

//     cv::Rect sizeRect2(clipsize, 0, right_img_ptr->image.cols - right_img_ptr->image.cols * clipsize, right_img_ptr->image.rows);
//     right_img_ptr->image = right_img_ptr->image(sizeRect2);

//     // cv::imshow("Right img", right_img_ptr->image);
//     // cv::waitKey(1);

//     if (left_img_ptr != nullptr && right_img_ptr != nullptr)
//     {
//         cv::hconcat(left_img_ptr->image, right_img_ptr->image, concat_img_ptr->image);
//         cv::flip(concat_img_ptr->image, concat_img_ptr->image, 0);
//         cv::cvtColor(concat_img_ptr->image, concat_img_ptr->image, cv::COLOR_RGBA2BGRA);

//         int ros_image_size = concat_img_ptr->image.cols * concat_img_ptr->image.rows * concat_img_ptr->image.elemSize();
//         int texture_image_size = m_rosImageTexture->m_image->getWidth() * m_rosImageTexture->m_image->getHeight() * m_rosImageTexture->m_image->getBytesPerPixel();

//         if (ros_image_size != texture_image_size)
//         {
//             cout << "INITILIZE rosImageTexture" << endl;
//             m_rosImageTexture->m_image->erase();
//             // m_rosImageTexture->m_image->allocate(cv_ptr->image.cols, cv_ptr->image.rows, getImageFormat(cv_ptr->encoding), getImageType(cv_ptr->encoding));

//             // For ZED 2i and AMBF rostopics
//             m_rosImageTexture->m_image->allocate(concat_img_ptr->image.cols, concat_img_ptr->image.rows, GL_RGBA, GL_UNSIGNED_BYTE);
//             m_rosImageTexture->m_image->setData(concat_img_ptr->image.data, ros_image_size);

//             m_rosImageTexture->saveToFile("rosImageTexture_juan.png");
//         }
//         else
//         {
//             m_rosImageTexture->m_image->setData(concat_img_ptr->image.data, ros_image_size);
//         }

//         //  cerr << "INFO! Image Sizes" << msg->width << "x" << msg->height << " - " << msg->encoding << endl;
//         m_rosImageTexture->markForUpdate();

//         // cv::imshow("Concat image", concat_img_ptr->image);
//         // cv::waitKey(1);
//         // cv::resize(cv_ptr2->image,cv_ptr2->image,cv::Size(cv_ptr2->image.cols/2,cv_ptr2->image.rows/2));
//     }
// }
