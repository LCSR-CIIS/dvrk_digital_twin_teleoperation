#include "projection_override.h"
#include <string>
#include <ambf_server/RosComBase.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

string g_current_filepath;

bool afCameraProjectionOverride::computeProjectionFromIntrinsics(const afCameraIntrinsics *a_attribs,
                                                                 double a_nearPlane, double a_farPlane)
{
    double fx = a_attribs->m_fx;
    double fy = a_attribs->m_fy;
    double cx = a_attribs->m_cx;
    double cy = a_attribs->m_cy;
    double s = a_attribs->m_s;
    double W = a_attribs->width;
    double H = a_attribs->height;
    // double image_center_x = W / 2.;
    // double image_center_y = H / 2.;
    double n = a_nearPlane;
    double f = a_farPlane;

    m_camera->getInternalCamera()->m_useCustomProjectionMatrix = true;
    m_camera->getInternalCamera()->m_projectionMatrix(0, 0) = (2 * fx) / W;
    m_camera->getInternalCamera()->m_projectionMatrix(0, 1) = (2 * s) / W;
    m_camera->getInternalCamera()->m_projectionMatrix(0, 2) = (W - 2 * cx) / W;
    m_camera->getInternalCamera()->m_projectionMatrix(1, 1) = (2 * fy) / H;
    m_camera->getInternalCamera()->m_projectionMatrix(1, 2) = (-H + 2 * cy) / H;
    m_camera->getInternalCamera()->m_projectionMatrix(2, 2) = (-f - n) / (f - n);
    m_camera->getInternalCamera()->m_projectionMatrix(2, 3) = (-2 * f * n) / (f - n);
    m_camera->getInternalCamera()->m_projectionMatrix(3, 2) = -1.0;

    m_camera->getInternalCamera()->m_projectionMatrix(3, 3) = 0.0; // MISSING FROM ADNAN'S AMBF FORK. This value is one by default.

    return true;
}

bool afCameraProjectionOverride::setProjectionFromYaml(YAML::Node projectionMatrixNode)
{
    try
    {
        vector<vector<double>> mat = projectionMatrixNode.as<vector<vector<double>>>();
        const int rows = 4, cols = 4;
        for (int r = 0; r < rows; r++)
        {
            cerr << "[";
            for (int c = 0; c < cols; c++)
            {
                cerr << mat[r][c] << " ";
                m_customProjectionMatrix(r, c) = mat[r][c];
            }
            cerr << "]" << endl;
        }

        m_camera->getInternalCamera()->m_useCustomProjectionMatrix = true;
        m_camera->getInternalCamera()->m_projectionMatrix = m_customProjectionMatrix;

        // cerr << "INFO! SETTING CUSTOM PROJECT MATRIX FOR " << m_camera->getName() << endl;
    }
    catch (YAML::Exception &e)
    {
        cerr << "ERROR! Exception " << e.what() << endl;
        return false;
    }

    return true;
}

bool get_camera_instrinsic_from_rostopic(string camera_info_topic, afCameraIntrinsics &cam_instrinsics)
{

    ros::NodeHandle *ros_node_handle = afROSNode::getNode();

    boost::shared_ptr<const sensor_msgs::CameraInfo> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, *ros_node_handle, ros::Duration(2));

    if (msg)
    {
        cam_instrinsics.width = msg->width;
        cam_instrinsics.height = msg->height;
        cam_instrinsics.m_fx = msg->P[0];
        cam_instrinsics.m_s  = msg->P[1];
        cam_instrinsics.m_cx = msg->P[2];
        cam_instrinsics.m_fy = msg->P[5];
        cam_instrinsics.m_cy = msg->P[6];
        cam_instrinsics.m_defined = true;
        cerr << "INFO! Camera Intrinsics " << endl;
        cerr << "fx: " << cam_instrinsics.m_fx << endl;
        cerr << "fy: " << cam_instrinsics.m_fy << endl;
        cerr << "cx: " << cam_instrinsics.m_cx << endl;
        cerr << "cy: " << cam_instrinsics.m_cy << endl;
        cerr << "s: " << cam_instrinsics.m_s << endl;
    }
    else
    {
        cerr << "ERROR! Failed to get camera intrinsics from rostopic " << camera_info_topic << endl;
        return false;
    }
    return true;
}

afCameraProjectionOverride::afCameraProjectionOverride()
{
    m_customProjectionMatrix.m_flagTransform = true;
}

int afCameraProjectionOverride::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_camera = (afCameraPtr)a_afObjectPtr;

    cerr << "INFO! START OF PROJECTION OVERRIDE PLUGIN FOR " << m_camera->getName() << endl;

    m_customProjectionMatrix = m_camera->getInternalCamera()->m_projectionMatrix;
    YAML::Node specificationDataNode;
    //    cerr << "INFO! SPECIFICATION DATA " << a_objectAttribs->getSpecificationData().m_rawData << endl;
    specificationDataNode = YAML::Load(a_objectAttribs->getSpecificationData().m_rawData);

    YAML::Node plugin_config = specificationDataNode["camera-override plugin config"];
    YAML::Node rostopicCameraInfoNode = plugin_config["rostopic camera info"];
    YAML::Node projectionMatrixNode = plugin_config["projection matrix"];

    if (rostopicCameraInfoNode.IsDefined()) // field "rostopic camera info" needs to be defined
    {

        cerr << "INFO! SETTING PROJECTION MATRIX FROM CAMERAINFO " << endl;
        string camera_info_topic = rostopicCameraInfoNode.as<string>();
        afCameraIntrinsics cam_instrinsics;
        bool status = get_camera_instrinsic_from_rostopic(camera_info_topic, cam_instrinsics);

        if (status)
        {
            computeProjectionFromIntrinsics(&cam_instrinsics,
                                            m_camera->getInternalCamera()->getNearClippingPlane(),
                                            m_camera->getInternalCamera()->getFarClippingPlane());
        }
    }
    else if (projectionMatrixNode.IsDefined())
    {
        cerr << "INFO! SETTING PROJECTION MATRIX FROM YAML " << endl;
        setProjectionFromYaml(projectionMatrixNode);
    }
    else
    {
        cerr << "ERROR! Projection matrix for camera " << m_camera->getName() << " was not modified" << endl;
    }

    cerr << "INFO! PROJECTION MATRIX FOR CAMERA " << m_camera->getName() << " IS SET TO" << endl;
    cerr << m_camera->getInternalCamera()->m_projectionMatrix.str(4) << endl;

    return 0; // Return -1 as we don't want to do anything else in this plugin
}

void afCameraProjectionOverride::graphicsUpdate()
{
    cerr << m_camera->getInternalCamera()->m_projectionMatrix.str(4) << endl;
}

void afCameraProjectionOverride::physicsUpdate(double dt)
{
}

void afCameraProjectionOverride::reset()
{
}

bool afCameraProjectionOverride::close()
{
    return true;
}