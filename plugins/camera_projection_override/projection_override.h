// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace ambf;

class afCameraIntrinsics;

class afCameraProjectionOverride: public afObjectPlugin{
public:
    afCameraProjectionOverride();
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

    bool computeProjectionFromIntrinsics(const afCameraIntrinsics *a_attribs, double a_nearPlane, double a_farPlane);
    bool setProjectionFromYaml(YAML::Node projectionMatrixNode);
 
protected:
    afCameraPtr m_camera;
    cTransform m_customProjectionMatrix;

};

struct afCameraIntrinsics
{
    afCameraIntrinsics()
    {
        m_defined = false;
        m_fx = 100.;
        m_fy = 100.;
        m_cx = 0.;
        m_cy = 0.;
        m_s = 0.;
    }

    bool m_defined;
    double m_fx; // Focal length X
    double m_fy; // Focal Length Y
    double m_cx; // Principal Offset X
    double m_cy; // Principal Offset Y
    double m_s;  // Shear

    int width;
    int height;
};

AF_REGISTER_OBJECT_PLUGIN(afCameraProjectionOverride)