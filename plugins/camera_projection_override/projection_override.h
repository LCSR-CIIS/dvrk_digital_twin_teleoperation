// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>

using namespace std;
using namespace ambf;


class afCameraProjectionOverride: public afObjectPlugin{
public:
    afCameraProjectionOverride();
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;
protected:
    afCameraPtr m_camera;
    cTransform m_customProjectionMatrix;

};


AF_REGISTER_OBJECT_PLUGIN(afCameraProjectionOverride)