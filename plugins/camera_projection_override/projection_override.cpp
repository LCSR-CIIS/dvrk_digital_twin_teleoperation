#include "projection_override.h"
#include <yaml-cpp/yaml.h>

using namespace std;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------

string g_current_filepath;

afCameraProjectionOverride::afCameraProjectionOverride()
{
    m_customProjectionMatrix.m_flagTransform = true;
}

int afCameraProjectionOverride::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_camera = (afCameraPtr)a_afObjectPtr;
    m_customProjectionMatrix = m_camera->getInternalCamera()->m_projectionMatrix;
    YAML::Node specificationDataNode;
    cerr << "INFO! RUNNING PROJECTION OVERRIDE PLUGIN " << endl;
//    cerr << "INFO! SPECIFICATION DATA " << a_objectAttribs->getSpecificationData().m_rawData << endl;
    specificationDataNode = YAML::Load(a_objectAttribs->getSpecificationData().m_rawData);

    YAML::Node projectionMatrixNode = specificationDataNode["projection matrix"];

    if (projectionMatrixNode.IsDefined()){
        try{
            vector<vector<double>> mat = projectionMatrixNode.as<vector<vector<double>>>();
            const int rows = 4, cols = 4;
            for (int r = 0 ; r < rows ; r++){
                cerr << "[";
                for (int c = 0 ; c < cols ; c++){
                    cerr << mat[r][c] << " ";
                    m_customProjectionMatrix(r, c) = mat[r][c];
                }
                cerr << "]" << endl;
            }

            m_camera->getInternalCamera()->m_useCustomProjectionMatrix = true;
            m_camera->getInternalCamera()->m_projectionMatrix = m_customProjectionMatrix;

            cerr << "INFO! SETTING CUSTOM PROJECT MATRIX FOR " << m_camera->getName() << endl;
        }
        catch(YAML::Exception& e){
            cerr << "ERROR! Exception " << e.what() << endl;
        }

    }
    cerr << m_camera->getInternalCamera()->m_projectionMatrix.str(4) << endl;

    return 0; // Return -1 as we don't want to do anything else in this plugin
}

void afCameraProjectionOverride::graphicsUpdate(){
    cerr << m_camera->getInternalCamera()->m_projectionMatrix.str(4) << endl;
}

void afCameraProjectionOverride::physicsUpdate(double dt){

}

void afCameraProjectionOverride::reset(){

}

bool afCameraProjectionOverride::close(){
    return true;
}