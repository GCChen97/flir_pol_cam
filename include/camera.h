#include <string>
#include <fstream>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;


class PCamConfig
{
public:
    PCamConfig();
    PCamConfig(string path_config);

    string pixelFormat;
    string acquisitionMode;
    float gamma;
    string exposureMode;
    float exposureTime;
    float frame_rate;
};

class PCamera
{

public:
    PCamera(CameraPtr pCam);
    PCamera(CameraPtr pCam, PCamConfig config);
    virtual void* AcquireImage();
    ~PCamera();
    void setEnumValue(string, string);
    void setIntValue(string, int);
    void setFloatValue(string, float);
    void setBoolValue(string, bool);

private:
    ImagePtr pImage;
    CameraPtr pCam;

};
