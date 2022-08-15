
#include "camera.h"

PCamConfig::PCamConfig(string path_config)
{
    cout << path_config << endl;
    ifstream file(path_config);

    string line;

    file >> line; // skip line
    file >> pixelFormat;

    file >> line;
    file >> acquisitionMode;

    file >> line;
    file >> gamma;

    file >> line;
    file >> exposureMode;

    file >> line;
    file >> exposureTime;
    
    file >> line;
    file >> frame_rate;
}

PCamera::PCamera(CameraPtr pCam_)
{
    pCam = pCam_;

    if (pCam_->IsInitialized()) {
        cout << "Camera already initialized. Deinitializing..." << endl;
        pCam_->EndAcquisition();
        pCam_->DeInit();
    }

    pCam->Init();

    try
    {
        setEnumValue("PixelFormat", "Mono8");
    }
    catch(exception e)
    {
        setEnumValue("PixelFormat", "BayerRGPolarized8");
    }

    setEnumValue("AcquisitionMode", "Continuous");

    setBoolValue("AcquisitionFrameRateEnable", true);
    setFloatValue("AcquisitionFrameRate", 30);

    pCam->BeginAcquisition();
}

PCamera::PCamera(CameraPtr pCam_, PCamConfig config)
{
    pCam = pCam_;

    if (pCam_->IsInitialized()) {
        cout << "Camera already initialized. Deinitializing..." << endl;
        pCam_->EndAcquisition();
        pCam_->DeInit();
    }

    pCam->Init();
    
    setEnumValue("PixelFormat", config.pixelFormat);

    setEnumValue("AcquisitionMode", config.acquisitionMode);

    setFloatValue("Gamma", config.gamma);

    setEnumValue("ExposureMode", config.exposureMode);

    if (config.exposureTime>0)
    {
        setEnumValue("ExposureAuto", "Off");
        setFloatValue("ExposureTime", config.exposureTime); //unit: us, 1e-6s
    }
    
    if (config.frame_rate>0)
    {
        setBoolValue("AcquisitionFrameRateEnable", true);
        setFloatValue("AcquisitionFrameRate", config.frame_rate);
    }

    pCam->BeginAcquisition();
}

void* PCamera::AcquireImage()
{
    try
    {
        pImage = pCam->GetNextImage(2000);
        if (pImage->IsIncomplete())
        {
            cout << "Image incomplete with image status " << pImage->GetImageStatus() << "..." << endl
                << endl;
        }
        return pImage->GetData();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return 0;
    }
}

PCamera::~PCamera()
{
    // GetData()的指针会被释放，必须在这之前把图片publish
    // pointer from GetData() would be released，make sure publish the image before that
    pImage->Release();
    pCam->EndAcquisition();
    pCam->DeInit();
    // pCam = NULL;
}

void PCamera::setEnumValue(string setting, string value)
{
    INodeMap & nodeMap = pCam->GetNodeMap();
    
    // Retrieve enumeration node from nodemap
    CEnumerationPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr))
        cout << "Unable to set " << setting << " to " << value << " (enum retrieval). Aborting..." << endl;

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrValue = ptr->GetEntryByName(value.c_str());
    if (!IsAvailable(ptrValue) || !IsReadable(ptrValue))
        cout << "Unable to set " << setting << " to " << value << " (entry retrieval). Aborting..." << endl;
		
    // retrieve value from entry node
    int64_t valueToSet = ptrValue->GetValue();
		
    // Set value from entry node as new value of enumeration node
    ptr->SetIntValue(valueToSet);    

    cout << setting << " set to " << value << endl;
    
}

void PCamera::setIntValue(string setting, int val)
{
    INodeMap & nodeMap = pCam->GetNodeMap();
    
    CIntegerPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        cout << "Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting..." << endl;
    }
    ptr->SetValue(val);

    cout << setting << " set to " << val << endl;
    
}

void PCamera::setFloatValue(string setting, float val)
{
    INodeMap & nodeMap = pCam->GetNodeMap();
    
    CFloatPtr ptr = nodeMap.GetNode(setting.c_str());
    // cout << "Default " << setting << " is " << ptr->GetValue() << endl;
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        cout << "Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting..." << endl;
    }
    ptr->SetValue(val);

    cout << setting << " set to " << val << endl;
    
}

void PCamera::setBoolValue(string setting, bool val)
{
    INodeMap & nodeMap = pCam->GetNodeMap();
    
    CBooleanPtr ptr = nodeMap.GetNode(setting.c_str());
    if (!IsAvailable(ptr) || !IsWritable(ptr)) {
        cout << "Unable to set " << setting << " to " << val << " (ptr retrieval). Aborting..." << endl;
    }
    if (val) ptr->SetValue("True");
		else ptr->SetValue("False");

    cout << setting << " set to " << val << endl;
    
}
