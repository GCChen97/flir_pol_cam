//=============================================================================
// Copyright (c) 2001-2021 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
 *  @example Polarization.cpp
 *
 *  @brief Polarization.cpp shows how to extract and create images from a source image of
 *  Polarized8 or BayerRGPolarized8 pixel format using methods from the ImageUtilityPolarization,
 *  ImageUtility and ImageUtilityHeatmap classes.
 *  It relies on information provided in the Enumeration, Acquisition, and NodeMapInfo examples.
 *
 *  This example demonstrates some of the methods that can be used to extract polarization quadrant
 *  images and create Stokes', AoLP, and DoLP images from the ImageUtilityPolarization class.
 *  It then demonstrates how to use some of the available methods in the ImageUtility and
 *  ImageUtilityHeatmap classes to create normalized and heatmap images.
 *
 *  Polarization is only available for polarized cameras. For more information
 *  please visit our website;
 * https://www.flir.com/discover/iis/machine-vision/imaging-reflective-surfaces-sonys-first-polarized-sensor
 */

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <string>
#include <array>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

static bool isPixelFormatColor = false;

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap)
{
    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            for (FeatureList_t::const_iterator it = features.begin(); it != features.end(); ++it)
            {
                const CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}

// This function sets the pixel format to a Polarized pixel format, and acquisition mode to single frame.
int ConfigureStream(INodeMap& nodeMap)
{
    //
    // Set the pixel format to Polarized8 or BayerRGPolarized8
    //
    // *** NOTES ***
    // Methods in the ImageUtilityPolarization class are supported for images of pixel format
    // Polarized8 and BayerRGPolarized8. These formats are only available on the polarized camera.
    // For more in-depth comments on formatting images, see the ImageFormatControl example.

    // Retrieve the enumeration node from the nodemap
    CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
    if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
    {
        // Retrieve the desired entry node from the enumeration node
        CEnumEntryPtr ptrPixelFormatPolarized8 = ptrPixelFormat->GetEntryByName("Polarized8");
        CEnumEntryPtr ptrPixelFormatBayerRGPolarized8 = ptrPixelFormat->GetEntryByName("BayerRGPolarized8");
        if (IsAvailable(ptrPixelFormatPolarized8) && IsReadable(ptrPixelFormatPolarized8))
        {
            // Retrieve the integer value from the entry node
            const int64_t pixelFormatPolarized8 = ptrPixelFormatPolarized8->GetValue();

            // Set integer as new value for enumeration node
            ptrPixelFormat->SetIntValue(pixelFormatPolarized8);

            isPixelFormatColor = false;

            cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
        }
        else if (IsAvailable(ptrPixelFormatBayerRGPolarized8) && IsReadable(ptrPixelFormatBayerRGPolarized8))
        {
            // Retrieve the integer value from the entry node
            const int64_t pixelFormatBayerRGPolarized8 = ptrPixelFormatBayerRGPolarized8->GetValue();

            // Set integer as new value for enumeration node
            ptrPixelFormat->SetIntValue(pixelFormatBayerRGPolarized8);

            isPixelFormatColor = true;

            cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
        }
        else
        {
            // Methods in the ImageUtilityPolarization class are supported for images of
            // polarized pixel formats only.
            cout << "Pixel format Polarized8 or BayerRGPolarized8 not available (entry retrieval). Aborting..." << endl;
            return -1;
        }
    }
    else
    {
        // Methods in the ImageUtilityPolarization class are supported for images of
        // polarized pixel formats only.
        cout << "Pixel format not available (enum retrieval). Aborting..." << endl;
        return -1;
    }

    // Set acquisition mode to single frame
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        cout << "Unable to set acquisition mode to single frame (enum retrieval). Aborting..." << endl << endl;
        return -1;
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeSingleFrame = ptrAcquisitionMode->GetEntryByName("SingleFrame");
    if (!IsAvailable(ptrAcquisitionModeSingleFrame) || !IsReadable(ptrAcquisitionModeSingleFrame))
    {
        cout << "Unable to set acquisition mode to single frame (entry retrieval). Aborting..." << endl << endl;
        return -1;
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeSingleFrame = ptrAcquisitionModeSingleFrame->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeSingleFrame);

    cout << "Acquisition mode set to single frame..." << endl;

    return 0;
}

// This function saves an image and prints some information.
// The serial number will be prepended to the filename if it is not empty.
int SaveImage(const ImagePtr& pImage, const string filename, gcstring& serialNumber)
{
    try
    {
        string fullFilename;
        if (!serialNumber.empty())
        {
            // Prepend the filename with the serial number
            fullFilename = serialNumber.c_str() + filename;
            // Add a hyphen between the serial number and original filename
            fullFilename.insert(serialNumber.length(), 1, '-');
        }
        else
        {
            fullFilename = filename;
        }

        // Save the image and print image info
        pImage->Save(fullFilename.c_str());
        cout << "Image saved at " << fullFilename << endl;
        cout << "Width = " << pImage->GetWidth() << ", height = " << pImage->GetHeight()
             << ", pixel format = " << pImage->GetPixelFormatName() << endl
             << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    return 0;
}

// This function returns a string of the specified polarization quadrant appendage.
std::string GetQuadFileNameAppendage(const ImageUtilityPolarization::PolarizationQuadrant quadrant)
{
    switch (quadrant)
    {
    case ImageUtilityPolarization::QUADRANT_I0:
        return "I0";
    case ImageUtilityPolarization::QUADRANT_I45:
        return "I45";
    case ImageUtilityPolarization::QUADRANT_I90:
        return "I90";
    case ImageUtilityPolarization::QUADRANT_I135:
        return "I135";
    default:
        return "UNKNOWN_QUAD";
    }
}

// This function creates and saves a heatmap image using the ImageUtilityHeatmap class.
// The function demonstrates setting the heatmap gradient and range.
int CreateHeatmapImages(const ImagePtr& mono8Image, const string baseFilename, gcstring& deviceSerialNumber)
{
    try
    {
        //
        // Set the heatmap color gradient and range.
        //
        // *** NOTES ***
        // By default the heatmap gradient will be set from HEATMAP_BLACK to HEATMAP_WHITE, and the
        // range from 0 to 100 percent radiance. Changes to the heatmap can be visualized in SpinView
        // using the 'Configure Heatmap Gradient' tool when streaming with any heatmap polarization
        // algorithm applied.
        // (ex. Heatmap (AoLP)). Below are the optional functions available to modify the heatmap.
        //
        ImageUtilityHeatmap::SetHeatmapColorGradient(
            ImageUtilityHeatmap::HEATMAP_BLACK, ImageUtilityHeatmap::HEATMAP_WHITE);

        //
        // *** NOTES ***
        // The heatmap can be manipulated to focus on a portion of the calculated range (from 0 to 100%).
        // The radiance of the heatmap describes the percent linear polarization for DoLP images, the
        // degree of linear polarization for AoLP images (from -90 to 90), and the percent radiance for
        // Stokes' parameters. Note that AoLP angles need to be expressed as a percentage of the maximum
        // range (-90 to 90) before being used as inputs to this function. In SpinView the percent is
        // shown in brackets in the range slider tool tip.
        // Converting from the range of (-90 to 90) deg to (0 to 100) percent is shown:
        //     degrees = (percent / 100) * 180 - 90
        //     percent = (degrees + 90) * 100 / 180
        //
        ImageUtilityHeatmap::SetHeatmapRange(0, 100);

        // Create a heatmap image and save it
        //
        // *** NOTES ***
        // Creating heatmap images is not exclusive to polarized cameras!
        // Any image of pixel format Mono8 or Mono16 can be used to create a heatmap image.
        //
        const auto heatmapImage = ImageUtilityHeatmap::CreateHeatmap(mono8Image);
        SaveImage(heatmapImage, (baseFilename + "_Heatmap.jpg"), deviceSerialNumber);
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 1;
}

// This function extracts polarization quadrant images using the ImageUtilityPolarization class.
// It then calls helper function CreateHeatmapImages on all monochrome polarization quadrant images.
int ExtractAndSavePolarQuadImages(const ImagePtr& pRawPolarizedImage, gcstring& deviceSerialNumber)
{
    try
    {
        // Define an array of polarization quadrant enums to use in ExtractPolarQuadrant method
        array<ImageUtilityPolarization::PolarizationQuadrant, 4> polarizationQuadEnums = {
            ImageUtilityPolarization::QUADRANT_I0,
            ImageUtilityPolarization::QUADRANT_I45,
            ImageUtilityPolarization::QUADRANT_I90,
            ImageUtilityPolarization::QUADRANT_I135};

        for (auto polarizationQuadEnum = polarizationQuadEnums.begin();
             polarizationQuadEnum != polarizationQuadEnums.end();
             ++polarizationQuadEnum)
        {
            // Save a string that describes the image being saved
            const string quadrantName = "Quadrant_" + GetQuadFileNameAppendage(*polarizationQuadEnum);

            // Extract the polarization quadrant image and save it
            //
            // *** NOTES ***
            // Polarization quadrant images are unaltered source data extracted into images that
            // represent all pixels with a polarizing filter of the specified orientation.
            // i.e. 0 deg polarization = QUADRANT_I0.
            // This means that each extracted image will be a quarter the size of the source image,
            // as each type of polarizing filter covers a fourth of the sensors photodiodes.
            // Polarization quadrant images are extracted as Mono8 and BayerRG8 for monochrome and
            // color cameras respectively.
            //
            const auto polarizationQuadImage =
                ImageUtilityPolarization::ExtractPolarQuadrant(pRawPolarizedImage, *polarizationQuadEnum);
            SaveImage(polarizationQuadImage, (quadrantName + ".jpg"), deviceSerialNumber);

            // Save heatmap images for each Mono8 polarization quadrant images.
            if (!isPixelFormatColor)
            {
                CreateHeatmapImages(polarizationQuadImage, quadrantName, deviceSerialNumber);
            }
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 1;
}

// This function creates and saves an image with reduced glare using the ImageUtilityPolarization class.
int CreateAndSaveGlareReducedImage(const ImagePtr& pRawPolarizedImage, gcstring& deviceSerialNumber)
{
    try
    {
        // Create a glare reduced image and save it
        //
        // *** NOTES ***
        // When unpolarized light is incident upon a dielectric surface, the reflected portion of the light
        // is partially polarized according to Brewster's law. Selecting the filtered pixel that most effectively
        // blocks this polarized light in each pixel quadrant reduces glare in the overall image. Since one pixel
        // is selected from each 2x2 polarized pixel quadrant the resulting image will be a quarter of the raw
        // image's resolution.
        //
        const auto glareReducedImage = ImageUtilityPolarization::CreateGlareReduced(pRawPolarizedImage);

        SaveImage(glareReducedImage, "Glare_Reduced.jpg", deviceSerialNumber);
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 1;
}

// This function creates and saves a normalized image using the ImageUtility class.
// Monochrome and color images are normalized to Pixelformat_Mono8 and PixelFormat_RGB8 respectively
int CreateNormalizedImage(
    const ImagePtr& imageToNormalize,
    const string baseFilename,
    gcstring& deviceSerialNumber,
    ImageUtility::SourceDataRange srcDataRange = ImageUtility::IMAGE_DATA_RANGE)
{
    try
    {
        // Create a normalized image
        //
        // *** NOTES ***
        // Creating normalized images is not exclusive to polarized cameras!
        // Any image with image data (pixel format) of type of char, short, or float can be used to
        // create a normalized image.
        //
        const auto normalizedImage = ImageUtility::CreateNormalized(
            imageToNormalize, isPixelFormatColor ? PixelFormat_RGB8 : PixelFormat_Mono8, srcDataRange);
        SaveImage(normalizedImage, (baseFilename + "_Normalized.jpg"), deviceSerialNumber);
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 1;
}

// This function creates and saves raw and normalized Stokes' images using the
// ImageUtilityPolarization class.
int CreateAndSaveStokesImages(const ImagePtr& pRawPolarizedImage, gcstring& deviceSerialNumber)
{
    try
    {
        // Create Stokes' images using the appropriate function calls
        //
        // *** NOTES ***
        // Stokes' images add (S0) or subtract (S1, S2) polarization quadrant images. Therefore
        // each created image is a quarter the size of the source image.
        //
        // The algorithms are as follows:
        // S0 = I0  + I90  : The overall intensity of light
        // S1 = I0  - I90  : The difference in intensity accepted through the polarizers at 0 and 90
        //                   to the horizontal
        // S2 = I45 - I135 : The difference in intensity accepted through the polarizers at 45 and -45
        //                   to the horizontal
        //
        // The calculated Stokes' values can range from, 0 (S0) or -255 (S1, S2), to 510 and thus are
        // stored with pixel formats Mono16s or RGB16s, for monochrome and color cameras respectively.
        // These formats can only be saved using a raw file extension.
        //
        const auto stokesS0Image = ImageUtilityPolarization::CreateStokesS0(pRawPolarizedImage);
        const auto stokesS1Image = ImageUtilityPolarization::CreateStokesS1(pRawPolarizedImage);
        const auto stokesS2Image = ImageUtilityPolarization::CreateStokesS2(pRawPolarizedImage);

        // Add all raw Stokes' images to an array
        array<ImagePtr, 3> stokesImages = {stokesS0Image, stokesS1Image, stokesS2Image};

        // Save a stokes Appendage to create a descriptive filename
        long long stokesAppendage = 0;

        // Loop through raw Stokes' images, saving a raw and normalized copy
        for (auto stokesImage = stokesImages.begin(); stokesImage != stokesImages.end(); ++stokesImage)
        {
            const string stokesName = "Stokes_S" + to_string(stokesAppendage++);

            // Save the raw Stokes' images
            SaveImage(*stokesImage, (stokesName + ".raw"), deviceSerialNumber);

            // Create and save a normalized Stokes' image
            CreateNormalizedImage(
                *stokesImage, stokesName, deviceSerialNumber, ImageUtility::SourceDataRange::ABSOLUTE_DATA_RANGE);
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 1;
}

// This function creates and saves raw and normalized AoLP and DoLP images using the
// ImageUtilityPolarization class.
int CreateAndSaveAolpDolpImages(const ImagePtr& pRawPolarizedImage, gcstring& deviceSerialNumber)
{
    try
    {
        // Create and save AoLP and DoLP images using the appropriate function calls
        //
        // *** NOTES ***
        // The Angle of Linear Polarization, AoLP, and Degree of Linear Polarization, DoLP, are calculated
        // using Stokes' values. Therefore each created image is a quarter the size of the source image.
        //
        // The algorithms are as follows:
        // DoLP = ((S1pow(2) + S2pow(2))pow(1/2)) / S0 : The fraction of incident light intensity in
        //                                               the linear polarization states
        // AoLP = (1/2)* arctan( S2 / S1)              : The angle at which linearly polarized light
        //                                               oscillates with respect to a reference axis
        //
        // The calculated AoLP will range from -90 deg to 90 deg and DoLP values will range from 0 to 1
        // (float). Therefore the images are stored with pixel formats Mono32f or RGB32f, for monochrome
        // and color cameras respectively. These formats can only be saved using a raw file extension.
        //
        const auto aolpImage = ImageUtilityPolarization::CreateAolp(pRawPolarizedImage);
        SaveImage(aolpImage, "AoLP.raw", deviceSerialNumber);

        const auto dolpImage = ImageUtilityPolarization::CreateDolp(pRawPolarizedImage);
        SaveImage(dolpImage, "DoLP.raw", deviceSerialNumber);

        // Create and save normalized AoLP and DoLP images
        const auto aolpNormalizedImage = ImageUtility::CreateNormalized(
            aolpImage,
            isPixelFormatColor ? PixelFormat_RGB8 : PixelFormat_Mono8,
            ImageUtility::SourceDataRange::ABSOLUTE_DATA_RANGE);
        SaveImage(aolpNormalizedImage, "AoLP_Normalized.jpg", deviceSerialNumber);

        const auto dolpNormalizedImage = ImageUtility::CreateNormalized(
            dolpImage,
            isPixelFormatColor ? PixelFormat_RGB8 : PixelFormat_Mono8,
            ImageUtility::SourceDataRange::ABSOLUTE_DATA_RANGE);
        SaveImage(dolpNormalizedImage, "DoLP_Normalized.jpg", deviceSerialNumber);

        // Create and save AoLP and DoLP heatmaps for mono images
        if (!isPixelFormatColor)
        {
            CreateHeatmapImages(aolpNormalizedImage, "AoLP", deviceSerialNumber);
            CreateHeatmapImages(dolpNormalizedImage, "DoLP", deviceSerialNumber);
        }
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 1;
}

// This function acquires a raw polarized image and then extracts and creates images using methods from
// the ImageUtilityPolarization, ImageUtility and ImageUtilityHeatmap classes;
// please see Acquisition example for more in-depth comments on acquiring images.
int AcquireImages(CameraPtr pCam, INodeMap& nodeMapTLDevice)
{
    cout << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

    // Get a polarized image from the camera and use helper functions to create and save unique images
    try
    {
        // Begin acquiring images
        pCam->BeginAcquisition();

        cout << "Acquiring an image from the polarized camera..." << endl;

        // Retrieve device serial number for filename
        gcstring deviceSerialNumber("");

        CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();

            cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
        }
        cout << endl;

        // Retrieve the received raw image
        ImagePtr pRawPolarizedImage = pCam->GetNextImage(1000);

        // Ensure image completion
        if (pRawPolarizedImage->IsIncomplete())
        {
            cout << "Image incomplete with image status " << pRawPolarizedImage->GetImageStatus() << "..." << endl
                 << endl;
        }
        else
        {
            // Save a polarized reference image
            //
            // *** NOTES ***
            // SaveImage prepends the serial number to the filename and save the image
            //
            SaveImage(pRawPolarizedImage, "Raw_Polarized_Image.jpg", deviceSerialNumber);

            // Extract and save all polarization quadrants and create heatmap images for all
            // monochrome images
            ExtractAndSavePolarQuadImages(pRawPolarizedImage, deviceSerialNumber);

            // Create and save raw and normalized Stokes' images
            CreateAndSaveStokesImages(pRawPolarizedImage, deviceSerialNumber);

            // Create and save raw and normalized AoLP and DoLP images
            CreateAndSaveAolpDolpImages(pRawPolarizedImage, deviceSerialNumber);

            // Create and save an image with a simple glare reduction applied
            CreateAndSaveGlareReducedImage(pRawPolarizedImage, deviceSerialNumber);
        }

        // Release image
        pRawPolarizedImage->Release();
        cout << endl;
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    // End acquisition
    pCam->EndAcquisition();
    return 0;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam)
{
    int result = 0;

    try
    {
        // Retrieve TL device nodemap and print device information
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam->Init();

        // Retrieve GenICam nodemap
        INodeMap& nodeMap = pCam->GetNodeMap();

        // Set the pixel format and acquisition mode
        if (ConfigureStream(nodeMap) != -1)
        {
            // Acquire images
            result = result | AcquireImages(pCam, nodeMapTLDevice);
        }

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

// Example entry point; please see Enumeration example for additional
// comments on the steps in this function.
int main(int /*argc*/, char** /*argv*/)
{
    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE* tempFile = fopen("test.txt", "w+");
    if (tempFile == nullptr)
    {
        cout << "Failed to create file in current folder.  Please check "
                "permissions."
             << endl;
        cout << "Press Enter to exit..." << endl;
        getchar();
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    int result = 0;

    // Print Application Build Information
    cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Print out current library version
    const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
         << endl;

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Release camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "Not enough cameras!" << endl;
        cout << "Done! Press Enter to exit..." << endl;
        getchar();

        return -1;
    }

    // Run example on each camera
    for (unsigned int i = 0; i < numCameras; i++)
    {
        cout << endl << "Running example for camera " << i << "..." << endl;

        result = result | RunSingleCamera(camList.GetByIndex(i));

        cout << "Camera " << i << " example complete..." << endl << endl;
    }

    // Release camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    cout << endl << "Done! Press Enter to exit..." << endl;
    getchar();

    return result;
}