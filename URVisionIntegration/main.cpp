#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <string>
#include <ur5.h>

//method
#define COLORSEG 0
#define HOUGH 1

//object
#define BALL 0
#define TARGET 1
/*
cv::Mat takePicture()
{
    int myExposure = 30000;
    cv::Mat imgUndistorted;
    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.

        Pylon::DeviceInfoList_t devices;
        Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
        if (devices.size() == 0) {
            std::cerr << "No devices found." << std::endl;
            return imgUndistorted;
        }

        Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateDevice(devices[0]));


        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap = camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width = nodemap.GetNode("Width");
        GenApi::CIntegerPtr height = nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;


        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto(nodemap.GetNode("ExposureAuto"));
        if (GenApi::IsWritable(exposureAuto)) {
            exposureAuto->FromString("Off");
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        if (exposureTime.IsValid()) {
            if (myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }
            else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }
        else {

            std::cout << ">> Failed to set exposure value." << std::endl;
            return imgUndistorted;
        }

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // image grabbing loop
        int frame = 1;
        if (camera.IsGrabbing())
        {

            //edit code here to grab just single image instead of continuous

            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;

                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);

                // Create an OpenCV image from a pylon image.
                openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());



                //////////////////////////////////////////////////////
                //////////// Here your code begins ///////////////////
                //////////////////////////////////////////////////////





                //calibration done
                //-----------------------------------------------------------------------------------------------------------------
                //hardcoding of K and k
                //Hardcoded camera matrix (K) and distortion coefficients (k)
                cv::Matx33f K = cv::Matx33f(1193.5651, 0, 751.68048,
                    0, 1194.4763, 588.64459,
                    0, 0, 1);

                cv::Vec<float, 5> k = { -0.261067, 0.282563, 0.000727901, 0.00017646, -0.387404 };

                cv::Size frameSize(1440, 1080); // Define the size of the images used in calibration



                // To be faster with a large nr of pictures, we use cv::initUndistortRectifyMap()
                // 6. Compute the undistortion map for lens correction
                cv::Mat mapX, mapY;
                // Initialize the undistortion map using the camera matrix and distortion coefficients
                cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

                //-------------------------------------------------------------------------------------------------
                // Den gør billederne "gode" her med de opnåede koefficienter
                // så evt indfør at den bare loader billederne til billedgenkendelse???
                //-------------------------------------------------------------------------------------------------

                // 7. Undistort and display the images


                // Apply the undistortion map to correct lens distortion
                cv::remap(openCvImage, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

                ////////////////////////////////////////////////////
                //////////// Here your code ends ///////////////////
                ////////////////////////////////////////////////////

            }

        }
        return imgUndistorted;
    }
    catch (GenICam::GenericException& e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
            << e.GetDescription() << std::endl;
        exitCode = 1;
    }
}

std::vector<double> findObject(cv::Mat& image, int method, int object)
{


    if (method == COLORSEG)
    {
        //init subimages
        cv::Mat HSVImage;
        cv::Mat maskedImage;
        cv::Mat isolatedImage;

        //convert from bgr to hsv
        cv::cvtColor(image, HSVImage, cv::COLOR_BGR2HSV);

        //define color mask thresholds
        cv::Scalar lowVal(40, 128, 128);
        cv::Scalar highVal(80, 255, 255);

        //set pixels in range to white, not to black
        cv::inRange(HSVImage, lowVal, highVal, maskedImage);

        //openCV find contours in the image
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(maskedImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        //find the largest area contour
        int largestArea = 0;
        int largestIndex = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            int area = cv::contourArea(contours[i]);
            if (area > largestArea)
            {
                largestArea = area;
                largestIndex = i;
            }

        }

        //find the bounding box of the largest area
        cv::Rect boundingBox = cv::boundingRect(contours[largestIndex]);

        //draw the bounding box
        cv::rectangle(maskedImage, boundingBox, cv::Scalar(60, 255, 255), 2);

        //define the center points of the bounding box
        int centerX = boundingBox.x + boundingBox.width / 2;
        int centerY = boundingBox.y + boundingBox.height / 2;

        //draw a small circle on the center point
        cv::circle(maskedImage, cv::Point(centerX, centerY), 5, cv::Scalar(60, 255, 255), -1);

        //define hardcoded camera -> world transformation
        cv::Matx33f H = cv::Matx33f(8.791885239048835e-05, -0.1265589388129923, 112.6477076592656,
            -0.1267161859667859, 4.388964932825915e-05, 112.8546197541133,
            -5.023934422312653e-06, -2.507979961614809e-06, 1);
        //indsæt de koordinater vi får fra billedgenkendelse i billed planet her? ------------------


        // Now, we can use the homography matrix to transform a point.
        //transform the center point previously found
        cv::Point2f pixelPoint(centerX, centerY);

        // Convert the point to homogeneous coordinates (add 1 to the point)
        std::vector<cv::Point2f> pixelPoints;
        pixelPoints.push_back(pixelPoint);

        std::vector<cv::Point2f> realWorldTransformedPoints;

        // Apply the homography to the point
        cv::perspectiveTransform(pixelPoints, realWorldTransformedPoints, H);

        // The resulting real world coordinates
        std::cout << "The transformed real world coordinates are: ("
            << realWorldTransformedPoints[0].x << ", "
            << realWorldTransformedPoints[0].y << ")" << std::endl;

        return { realWorldTransformedPoints[0].x, realWorldTransformedPoints[0].y };
    }
    else if (method == HOUGH)
    {
        //init subimages
        cv::Mat grayImage;

        //convert from bgr to gray
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

        //smoothing
        cv::GaussianBlur(grayImage, grayImage, cv::Size(3, 3), 0, 0);

        std::vector<cv::Vec3f> circles;
        HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1,
            grayImage.rows / 16,  // change this value to detect circles with different distances to each other
            100, 30, 14, 17 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
        );

        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            // circle center
            circle(image, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
            // circle outline
            int radius = c[2];
            circle(image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
        }


        //define hardcoded camera -> world transformation
        cv::Matx33f H = cv::Matx33f(-0.008453171254899592, -0.1027136663102614, 100.4104391989221,
            -0.1108970334507943, 1.095836764117068e-05, 100.503555926516,
            -0.0002133131348338028, -1.340640846687117e-05, 1);
        //indsæt de koordinater vi får fra billedgenkendelse i billed planet her? ------------------

        //sort circles by radius
        std::sort(circles.begin(), circles.end(), [](cv::Vec3f& a, cv::Vec3f& b)
            {
            return a[2] < b[2];
            }
        );


        //consider further segmenting, like still by color, example:
        //average color value inside circle, grab the one closest to ping pong ball

        //consider covering holes for even symmetric smooth surface


        //define center points under assumption of only one circle
        double centerX = circles[object][0];
        double centerY = circles[object][1];

        // Now, we can use the homography matrix to transform a point.
        //transform the center point previously found
        cv::Point2f pixelPoint(centerX, centerY);

        // Convert the point to homogeneous coordinates (add 1 to the point)
        std::vector<cv::Point2f> pixelPoints;
        pixelPoints.push_back(pixelPoint);

        std::vector<cv::Point2f> realWorldTransformedPoints;

        // Apply the homography to the point
        cv::perspectiveTransform(pixelPoints, realWorldTransformedPoints, H);
        std::cout << "radius is: " << circles[object][2] << '\n';
        //imshow("gray", grayImage);
        cv::resize(image, image, image.size()/2, cv::INTER_LINEAR);
        imshow("notGray", image);
        //cv::waitKey(0);
        return { realWorldTransformedPoints[0].x, realWorldTransformedPoints[0].y };
    }

}
*/
int main()
{

    UR5 UR;
    std::vector<double> jPos = {0, M_PI/3,0 ,M_PI/4, M_PI/6.0, 0};
    std::cout << UR.getJacobean(jPos) << std::endl;

//    UR.moveL(0, 0, 0, 0);

//    cv::Mat img = takePicture();
//    std::vector<double> ballCoordinates = findObject(img, HOUGH, BALL);
//    std::cout << "x: " << ballCoordinates[0]*0.01 << " y:" << ballCoordinates[1]*0.01 << std::endl;

//    UR.moveL(ballCoordinates[0]*0.01, ballCoordinates[1]*0.01, 0.2, 0);
//    UR.moveL(ballCoordinates[0]*0.01, ballCoordinates[1]*0.01, 0, 0);
//    UR.gripper_grip();
//    UR.moveL(0, 0, 0.2, 0);
//    UR.gripper_release(20);

//    cv::waitKey(0);

    //X,Y,Z,TCPAngle
//    UR.moveL(0, 0, 0, 0);
//    UR.gripper_grip();
//    UR.moveL(0.2, 0.2, 0, -45);
//    UR.gripper_release(80);

    return 0;
}
