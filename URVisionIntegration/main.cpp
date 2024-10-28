#include <QCoreApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <string>

// ------------------- UR RTDE ------------
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

// --------------------- EIGEN MATHS ---------------------------------
//sudo apt-get install libeigen3-dev
//https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <Eigen/Core>       //Basic Linear Algebra (Matrix+vectors)
#include <Eigen/Geometry>   //Basic Transformation, 2D, 3D rotations
#include <Eigen/LU>         //Inverse ect

double A2R(double angle)
{
    return angle*(M_PI/180);
}

double R2A(double radian)
{
    return (radian*180)/M_PI;
}

std::vector<double> findBall()
{
    std::vector<double> coordinates;
    //take at least one picture, maybe multiple




    int myExposure = 30000;

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
            return { 1,1 };
        }

        std::cout << "Found " << devices.size() << " devices." << std::endl;
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
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
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
            return { 2,2 };
        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // image grabbing loop
        int frame = 1;
        if(camera.IsGrabbing())
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


                cv::Mat imgUndistorted;
                // Apply the undistortion map to correct lens distortion
                cv::remap(openCvImage, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);

                //cv::setMouseCallback("undistorted image", onMouse, nullptr);
                // Display the undistorted image
                // cv::imshow("undistorted image", imgUndistorted);
                //cv::waitKey(0); // Wait for a key press to continue






            // Create an OpenCV display window.
                cv::namedWindow("myWindow", cv::WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO

                // Display the current image in the OpenCV display window.
                //cv::imshow("myWindow", openCvImage);

                // Detect key press and quit if 'q' is pressed
                int keyPressed = cv::waitKey(1);
                if (keyPressed == 'q')
                { //quit
                    std::cout << "Shutting down camera..." << std::endl;
                    camera.Close();
                    std::cout << "Camera successfully closed." << std::endl;
                }


                //i actually code here


                //init subimages
                cv::Mat HSVImage;
                cv::Mat maskedImage;
                cv::Mat isolatedImage;

                //convert from bgr to hsv
                cv::cvtColor(imgUndistorted, HSVImage, cv::COLOR_BGR2HSV);

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

                cv::Mat resizedImage;

                //resize image to fit on monitor
                cv::resize(maskedImage, resizedImage, maskedImage.size() / 2, cv::INTER_LINEAR);

                //display image
                cv::imshow("resized", resizedImage);

                //define hardcoded camera -> world transformation
                cv::Matx33f H = cv::Matx33f(-0.1058731429056703, -0.01136285388352953, 120.5186046992974,
                    1.688552128707462e-17, -0.117067944181413, 120.3458466184926,
                    -3.892943794818235e-06, -0.0001923031532593975, 1);
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
                coordinates.push_back(realWorldTransformedPoints[0].x);
                coordinates.push_back(realWorldTransformedPoints[0].y);


                ////////////////////////////////////////////////////
                //////////// Here your code ends ///////////////////
                ////////////////////////////////////////////////////

            }

        }

    }
    catch (GenICam::GenericException& e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
            << e.GetDescription() << std::endl;
        exitCode = 1;
    }



    return {3,3};
}

int main()
{
    std::string ip = "192.168.1.54"; //UR
    //std::string ip = "192.168.56.101"; //UR sim
    ur_rtde::RTDEControlInterface rtde_control(ip);
    rtde_control.setWatchdog(0.0043);
    ur_rtde::RTDEIOInterface rtde_IO(ip);
    ur_rtde::RTDEReceiveInterface rtde_recv(ip);


    // ---------------------------------- Calculate Transformation Matrix ------------------------------------

    // Define points
    double x1 = -0.07983, y1 = -0.46546, z1 = 0.160;
    double x2 = 0.07224, y2 = -0.83567, z2 = 0.160;

    // Compute ux (vector from point 1 to point 2)
    Eigen::Vector3d ux(x2 - x1, y2 - y1, z2 - z1);

    // Normalize ux
    ux.normalize();

    // Define uz as the unit vector along the z-axis
    Eigen::Vector3d uz(0, 0, 1);

    // Compute uy as the cross product of uz and ux
    Eigen::Vector3d uy = uz.cross(ux);

    // Define Q (the origin point for the transformation matrix)
    Eigen::Vector3d Q(x1, y1, z1);

    // Construct the 4x4 transformation matrix T_BH
    Eigen::Matrix4d T_BH;
    T_BH << ux(0), uy(0), uz(0), Q(0),
           ux(1), uy(1), uz(1), Q(1),
           ux(2), uy(2), uz(2), Q(2),
           0,     0,     0,     1;

    Eigen::Matrix4d T_BH_INV = T_BH.inverse();


    // ---------------------------------- INIT -------------------------------------

    //Calculate angle from rotation matrix ect.
    Eigen::Vector3d x(1, 0, 0);
    double initAngle = acos(x.dot(ux));
    std::vector<double> currentJointPos, targetJointPos;

    //Move to home position
    Eigen::Vector4d P_H(0.0, 0.2, 0.2, 1);  //Point int world coordinates
    Eigen::Vector4d P_B = T_BH * P_H;          //Point in Robot coordinates
    rtde_control.moveL({P_B(0), P_B(1), P_B(2), 0, A2R(-180), 0});


    // --------------------------------- Make new orientation for TCP --------------------

    //Add angle-90degrees to tool-flange-joint & move
    double targetAngle = A2R(0); //x decrees
    currentJointPos = rtde_recv.getActualQ();
    targetJointPos = currentJointPos;
    targetJointPos[5] = targetJointPos[5]+initAngle-M_PI/2+targetAngle;
    rtde_control.moveJ(targetJointPos);

    //Use tcpOri to orientate tcp
    Eigen::Vector3d tcpOri;
    tcpOri(0) = rtde_recv.getActualTCPPose()[3];
    tcpOri(1) = rtde_recv.getActualTCPPose()[4];
    tcpOri(2) = rtde_recv.getActualTCPPose()[5];

    // ---------------------------------- PROGRAM ------------------------------------

    std::vector<double> ballCoordinates = findBall();
    std::cout << "x: " << ballCoordinates[0] << " y:" << ballCoordinates[1] << std::endl;
    Eigen::Vector4d P_H1(ballCoordinates[0]*0.01, ballCoordinates[1]*0.01, 0, 1);  //Point int world coordinates
    Eigen::Vector4d P_B1 = T_BH * P_H1;          //Point in Robot coordinates

    rtde_control.moveL({P_B1(0),P_B1(1),P_B1(2), tcpOri(0), tcpOri(1), tcpOri(2)}); //Go to cordinates & align tcp 0degress

    return 0;
}
