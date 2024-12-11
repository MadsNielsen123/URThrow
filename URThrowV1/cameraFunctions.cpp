#include "cameraFunctions.h"



cv::Mat takePicture()
{
    //cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT); //Doesnt work
    int myExposure = 30000;
    cv::Mat imgUndistorted;

    Pylon::PylonAutoInitTerm autoInitTerm;


    // Create an instant camera object with the camera device found first.

    Pylon::DeviceInfoList_t devices;
    Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
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


    // Set custom exposure
    GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
    exposureTime->SetValue(myExposure);


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

        // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {

            // Convert the grabbed buffer to a pylon image.
            formatConverter.Convert(pylonImage, ptrGrabResult);

            // Create an OpenCV image from a pylon image.
            openCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());


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


        }

    }
    return imgUndistorted;

}

std::vector<Eigen::Vector3d> findBalls(cv::Mat& image)
{

        int minRadiusBall = 14;
        int maxRadiusBall = 17;


        //GrayImage
        std::vector<cv::Mat> channels(3);
        cv::split(image, channels);
        cv::Mat grayImage = channels[1];

        //cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);


        cv::imshow("Bolde", grayImage);
        //smoothing
        cv::GaussianBlur(grayImage, grayImage, cv::Size(3, 3), 0, 0);


        //Hough Detection
        int ballDistance = grayImage.rows / 16; // change this value to detect circles with different distances to each other
        std::vector<cv::Vec3f> circles;
        HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1, ballDistance, 100, 30, minRadiusBall, maxRadiusBall);

        if(circles.size() < 1)
            std::cout << "Fandt ingen bolde" << std::endl;


        //Draw balls
        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            // circle center
            circle(image, center, 1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
            // circle outline
            int radius = c[2];
            circle(image, center, radius, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }

        //indsæt de koordinater vi får fra billedgenkendelse i billed planet her? ------------------

        //sort circles by radius
        std::sort(circles.begin(), circles.end(), [](cv::Vec3f& a, cv::Vec3f& b)
            {
                return a[2] < b[2];
            }
        );


        std::vector<cv::Point2f> pixelPoints;

        for (int i = 0; i < circles.size(); i++)
        {
            pixelPoints.emplace_back(circles[i][0], circles[i][1]);
        }

        //Transform points to world coordinates
        cv::Matx33f H = cv::Matx33f(-0.008453171254899592 , -0.1027136663102614   , 100.4104391989221,
                                    -0.1108970334507943   ,  1.095836764117068e-05, 100.503555926516,
                                    -0.0002133131348338028, -1.340640846687117e-05, 1);

        cv::Matx33f K = cv::Matx33f(1193.5651, 0        , 751.68048,
                                    0        , 1194.4763, 588.64459,
                                    0        , 0        , 1);

        std::vector<cv::Point2f> realWorldTransformedPoints;
        cv::perspectiveTransform(pixelPoints, realWorldTransformedPoints, H);

        std::vector<Eigen::Vector3d> coordinates;
        for (int i = 0; i < realWorldTransformedPoints.size(); i++)
        {
            //Scale cm -> meter
            realWorldTransformedPoints[i].x *= 0.01;
            realWorldTransformedPoints[i].y *= 0.01;

            //Save point as Eigen 3D-vector
            coordinates.push_back({realWorldTransformedPoints[i].x, realWorldTransformedPoints[i].y, 0}); //ball heigh: 0

        }

        return coordinates;

}

std::vector<Eigen::Vector3d> findCups(cv::Mat& image)
{

    int minRadiusCup = 32;
    int maxRadiusCup = 40;


    //GrayImage
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    cv::imshow("Found this: ", grayImage);
    cv::waitKey(0);
    //smoothing
    cv::GaussianBlur(grayImage, grayImage, cv::Size(3, 3), 0, 0);

    //Hough Detection
    int cupdistance = grayImage.rows / 16; // change this value to detect circles with different distances to each other
    std::vector<cv::Vec3f> circles;
    HoughCircles(grayImage, circles, cv::HOUGH_GRADIENT, 1, cupdistance, 100, 30, minRadiusCup, maxRadiusCup);

    if(circles.size() < 1)
        std::cout << "Fandt inge cups";


    std::vector<cv::Vec3f> filteredCircles;
    for (int i = 0; i < circles.size(); i++)
    {
        if (circles[i][0] > 380 && circles[i][0] < 1030 && circles[i][1] < 700)//lower final value for cups further towards robot
            filteredCircles.emplace_back(circles[i]);
    }

    circles = filteredCircles;

    //Draw in circles
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        circle(image, center, 1, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        circle(image, center, radius, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    }

    cv::imshow("Found this: ", grayImage);
    cv::waitKey(0);

    //sort circles by radius
    std::sort(circles.begin(), circles.end(), [](cv::Vec3f& a, cv::Vec3f& b){return a[2] < b[2];});


    std::vector<cv::Point2f> pixelPoints;

    for (int i = 0; i < circles.size(); i++)
    {
        pixelPoints.emplace_back(circles[i][0], circles[i][1]);
    }


    //Transform points to world coordinates
    cv::Matx33f H = cv::Matx33f (-0.007791881465124858 , -0.09948645122707399  , 99.89002391428173,
                                 -0.1075931183674724   , -0.0002392212860721122, 101.6042958591801,
                                 -0.0001785793076406061,  2.295218118932419e-05, 0.9999999999999999);

    cv::Matx33f K = cv::Matx33f(1193.5651,  0,          751.68048,
                                0,          1194.4763,  588.64459,
                                0,          0,          1);


    std::vector<cv::Point2f> realWorldTransformedPoints;
    cv::perspectiveTransform(pixelPoints, realWorldTransformedPoints, H);


    std::vector<Eigen::Vector3d> coordinates;
    for (int i = 0; i < realWorldTransformedPoints.size(); i++)
    {
        //Scale cm -> meter
        realWorldTransformedPoints[i].x *= 0.01;
        realWorldTransformedPoints[i].y *= 0.01;

        //Save point as Eigen 3D-vector
        coordinates.push_back({realWorldTransformedPoints[i].x, realWorldTransformedPoints[i].y, 0.11}); //cup height: 11 cm heigh

    }

    return coordinates;

}
