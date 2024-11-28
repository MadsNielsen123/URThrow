#include "cameraFunctions.h"



cv::Mat takePicture()
{
    //cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_SILENT);
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

std::vector<cv::Point2f> findObject(cv::Mat& image, int object, int method, int minRadiusBall, int maxRadiusBall, int minRadiusCup, int maxRadiusCup, int targetHeight)
{

    cv::Matx33f H = cv::Matx33f(-0.008453171254899592, -0.1027136663102614, 100.4104391989221,
        -0.1108970334507943, 1.095836764117068e-05, 100.503555926516,
        -0.0002133131348338028, -1.340640846687117e-05, 1);
    if (object == TARGET)
    {
        H = cv::Matx33f (-0.007791881465124858, -0.09948645122707399, 99.89002391428173,
                         -0.1075931183674724, -0.0002392212860721122, 101.6042958591801,
                         -0.0001785793076406061, 2.295218118932419e-05, 0.9999999999999999);
    }

    cv::Matx33f K = cv::Matx33f(1193.5651, 0, 751.68048,
        0, 1194.4763, 588.64459,
        0, 0, 1);


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

        return { { realWorldTransformedPoints[0].x, realWorldTransformedPoints[0].y } };
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
            100, 30, object ? minRadiusCup : minRadiusBall, object ? maxRadiusCup : maxRadiusBall // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
        );

        if(circles.size() < 1)
            std::cout << "Fandt ikke noget";


        if (object == TARGET)
        {
            std::vector<cv::Vec3f> filteredCircles;
            for (int i = 0; i < circles.size(); i++)
            {
                /*colorseg
                cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
                circle(mask, cv::Point(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(255), cv::FILLED);

                cv::Mat maskedImage;
                image.copyTo(maskedImage, mask);

                // Convert the masked region to grayscale
                cv::Mat grayMaskedImage;
                cvtColor(maskedImage, grayMaskedImage, cv::COLOR_BGR2GRAY);

                // Calculate mean intensity of the masked region
                cv::Scalar meanIntensity = mean(grayMaskedImage, mask);

                if (meanIntensity[0] > 200)//needs adjusting
                {
                    filteredCircles.emplace_back(circles[i]);
                }*/

                //location segmentation
                if (circles[i][0] > 380 && circles[i][0] < 1030 && circles[i][1] > 380 && circles[i][1] < 700)//lower final value for cups further towards robot
                    filteredCircles.emplace_back(circles[i]);
            }
            circles = filteredCircles;
        }

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
        std::vector<cv::Point2f> pixelPoints;
        std::vector<cv::Point2f> realWorldTransformedPoints;

        pixelPoints.emplace_back(circles[0][0], circles[0][1]);
        if (object == TARGET)
        {
            //cv::Matx33f Kinv = K.inv();
            //cv::Matx33f Hinv = H.inv();
            for (int i = 1; i < circles.size(); i++)
            {
                pixelPoints.emplace_back(circles[i][0], circles[i][1]);
                /*
                cv::Mat p_normalized = Kinv * (cv::Mat_<double>(3, 1) << circles[i][0], circles[i][1], 1);
                cv::Mat ray_world = Hinv * p_normalized;
                ray_world /= ray_world.at<double>(2, 0);
                double scale = targetHeight / ray_world.at<double>(2, 0);
                realWorldTransformedPoints.emplace_back(
                    scale * ray_world.at<double>(0, 0),
                    scale * ray_world.at<double>(1, 0)
                );*/
            }
            //return realWorldTransformedPoints;
        }

        cv::perspectiveTransform(pixelPoints, realWorldTransformedPoints, H);


        for (int i = 0; i < realWorldTransformedPoints.size(); i++)
        {
            realWorldTransformedPoints[i].x *= 0.01;
            realWorldTransformedPoints[i].y *= 0.01;
        }

        //std::cout << "the smallest circle radius is cm: " << circles[0][2] * 0.01 << std::endl;
        return realWorldTransformedPoints;

    }

}
