#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int clicks = 0;


// Mouse callback function to capture pixel coordinates in the picture frame
void onMouse(int event, int x, int y, int flags, void* userdata) {
/*
    cv::Matx33f X = cv::Matx33f    (-0.008453171254899592, -0.1027136663102614, 100.4104391989221,
                                -0.1108970334507943, 1.095836764117068e-05, 100.503555926516,
                                -0.0002133131348338028, -1.340640846687117e-05, 1);

    //GetPerspectiveTransform
    cv::Matx33f H = cv::Matx33f (8.791885239048835e-05, -0.1265589388129923, 112.6477076592656,
                                -0.1267161859667859, 4.388964932825915e-05, 112.8546197541133,
                                -5.023934422312653e-06, -2.507979961614809e-06, 1);
*/

    if (event == cv::EVENT_LBUTTONDOWN) {

        cv::Point2f pixelPoint(x, y);

        /*
        // Convert the point to homogeneous coordinates (add 1 to the point)
        std::vector<cv::Point2f> pixelPoints;
        pixelPoints.push_back(pixelPoint);

        std::vector<cv::Point2f> realWorldTransformedPointsX;
        std::vector<cv::Point2f> realWorldTransformedPointsH;

        // Apply the homography to the point
        cv::perspectiveTransform(pixelPoints, realWorldTransformedPointsX, X);
        cv::perspectiveTransform(pixelPoints, realWorldTransformedPointsH, H);

        int worldX = (clicks*5)%65;
        int worldY = (clicks*5)/65*5;
        double errorX = (realWorldTransformedPointsX[0].x - worldX) * (realWorldTransformedPointsX[0].x - worldX) + (realWorldTransformedPointsX[0].y - worldY) * (realWorldTransformedPointsX[0].y - worldY);
        double errorH = (realWorldTransformedPointsH[0].x - worldX ) * (realWorldTransformedPointsH[0].x - worldX) + (realWorldTransformedPointsH[0].y - worldY ) * (realWorldTransformedPointsH[0].y - worldY );

        std::cout << "(" << worldX << ";" << worldY<< "),(" << errorX << ";" << errorH << ")\n";
*/
        std::cout << "Mouse clicked at: (" << x << ", " << y << ")" << std::endl;

        clicks++;
    }






}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    // 1. Find all images matching the specified pattern
    std::vector<cv::String> fileNames;
    // glob() is used to find all image files in the specified directory
    cv::glob("../../../Billede/ *.png", fileNames, false);

    // Define the pattern size of the chessboard (number of inner corners per row and column)
    cv::Size patternSize(10 - 1, 7 - 1);
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());

    // Iterate over all images to detect chessboard corners
    std::size_t i = 0;
    for (auto const &f : fileNames) {
        std::cout << std::string(f) << std::endl;

        // 2. Load image in grayscale mode
        cv::Mat img = cv::imread(f, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "Error loading image: " << f << std::endl;
            continue; // Skip to the next image if the current one cannot be loaded
        }

        // Vector to hold the detected corner points
        std::vector<cv::Point2f> corners;
        // Detect chessboard corners in the grayscale image
        bool success = cv::findChessboardCorners(img, patternSize, corners,
                                                 cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (success) {
            // 3. Refine the detected corners to sub-pixel accuracy

            //slet?
            //cv::Mat imgColor; // Create a color image for visualization
            //cv::cvtColor(img, imgColor, cv::COLOR_GRAY2BGR); // Convert grayscale to color image for drawing corners

            // Refine the corner positions to sub-pixel accuracy
            cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.001));

            q[i] = corners; // Store the refined corners in the vector

            //slet?
            // Draw the detected and refined chessboard corners on the image
           // cv::drawChessboardCorners(imgColor, patternSize, corners, success);
        }
        i++;
    }

    // 4. Generate the world coordinates of the checkerboard corners
    std::vector<std::vector<cv::Point3f>> Q;
    std::vector<cv::Point3f> obj;
    // Define the 3D coordinates of the chessboard corners assuming each square is 70x70 mm
    for (int i = 0; i < patternSize.height; ++i) {
        for (int j = 0; j < patternSize.width; ++j) {
            obj.push_back(cv::Point3f(j * 70.0f, i * 70.0f, 0)); // X and Y coordinates are in mm, Z is zero
        }
    }

    // Initialize the world coordinates for each image
    for (std::size_t i = 0; i < fileNames.size(); i++) {
        Q.push_back(obj); // Each element in Q corresponds to a view/image
    }

    // 5. Initialize camera matrix and distortion coefficients
    cv::Matx33f K(cv::Matx33f::eye());  // Intrinsic camera matrix (identity matrix for initialization)
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // Distortion coefficients (initialized to zero)

    // Vectors to hold the output of the calibration
    std::vector<cv::Mat> rvecs, tvecs; // Rotation and translation vectors
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors; // Standard deviations and per-view errors
    cv::Size frameSize(1440, 1080); // Define the size of the images used in calibration

    std::cout << "Calibrating..." << std::endl;
    // Perform camera calibration
    double error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics, perViewErrors);
    std::cout << "Reprojection error = " << error << "\nK =\n" << K << "\nk=\n" << k << std::endl;


    //calibration done
    //-----------------------------------------------------------------------------------------------------------------
    //hardcooding of K and k
    // Hardcoded camera matrix (K) and distortion coefficients (k)
    cv::Matx33f K = cv::Matx33f(1193.5651, 0, 751.68048,
                                0, 1194.4763, 588.64459,
                                0, 0, 1);

    cv::Vec<float, 5> k = {-0.261067, 0.282563, 0.000727901, 0.00017646, -0.387404};

    cv::Size frameSize(1440, 1080); // Define the size of the images used in calibration

    std::vector<cv::String> fileNames;
    cv::glob("../../../Billede kop/*.png", fileNames, false);  //------------------------ henter billederne ind her.

    // To be faster with a large nr of pictures, we use cv::initUndistortRectifyMap()
    // 6. Compute the undistortion map for lens correction
    cv::Mat mapX, mapY;
    // Initialize the undistortion map using the camera matrix and distortion coefficients
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);

    //-------------------------------------------------------------------------------------------------
    // Den gør billederne "gode" her med de opnåede koefficienter
    //-------------------------------------------------------------------------------------------------

    // 7. Undistort and display the images
    for (auto const &f : fileNames) {
        std::cout << std::string(f) << std::endl;

        cv::Mat img = cv::imread(f, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "Error loading image: " << f << std::endl;
            continue;
        }

        cv::Mat imgUndistorted;
        // Apply the undistortion map to correct lens distortion
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);


        cv::setMouseCallback("undistorted image", onMouse, nullptr);
        // Display the undistorted image
         cv::imshow("undistorted image", imgUndistorted);
        cv::waitKey(0); // Wait for a key press to continue

}


//--------------------------------------------------------------------------------------------------

//Real world transformation of coordiantes
// Image points (corresponding to real-world points)
std::vector<cv::Point2f> imagePoints;
    /*
imagePoints.push_back(cv::Point2f(906, 903)); // (0, 0)
imagePoints.push_back(cv::Point2f(906, 400)); // (0, 80)
imagePoints.push_back(cv::Point2f(400, 903)); // (80, 0)
imagePoints.push_back(cv::Point2f(399, 398)); // (80, 80)

*/ /*
imagePoints.push_back(cv::Point2f(906, 903)); //
imagePoints.push_back(cv::Point2f(756, 551)); //
imagePoints.push_back(cv::Point2f(796, 833)); //
imagePoints.push_back(cv::Point2f(637, 800)); // (80, 80)
imagePoints.push_back(cv::Point2f(870, 670)); // (80, 80)
imagePoints.push_back(cv::Point2f(594, 592)); // (80, 80)
imagePoints.push_back(cv::Point2f(756, 591)); // (80, 80)
imagePoints.push_back(cv::Point2f(596, 887)); // (80, 80)
imagePoints.push_back(cv::Point2f(907, 436)); // (80, 80)
imagePoints.push_back(cv::Point2f(421, 943)); // (80, 80) */

//kop koordinater image
imagePoints.push_back(cv::Point2f(940, 632)); //
imagePoints.push_back(cv::Point2f(862, 633)); //
imagePoints.push_back(cv::Point2f(780, 634)); //
imagePoints.push_back(cv::Point2f(942, 762)); //
imagePoints.push_back(cv::Point2f(863, 719)); //
imagePoints.push_back(cv::Point2f(781, 809)); //
imagePoints.push_back(cv::Point2f(699, 814)); //
imagePoints.push_back(cv::Point2f(607, 910)); //
imagePoints.push_back(cv::Point2f(564, 822)); //
imagePoints.push_back(cv::Point2f(655, 908)); //
imagePoints.push_back(cv::Point2f(472, 780)); //
imagePoints.push_back(cv::Point2f (421, 971)); //
imagePoints.push_back(cv::Point2f(565, 503)); //
imagePoints.push_back(cv::Point2f(516, 409)); //
imagePoints.push_back(cv::Point2f(422, 501)); //
imagePoints.push_back(cv::Point2f(782, 419)); //
imagePoints.push_back(cv::Point2f(743, 636)); //
imagePoints.push_back(cv::Point2f(698, 504)); //



//Real world transformation of coordiantes


// Real world points
std::vector<cv::Point2f> realWorldPoints;
/*
    realWorldPoints.push_back(cv::Point2f(0.0f, 0.0f)); //(y,x)
    realWorldPoints.push_back(cv::Point2f(62.5f, 0.0f));
    realWorldPoints.push_back(cv::Point2f(0.0f, 62.5f));
    realWorldPoints.push_back(cv::Point2f(62.5f, 62.5f));

*/ /*
realWorldPoints.push_back(cv::Point2f(0.0, 0.0));   // (y, x)
realWorldPoints.push_back(cv::Point2f(45.0, 20.0));  // (y, x)
realWorldPoints.push_back(cv::Point2f(10.0, 15.0));  // (y, x)
realWorldPoints.push_back(cv::Point2f(15.0, 35.0)); // (y, x)
realWorldPoints.push_back(cv::Point2f(30.0, 5.0)); // (y, x)
realWorldPoints.push_back(cv::Point2f(40.0, 40.0)); // (y, x)
realWorldPoints.push_back(cv::Point2f(40.0, 20.0)); // (y, x)
realWorldPoints.push_back(cv::Point2f(5.0, 40.0)); // (y, x)
realWorldPoints.push_back(cv::Point2f(60.0, 0.0)); // (y, x)
realWorldPoints.push_back(cv::Point2f(0.0, 60.0)); // (y, x) */

//real world kop koordinater
realWorldPoints.push_back(cv::Point2f(35 ,0));   // (x, y)
realWorldPoints.push_back(cv::Point2f(35, 10));  //
realWorldPoints.push_back(cv::Point2f(35, 20));  //
realWorldPoints.push_back(cv::Point2f(20, 0.0)); //
realWorldPoints.push_back(cv::Point2f(25, 10.0)); //
realWorldPoints.push_back(cv::Point2f(15, 20.0)); //
realWorldPoints.push_back(cv::Point2f(15, 30.0)); //
realWorldPoints.push_back(cv::Point2f(5, 40.0)); //
realWorldPoints.push_back(cv::Point2f(15, 40.0));   //
realWorldPoints.push_back(cv::Point2f(5, 35.0));  //
realWorldPoints.push_back(cv::Point2f(20, 55.0));  //
realWorldPoints.push_back(cv::Point2f(0, 60.0)); //
realWorldPoints.push_back(cv::Point2f(50, 45.0)); //
realWorldPoints.push_back(cv::Point2f(60, 50.0)); //
realWorldPoints.push_back(cv::Point2f(50, 60.0)); //
realWorldPoints.push_back(cv::Point2f(60, 20.0)); //
realWorldPoints.push_back(cv::Point2f(35, 25.0)); //
realWorldPoints.push_back(cv::Point2f(50, 30.0)); //



// Compute the homography matrix

//cv::Mat H = cv::getPerspectiveTransform(imagePoints, realWorldPoints); //givet funktion med 4 koordinater
cv::Mat X = cv::findHomography(imagePoints, realWorldPoints); //fundet funktion med x koordinater


//std::cout << "Homography matrix H: " << H << std::endl;
std::cout << "Homography matrix X: " << X << std::endl;


cv::Mat img = cv::imread("../../../Billede/1.12.png", cv::IMREAD_GRAYSCALE);

cv::Mat imgUndistorted;
// Apply the undistortion map to correct lens distortion
cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);



// Display the undistorted image
cv::imshow("undistorted image", imgUndistorted);
while (1){
cv::setMouseCallback("undistorted image", onMouse, nullptr);
// Afslut hvis brugeren trykker på 'Esc'-tasten
if( cv::waitKey(10) == 27 )
{
    break;
}
}


 /*//hardcoded real world transformation of coordiantes
  cv::Matx33f H = cv::Matx33f    (-0.008453171254899592, -0.1027136663102614, 100.4104391989221,
 -0.1108970334507943, 1.095836764117068e-05, 100.503555926516,
 -0.0002133131348338028, -1.340640846687117e-05, 1);

ved GetPerspectiveTransform
cv::Matx33f H = cv::Matx33f (0, -0.1576812097236241, 142.3861323804325,
 -0.1570591931172192, -2.366511529057011e-17, 142.2956289642006,
 -7.759932481097805e-06, -3.872237587097853e-06, 1);
*/

cv::Matx33f H = cv::Matx33f(-0.007791881465124858, -0.09948645122707399, 99.89002391428173,
                            -0.1075931183674724, -0.0002392212860721122, 101.6042958591801,
                            -0.0001785793076406061, 2.295218118932419e-05, 0.9999999999999999);


//nye data


//indsæt de koordinater vi får fra billedgenkendelse i billed planet her? ------------------
// Now, we can use the homography matrix to transform a point
// Let's take the pixel coordinates (443, 971) (0,0)
cv::Point2f pixelPoint(833, 515);

// Convert the point to homogeneous coordinates (add 1 to the point)
std::vector<cv::Point2f> pixelPoints;
pixelPoints.push_back(pixelPoint);

std::vector<cv::Point2f> realWorldTransformedPoints;

// Apply the homography to the point
cv::perspectiveTransform(pixelPoints, realWorldTransformedPoints, X);



// The resulting real world coordinates
std::cout << "The transformed real world coordinates are: ("
          << realWorldTransformedPoints[0].x << ", "
          << realWorldTransformedPoints[0].y << ")" << std::endl;



return 0;
}
