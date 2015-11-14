/**
  *     Controls:
  *         left mouse button   - draw 4 points (top/bottom polyhedron face)
  *         UP / DOWN keys      - set polyhedron depth (height)
  *         ENTER key           - next picture
  *
  *     Data:
  *         The folder 'data' must be placed in the same directory as the executable.
  *
  *		Author: Jan Bednarik (jan.bedanrik@hotmail.cz)
  *
  */

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/photo/photo.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>

using namespace cv;

const Size IMG_SIZE(1280, 720);

std::string calibration_images[] = {
    "data/DSC_0492.JPG",
    "data/DSC_0491.JPG",
    "data/DSC_0490.JPG",
    "data/DSC_0489.JPG",
    "data/DSC_0488.JPG",
    "data/DSC_0487.JPG",
    "data/DSC_0486.JPG",
    "data/DSC_0485.JPG",
    "data/DSC_0482.JPG"
};

//std::string calibration_images[] = {
//    "img/1.jpg",
//    "img/2.jpg",
//    "img/3.jpg",
//    "img/4.jpg",
//    "img/5.jpg"
//};

const int NUM_IMGS = 9;

const string winName = "image"; // window name
cv::Mat image;                  // image for drawing points and showing a result
std::vector<cv::Mat> blockImg;  // block screen vertices (2D - after projection)
int numPointsClicked = 0;       // number of points the user clicked so far
double Z = 0.0;                 // Z coordinate of the top/bottom face
Mat currentImage;               // current reconstruction view

Point2f m2p(Mat m) {
    return Point2f(m.at<double>(0, 0), m.at<double>(1, 0));
}

Point2i m2pi(Mat m) {
    return Point2i((int)m.at<double>(0, 0), (int)m.at<double>(1, 0));
}

void onMouse(int event, int x, int y, int, void*)
{
    if(event == EVENT_LBUTTONDOWN) {
        if(numPointsClicked == 4) {
            numPointsClicked = 0;
            blockImg.clear();
            Z = 0.0;
            currentImage.copyTo(image);
        }

        // save point
        Mat p = (Mat_<double>(3, 1) << x, y, 1.0);
        blockImg[numPointsClicked] = p;

        // draw point
        circle(image, Point(x, y), 4, Scalar(0, 255, 0), 2);
        imshow(winName, image);

        if(numPointsClicked == 3) {
            line(image, m2p(blockImg[0]), m2p(blockImg[1]), Scalar(255, 0, 0), 2);
            line(image, m2p(blockImg[1]), m2p(blockImg[2]), Scalar(255, 0, 0), 2);
            line(image, m2p(blockImg[2]), m2p(blockImg[3]), Scalar(255, 0, 0), 2);
            line(image, m2p(blockImg[3]), m2p(blockImg[0]), Scalar(255, 0, 0), 2);
        }

        numPointsClicked += 1;
    }
}

int main(int argc, char *argv[])
{
    int numBoards = 1;
    int numCornersHor = 9;
    int numCornersVer = 6;


    int numSquares = numCornersHor * numCornersVer;
    cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);

    std::vector<std::vector<cv::Point3f> > object_points;
    std::vector<std::vector<cv::Point2f> > image_points;


    std::vector<cv::Point2f> corners;
    int successes = 0;

    cv::Mat gray_image;

    std::vector<cv::Point3f> obj;
    for(int j = 0; j < numSquares; j++)
        obj.push_back(cv::Point3f(j / numCornersHor, j % numCornersHor, 0.0f));


    for (int i = 0; i < NUM_IMGS; ++i) {
        // Read data.
        image = cv::imread(calibration_images[i]);

        cv::resize(image, image, IMG_SIZE);
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);

        bool found = cv::findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found) {
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(gray_image, board_sz, corners, found);

            image_points.push_back(corners);
            object_points.push_back(obj);
        }
        else {
            std::cout << "Not found in " << i << std::endl;
            exit(-1);
        }       
    }

    // Calibrate.
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;

    cv::calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

    // Let a user to draw the 4 base points of the block and create a block for each image
    namedWindow("image", WINDOW_AUTOSIZE);
    setMouseCallback("image", onMouse);
    for(int i = 0; i < NUM_IMGS; ++i) {
//        if(i != 1 && i != 3) continue;
        blockImg.clear();
        for(int j = 0; j < 8; ++j) blockImg.push_back(Mat(3, 1, CV_64FC1));
        numPointsClicked = 0;

        image = imread(calibration_images[i]);
        resize(image, image, IMG_SIZE);
        imshow("image", image);

        // wait for 'Enter' key
        int key;
        while((key = waitKey(0) & 255) != 10) { // move to next picture on 'Enter' key press
            if(numPointsClicked == 4) {
                // key 'UP'
                if(key == 82) {
                    Z += 0.1;
                // key 'DOWN'
                } else if(key == 84) {
                    Z -= 0.1;
                }

                // compute world -> camera image transformation
                Mat rot;
                Rodrigues(rvecs[i], rot);

                Mat rotTran = Mat::zeros(3, 4, CV_64FC1);
                rot.copyTo(rotTran(Range::all(), Range(0, 3)));
                tvecs[i].copyTo(rotTran(Range::all(), Range(3, 4)));

                Mat tf;
                tf = intrinsic * rotTran;

                // compute homography (camera image -> world transformation)
                Mat rotTranH = Mat::zeros(3, 3, CV_64FC1);
                rotTran(Range::all(), Range(0, 2)).copyTo(rotTranH(Range::all(), Range(0, 2)));
                rotTran(Range::all(), Range(3, 4)).copyTo(rotTranH(Range::all(), Range(2, 3)));

                Mat H = intrinsic * rotTranH;
                H /= H.at<double>(2, 2);
                invert(H, H);

                // compute block object coordinates (3D - in scene)
                std::vector<cv::Mat> blockObj;

                // lower base 4 points
                for(int j = 0; j < 4; ++j) {
                    Mat pt = H * blockImg[j];
                    pt /= pt.at<double>(2, 0);

                    // force set Z value to 0.0 (for some reason it is computed to be 1.0)
                    pt.at<double>(2, 0) = 0.0;

                    Mat blockP = Mat::zeros(4, 1, CV_64FC1);
                    pt.copyTo(blockP(Range(0, 3), Range::all()));
                    blockP.at<double>(3, 0) = 1.0;

                    blockObj.push_back(blockP);
                }

                // upper base 4 points
                for(int j = 0; j < 4; ++j) {
                    Mat p;
                    blockObj[j].copyTo(p);
                    p.at<double>(2, 0) = Z;
                    blockObj.push_back(p);
                }

                // debug - print block object vertices
                std::cout << "block object vertices: " << std::endl;
                for(int i = 0; i < blockObj.size(); ++i) {
                    std::cout << blockObj[i] << std::endl;
                }

                // compute remaining block image vertices (for upper base) from world coordinates
                for(int j = 0; j < 4; ++j) {
                    Mat p = tf * blockObj[j + 4];
                    p /= p.at<double>(2, 0);
                    blockImg[j + 4] = p(Range(0, 3), Range::all());
                }

                // debug - print block screen vertices
                std::cout << "block image vertices: " << std::endl;
                for(int j = 0; j < blockImg.size(); ++j) {
                    std::cout << blockImg[j] << std::endl;
                }

                image.copyTo(currentImage);

                line(currentImage, m2p(blockImg[0]), m2p(blockImg[1]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[1]), m2p(blockImg[2]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[2]), m2p(blockImg[3]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[3]), m2p(blockImg[0]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[4]), m2p(blockImg[5]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[5]), m2p(blockImg[6]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[6]), m2p(blockImg[7]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[7]), m2p(blockImg[4]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[0]), m2p(blockImg[4]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[1]), m2p(blockImg[5]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[2]), m2p(blockImg[6]), Scalar(255, 0, 0), 2);
                line(currentImage, m2p(blockImg[3]), m2p(blockImg[7]), Scalar(255, 0, 0), 2);

                imshow(winName, currentImage);
            }
        }

        numPointsClicked = 0;
        blockImg.clear();
        Z = 0.0;
    }

    return 0;
}
