/*
    NOT WORKING, do not use.
*/


#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
using namespace cv;
using namespace std;

int boardHeight = 6;
int boardWidth = 9;
Size cbSize = Size(boardHeight,boardWidth);

string filename = "default.yml";

bool doneYet = false;
vector<Point2f> totalBuf;
vector<Point2f> pointBuf;
vector<Point2f> imagePts;
//default image size
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

ros::Subscriber * cameraSubscriber;
int foundImages = 0;

CvMat* intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
CvMat* distortion_coeffs = cvCreateMat(4,1,CV_32FC1);


void printMat(CvMat* mat)
{
    printf("(%dx%d)\n",mat->cols,mat->rows);
    for(int i=0; i<mat->rows; i++)
    {
        if(i==0)
        {
            for(int j=0; j<mat->cols; j++)  printf("%10d",j+1);
        }

        printf("\n%4d: ",i+1);
        for(int j=0; j<mat->cols; j++)
        {

            printf("%10.2f",cvGet2D(mat,i,j).val[0]);
        }
    }
}


void colorCb(const sensor_msgs::ImageConstPtr& msg)
{
     cv_bridge::CvImagePtr cv_ptr;
     try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        bool found = findChessboardCorners( (*cv_ptr).image, cbSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        if(found){            
            cv::cvtColor((*cv_ptr).image, (*cv_ptr).image, CV_RGB2GRAY);
            cornerSubPix((*cv_ptr).image, pointBuf, Size(11, 11), Size(-1, -1),
                TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

            drawChessboardCorners((*cv_ptr).image, cbSize, Mat(pointBuf), found);
            totalBuf.insert( totalBuf.end(), pointBuf.begin(), pointBuf.end() );

            if(pointBuf.size() == (6*9))
            {
                std::stringstream ss;
                ss << "./images/"<< foundImages<< ".png";
                ROS_INFO("Found chessboard. Writing to ./images/ ");
                foundImages++;
                imwrite(ss.str().c_str(), (*cv_ptr).image );
                std::cout << "Got enough points" << std::endl;
                for(int k =0; k< 9*6; k++){
                    imagePts.push_back(Point2f(pointBuf[k].x,pointBuf[k].y));
                }
                // step = successes*board_total;
                // for( int i=step, j=0; j<board_total; ++i,++j ) {
                // CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
                // CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
                // CV_MAT_ELEM(*object_points,float,i,0) = (float) j/board_w;
                // CV_MAT_ELEM(*object_points,float,i,1) = (float) (j%board_w);
                // CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
            }
           //CV_MAT_ELEM(*point_counts, int,successes,0) = board_total;

        }

    } catch (std::exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow("OPENCV_WINDOW", cv_ptr->image);
    if(cv::waitKey(30) >= 0) 
    {

        intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
        cvSetZero(intrinsic_matrix);
        CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
        CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;
        CV_MAT_ELEM( *intrinsic_matrix, float, 2, 2 ) = 1.0f;
        printMat(intrinsic_matrix);
        std::cout << "HALLO" << std::endl;
        vector<int> point_counts;
        for(int k = 0; k < foundImages; k++)
        {
            point_counts.push_back(9*6);
        }
        CvSize dim;
        dim.width = cv_ptr->image.rows;
        dim.height = cv_ptr->image.cols;

        Mat pb = Mat(totalBuf);
        Mat ip = Mat(imagePts);
        CvMat a = pb;
        CvMat b = ip;
        CvMat c = Mat(point_counts);
        std::cout << totalBuf.size() << std::endl;
        std::cout << imagePts.size() << std::endl;
        cvCalibrateCamera2(&a,&b, &c, 
                           dim,
                           intrinsic_matrix, distortion_coeffs, NULL, NULL,CV_CALIB_USE_INTRINSIC_GUESS );
        cvSave("Intrinsics.xml",intrinsic_matrix);
        cvSave("Distortion.xml",distortion_coeffs);
        
        cv::destroyWindow("OPENCV_WINDOW");
        cameraSubscriber->shutdown();
        return;
    }
}



// Main -------------------------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    ros::init(argc,argv,"camera_calib");
    ros::NodeHandle nh;
    cameraSubscriber = new ros::Subscriber();
    cv::namedWindow("OPENCV_WINDOW");
    *cameraSubscriber = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_color",1, colorCb);
    ros::spin();
    return 0;
    //set up a FileStorage object to read camera params from file
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    // read camera matrix and distortion coefficients from file
    Mat intrinsics, distortion;
    fs["Camera_Matrix"] >> intrinsics;
    fs["Distortion_Coefficients"] >> distortion;
    // close the input file
    fs.release();




    //set up matrices for storage
    Mat webcamImage, gray, one;
    Mat rvec = Mat(Size(3,1), CV_64F);
    Mat tvec = Mat(Size(3,1), CV_64F);

    //setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
    vector<Point2d> imagePoints, imageFramePoints, imageOrigin;
    vector<Point3d> boardPoints, framePoints;


    //generate vectors for the points on the chessboard
    for (int i=0; i<boardWidth; i++)
    {
        for (int j=0; j<boardHeight; j++)
        {
            boardPoints.push_back( Point3d( double(i), double(j), 0.0) );
        }
    }
    //generate points in the reference frame
    framePoints.push_back( Point3d( 0.0, 0.0, 0.0 ) );
    framePoints.push_back( Point3d( 5.0, 0.0, 0.0 ) );
    framePoints.push_back( Point3d( 0.0, 5.0, 0.0 ) );
    framePoints.push_back( Point3d( 0.0, 0.0, 5.0 ) );


    //set up VideoCapture object to acquire the webcam feed from location 0 (default webcam location)
    VideoCapture capture;
    capture.open(0);
    //set the capture frame size
    capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);

    while(!doneYet)
    {
         //store image to matrix
         capture.read(webcamImage);

         //make a gray copy of the webcam image
         cvtColor(webcamImage,gray,COLOR_BGR2GRAY);


         //detect chessboard corners
         bool found = findChessboardCorners(gray, cbSize, imagePoints, CALIB_CB_FAST_CHECK);
         //drawChessboardCorners(webcamImage, cbSize, Mat(imagePoints), found);

         

         //find camera orientation if the chessboard corners have been found
         if ( found )
         {
             //find the camera extrinsic parameters
             solvePnP( Mat(boardPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false );

             //project the reference frame onto the image
             projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints );
             

             //DRAWING
             //draw the reference frame on the image
             //circle(webcamImage, (Point) imagePoints[0], 4 ,CV_RGB(255,0,0) );
             
             Point one, two, three;
             one.x=10; one.y=10;
             two.x = 60; two.y = 10;
             three.x = 10; three.y = 60;

             line(webcamImage, one, two, CV_RGB(255,0,0) );
             line(webcamImage, one, three, CV_RGB(0,255,0) );


             line(webcamImage, imageFramePoints[0], imageFramePoints[1], CV_RGB(255,0,0), 2 );
             line(webcamImage, imageFramePoints[0], imageFramePoints[2], CV_RGB(0,255,0), 2 );
             line(webcamImage, imageFramePoints[0], imageFramePoints[3], CV_RGB(0,0,255), 2 );
             

             
             //show the pose estimation data
             cout << fixed << setprecision(2) << "rvec = ["
                  << rvec.at<double>(0,0) << ", "
                  << rvec.at<double>(1,0) << ", "
                  << rvec.at<double>(2,0) << "] \t" << "tvec = ["
                  << tvec.at<double>(0,0) << ", "
                  << tvec.at<double>(1,0) << ", "
                  << tvec.at<double>(2,0) << "]" << endl;
            
         }

         //show the image on screen
         namedWindow("OpenCV Webcam", 0);
         imshow("OpenCV Webcam", webcamImage);


         //show the gray image
         //namedWindow("Gray Image", CV_WINDOW_AUTOSIZE);
         //imshow("Gray Image", gray);


         waitKey(10);
    }

    return 0;
}
