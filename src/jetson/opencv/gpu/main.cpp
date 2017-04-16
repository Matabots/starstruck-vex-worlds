#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;


Mat imgThresholded;

int main( int argc, char** argv )
{
  VideoCapture cap(0); //capture the video from web cam
  
  if ( !cap.isOpened() )  // if not success, exit program
  {
    cout << "Cannot open the web cam" << endl;
    return -1;
  }

  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  



  int iLowH = 0;
  int iHighH = 179;
  
  int iLowS = 0; 
  int iHighS = 255;
  
  int iLowV = 0;
  int iHighV = 255;
  
  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);
  
  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);
  
  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);


  // Setup SimpleBlobDetector parameters.
  SimpleBlobDetector::Params params;
  
  // Change thresholds
  params.minThreshold = 0;
  params.maxThreshold = 200;
  
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 100;
  params.maxArea = 9000;
  
  // Filter by Circularity
  params.filterByCircularity = false;
  params.minCircularity = 0.1;
  
  // Filter by Convexity
  params.filterByConvexity = true;
  params.minConvexity = 0.01;
  
  // Filter by Inertia
  params.filterByInertia = true;
  params.minInertiaRatio = 0.01;
  SimpleBlobDetector detector(params);
 
  std::vector<KeyPoint> keypoints;   

  while (true)
  {
    Mat imgOriginal;
    
    bool bSuccess = cap.read(imgOriginal); // read a new frame from video
    
    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read a frame from video stream" << endl;
      break;
    }
	 
    Mat imgHSV;
    
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    

    
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    
    //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    blur( imgThresholded, imgThresholded, Size(9,9) );
    bitwise_not(imgThresholded, imgThresholded);
    detector.detect( imgThresholded, keypoints);

    Mat im_with_keypoints;

    drawKeypoints( imgThresholded, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
 
    imshow("Thresholded Image", im_with_keypoints/*imgThresholded*/); //show the thresholded image
    imshow("Original", imgOriginal); //show the original image
    
    if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed by user" << endl;
      break; 
    }
  }
  
  return 0;

}
