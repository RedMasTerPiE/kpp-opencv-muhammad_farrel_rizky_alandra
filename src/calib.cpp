#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{   
    VideoCapture vidCap(0);
    Mat cam; 
    Mat camMod;
    Mat camDrawn;
    TermCriteria termCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 32, 0.001);
    int flags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK;

    Size innerCorners(8, 6);
    double sizePerSquare = 30.0; //length in mm

    if(!vidCap.isOpened())
    { 
      cout << "Cam failed to open";
      return 1;
    }

    vector<Point3f> board3DCoords; //3D coords of board inner corners

    for(int y = 0; y < innerCorners.height; y++)
    {
      for(int x = 0; x < innerCorners.width; x++)
      {
        board3DCoords.push_back(Point3f(x * sizePerSquare, y * sizePerSquare, 0.0f));
      }
    }

    vector<vector<Point3f>> objectPoints; //temporary storage for 3D coords
    vector<vector<Point2f>> imagePoints; //temporary storage for image points

    while(vidCap.isOpened())
    {
        vidCap.read(cam);
        if(cam.empty())
        {   
            break;
        }
        
        cvtColor(cam, camMod, COLOR_BGR2GRAY);
        resize(camMod, camMod, Size(640, 480));

        vector<Point2f> boardCorners;

        bool found = findChessboardCorners(camMod, innerCorners, boardCorners, flags);

        resize(cam, camDrawn, Size(640, 480));
        drawChessboardCorners(camDrawn, innerCorners, boardCorners, found);
        
        imshow("Original Camera", cam);
        imshow("Camera Resized", camMod);
        imshow("Chessboard Projection", camDrawn);

        int key = waitKey(1);

        if(key == 32 && found) //key = space
        { 
          cornerSubPix(camMod, boardCorners, Size(11, 11), Size(-1, -1), termCriteria); //counts pixels in decimal precision by cutting every pixel in pieces
          
          objectPoints.push_back(board3DCoords);
          imagePoints.push_back(boardCorners);

          cout << "Snapshots: " << imagePoints.size() << endl;
        }
        if(key == 'c' && imagePoints.size() >= 32) //key = c
        { 
          Mat valueOfK, distCoeffs;
          vector<Mat> tVecs, rVecs;

          double rms = calibrateCamera(objectPoints, imagePoints, camMod.size(), valueOfK, distCoeffs, rVecs, tVecs);

          cout << "RMS Error = " << rms << endl;

          FileStorage fs("../yaml/kalibrasi.yaml", FileStorage::WRITE);
          fs << "img_width" << camMod.cols;
          fs << "img_height" << camMod.rows;
          fs << "cam_matrix" << valueOfK;
          fs << "dist_coeff" << distCoeffs;
          fs << "rms_error" << rms;
          fs.release();

          cout << "Saved camCalib.yaml" << endl;
        }
        if(key == 27) //key = esc
        {   
            break;
        }
    }
    vidCap.release();
    destroyAllWindows();

    return 0;
}