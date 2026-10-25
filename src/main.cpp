#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct mainObjects
{   
    string name;
    Scalar lowerThresh;
    Scalar upperThresh;
    Scalar boxColor;
    bool invertThresh;
    int kernelOpen;
    int kernelClose;
    int iterOpen;
    int iterClose;
    double rtWidthCm;
};

int main() 
{   
    VideoCapture vidCap(0);
    Mat cam;
    Mat camGray;
    Mat camMod;
    Mat toHLS;
    Mat vidTuneMask;
    Mat camK;
    Mat distCoeffs;

    int hLow = 0, hHigh = 179, lLow = 0, lHigh = 255, sLow = 0, sHigh = 255, kOpen = 0, kClose = 0, iOpen = 1, iClose = 1, imgWidth, imgHeight;

    int previousTick = getTickCount();
    
    FileStorage fs("../yaml/kalibrasi.yaml", FileStorage::READ);
    fs["cam_matrix"] >> camK;
    fs["dist_coeff"] >> distCoeffs;
    fs["img_width"] >> imgWidth;
    fs["img_height"] >> imgHeight;
    fs.release();

    double fx = camK.at<double>(0, 0), fy = camK.at<double>(1, 1);
    double cx = camK.at<double>(0, 2), cy =  camK.at<double>(1, 2);

    vector<mainObjects> objects =
    {   
        {"Bag", Scalar(0, 29, 75), Scalar(37, 255, 255), Scalar(0, 255, 0), false, 1, 1, 1, 3, 20.0},
        {"Spray", Scalar(88, 58, 80), Scalar(109, 216, 255), Scalar(0, 255, 0), false, 1, 1, 1, 8, 4.0},
        {"Calculator", Scalar(101, 56, 9), Scalar(121, 107, 39), Scalar(0, 255, 0), false, 1, 1, 3, 9, 8.5}
    };

    namedWindow("tuning_trackbar");
    createTrackbar("hLow", "tuning_trackbar", &hLow, 179);
    createTrackbar("hHigh", "tuning_trackbar", &hHigh, 179);
    createTrackbar("lLow", "tuning_trackbar", &lLow, 255);
    createTrackbar("lHigh", "tuning_trackbar", &lHigh, 255);
    createTrackbar("sLow", "tuning_trackbar", &sLow, 255);
    createTrackbar("sHigh", "tuning_trackbar", &sHigh, 255);
    createTrackbar("kOpen", "tuning_trackbar", &kOpen, 10);
    createTrackbar("kClose", "tuning_trackbar", &kClose, 10);
    createTrackbar("iOpen", "tuning_trackbar", &iOpen, 10);
    createTrackbar("iClose", "tuning_trackbar", &iClose, 10);

    if(!vidCap.isOpened())
    {   
        cout << "Error: camera is unable to be opened";
        
        return 1;
    }

    while(true)
    {   
        vidCap.read(cam);
        if(cam.empty())
        {
            break;
        }

        cvtColor(cam, toHLS, COLOR_BGR2HLS);

        for(mainObjects &obj : objects)
        {   
            Mat vidMask;
            int best = -1;
            double bestArea = 0;
            inRange(toHLS, obj.lowerThresh, obj.upperThresh, vidMask);

            if(obj.invertThresh) 
            {   
                bitwise_not(vidMask, vidMask);
            }

            Mat sizeOpen = getStructuringElement(MORPH_ELLIPSE, Size(2*obj.kernelOpen + 1, 2*obj.kernelOpen + 1));
            Mat sizeClose = getStructuringElement(MORPH_ELLIPSE, Size(2*obj.kernelClose + 1, 2*obj.kernelClose + 1));

            erode(vidMask, vidMask, sizeOpen, Point(-1, -1), obj.iterOpen);
            dilate(vidMask, vidMask, sizeOpen, Point(-1, -1), obj.iterOpen);
            dilate(vidMask, vidMask, sizeClose, Point(-1, -1), obj.iterClose);
            erode(vidMask, vidMask, sizeClose, Point(-1, -1), obj.iterClose);

            vector<vector<Point>> objContouring;

            findContours(vidMask, objContouring, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            int i = 0;
            for(const vector<Point> &cont : objContouring) 
            {   
                double a = contourArea(cont);
                if(a > bestArea)
                {   
                    bestArea = a;
                    best = i;
                }
                i++;
            }
            if(best >= 0 && bestArea >= 5000) 
            {   
                Rect rectCont = boundingRect(objContouring[best]);
                rectangle(cam, rectCont, obj.boxColor, 2);

                Moments m = moments(objContouring[best]);
                double xPix = m.m10/m.m00, yPix = m.m01/m.m00;
                Point center((int)xPix, (int)yPix);

                double zCm = (fx*obj.rtWidthCm)/rectCont.width;
                double xCm = ((xPix - cx)*zCm)/fx;
                double yCm = ((yPix - cy)*zCm)/fy;

                ostringstream line_1, line_2;

                line_1 << obj.name << " " << fixed << setprecision(1) << zCm << " cm";
                line_2 << fixed << setprecision(1) << "X: " << xCm << " Y: " << yCm << " Z:" << zCm;

                circle(cam, center, 5, obj.boxColor, -1);
                putText(cam, obj.name, Point(rectCont.x, rectCont.y - 5), FONT_HERSHEY_SIMPLEX, 0.6, obj.boxColor, 2);
                putText(cam, line_2.str(), Point(rectCont.x, rectCont.y - 25), FONT_HERSHEY_SIMPLEX, 0.6, obj.boxColor, 2);
                putText(cam, line_1.str(), Point(rectCont.x, rectCont.y - 45), FONT_HERSHEY_SIMPLEX, 0.6, obj.boxColor, 2);
            }
        }

        inRange(toHLS, Scalar(hLow, lLow, sLow), Scalar(hHigh, lHigh, sHigh), vidTuneMask);

        Mat sOpen = getStructuringElement(MORPH_ELLIPSE, Size(2*kOpen + 1, 2*kOpen + 1));
        Mat sClose = getStructuringElement(MORPH_ELLIPSE, Size(2*kClose + 1, 2*kClose + 1));
        erode(vidTuneMask, vidTuneMask, sOpen, Point(-1, -1), iOpen);
        dilate(vidTuneMask, vidTuneMask, sOpen, Point(-1, -1), iOpen);
        dilate(vidTuneMask, vidTuneMask, sClose, Point(-1, -1), iClose);
        erode(vidTuneMask, vidTuneMask, sClose, Point(-1, -1), iClose);

        int realtimeTick = getTickCount();
        double fps = getTickFrequency()/(realtimeTick - previousTick);
        previousTick = realtimeTick;

        putText(cam, "FPS: " + to_string((int)fps), Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
        
        imshow("Original Camera", cam);
        imshow("HLS Camera", toHLS);
        imshow("Tuning Window", vidTuneMask);

        int key = waitKey(1);

        if(key == 27) 
        {
            break;
        }
    }

    vidCap.release();
    destroyAllWindows();
    
    return 0;
}