#include <opencv2/opencv.hpp>  
#include <opencv2\imgproc\imgproc.hpp>  
#include <opencv2\core\core.hpp>  
#include <opencv2\highgui\highgui.hpp>  
#include <opencv2\calib3d\calib3d.hpp>  
#include <opencv2\features2d\features2d.hpp>  
#include <opencv2\legacy\legacy.hpp>  
using namespace std;
using namespace cv;

Mat stereoGc(const char *filename1,const char *filename2) 
{  
	IplImage * img1 = cvLoadImage(filename1,0);  
    IplImage * img2 = cvLoadImage(filename2,0);  
    CvStereoGCState* GCState=cvCreateStereoGCState(64,3);  
    assert(GCState);  
    cout<<"start matching using GC"<<endl;  
    CvMat* gcdispleft=cvCreateMat(img1->height,img1->width,CV_16S);  
    CvMat* gcdispright=cvCreateMat(img2->height,img2->width,CV_16S);  
    CvMat* gcvdisp=cvCreateMat(img1->height,img1->width,CV_8U);  
    int64 t=getTickCount();  
    cvFindStereoCorrespondenceGC(img1,img2,gcdispleft,gcdispright,GCState);  
    t=getTickCount()-t;  
    cout<<"Time elapsed:"<<t*1000/getTickFrequency()<<endl;  
    //cvNormalize(gcdispleft,gcvdisp,0,255,CV_MINMAX);  
    //cvSaveImage("GC_left_disparity.png",gcvdisp);  
    //cvNormalize(gcdispright,gcvdisp,0,255,CV_MINMAX);  
    CvMat* disparity_left = cvCreateMat(gcdispleft->height,gcdispleft->width, CV_8U );
	cvConvertScale( gcdispleft, disparity_left,-16);  
	cvSaveImage("GCdisparity3.png",disparity_left);  
  
  
    cvNamedWindow("GC_disparity",0);  
    cvShowImage("GC_disparity",gcvdisp);  
    cvWaitKey(0);  
    cvReleaseMat(&gcdispleft);  
    cvReleaseMat(&gcdispright);  
    cvReleaseMat(&gcvdisp);  
	Mat result = Mat::Mat(disparity_left, true);
	cvReleaseMat(&disparity_left); 
	return result;
}  