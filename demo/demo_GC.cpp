#include <opencv2/opencv.hpp>  
#include <opencv2\imgproc\imgproc.hpp>  
#include <opencv2\core\core.hpp>  
#include <opencv2\highgui\highgui.hpp>  
#include <opencv2\calib3d\calib3d.hpp>  
#include <opencv2\features2d\features2d.hpp>  
#include <opencv2\legacy\legacy.hpp>  
using namespace std;
using namespace cv;

int main()  
{  
    IplImage * img1 = cvLoadImage("left1.jpg",0);  
    IplImage * img2 = cvLoadImage("right1.jpg",0);  
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
    cvNormalize(gcdispright,gcvdisp,0,255,CV_MINMAX);  
    cvSaveImage("GC_right_disparity3.png",gcvdisp);  
  
  
    cvNamedWindow("GC_disparity",0);  
    cvShowImage("GC_disparity",gcvdisp);  
    cvWaitKey(0);  
    cvReleaseMat(&gcdispleft);  
    cvReleaseMat(&gcdispright);  
    cvReleaseMat(&gcvdisp);  
	return 0;
}  