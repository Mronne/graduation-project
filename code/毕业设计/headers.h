#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/legacy/legacy.hpp"
#include <iostream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

#define MAX_CAPTURE_NUM 20
#define SQUARE_WIDTH 14
#pragma comment(lib,"opencv_videostab249d.lib")
#pragma comment(lib,"opencv_video249d.lib")
#pragma comment(lib,"opencv_ts249d.lib")
#pragma comment(lib,"opencv_superres249d.lib")
#pragma comment(lib,"opencv_stitching249d.lib")
#pragma comment(lib,"opencv_photo249d.lib")
#pragma comment(lib,"opencv_ocl249d.lib")
#pragma comment(lib,"opencv_objdetect249d.lib")
#pragma comment(lib,"opencv_nonfree249d.lib")
#pragma comment(lib,"opencv_ml249d.lib")
#pragma comment(lib,"opencv_legacy249d.lib")
#pragma comment(lib,"opencv_imgproc249d.lib")
#pragma comment(lib,"opencv_highgui249d.lib")
#pragma comment(lib,"opencv_gpu249d.lib")
#pragma comment(lib,"opencv_flann249d.lib")
#pragma comment(lib,"opencv_features2d249d.lib")
#pragma comment(lib,"opencv_core249d.lib")
#pragma comment(lib,"opencv_contrib249d.lib")
#pragma comment(lib,"opencv_calib3d249d.lib")



int chessCapture(int leftcamNum,int rightcamNum,string savePath);//获取标定图像
int cameraCalibrate(string path,Size board_size,int imageNum,double squareLength);//进行相机标定
void imageRect(Mat& imgL,Mat& imgR,Mat& Q,string resultPath);//图像校正
Mat stereoMatch(Mat imgL,Mat imgR);//立体匹配
void savePoint(const string filename,const Mat disp,const Mat Q);//点云数据存储
void Delaunay(Mat& img);//Delaunay三角剖分
