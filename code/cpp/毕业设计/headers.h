#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<opencv2\imgproc\imgproc.hpp>
#include<opencv2\calib3d\calib3d.hpp>
#include <iostream>
#include <fstream>
#include <string>
/*
#include <pcl/io/pcd_io.h>           //PCD读写类相关的头文件
#include <pcl/point_types.h>      //PCL中支持的点类型的头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
*/
using namespace cv;
using namespace std;

#define MAX_CAPTURE_NUM 20
#define SQUARE_WIDTH 20.65

void OnMouseAction(int event,int x,int y,int flags,void *ustc);


int chessCapture(int leftcamNum,int rightcamNum,string savePath);//获取标定图像
int cameraCalibrate(string path,Size board_size,int imageNum,double squareLength);//进行相机标定
void imageRect(Mat& imgL,Mat& imgR,Mat& Q,string resultPath);//图像校正
Mat stereoMatch(Mat imgL,Mat imgR);//立体匹配
Mat stereoGc();//图割算法
Mat savePoint(const string filename,const Mat disp,const Mat Q,Mat texture);//点云数据存储
void Delaunay(Mat& img);//Delaunay三角剖分
void display(Mat global_xyz,Mat img);