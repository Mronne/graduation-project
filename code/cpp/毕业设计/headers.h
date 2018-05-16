#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include<opencv2\imgproc\imgproc.hpp>
#include<opencv2\calib3d\calib3d.hpp>
#include <iostream>
#include <fstream>
#include <string>
/*
#include <pcl/io/pcd_io.h>           //PCD��д����ص�ͷ�ļ�
#include <pcl/point_types.h>      //PCL��֧�ֵĵ����͵�ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
*/
using namespace cv;
using namespace std;

#define MAX_CAPTURE_NUM 20
#define SQUARE_WIDTH 20.65

void OnMouseAction(int event,int x,int y,int flags,void *ustc);


int chessCapture(int leftcamNum,int rightcamNum,string savePath);//��ȡ�궨ͼ��
int cameraCalibrate(string path,Size board_size,int imageNum,double squareLength);//��������궨
void imageRect(Mat& imgL,Mat& imgR,Mat& Q,string resultPath);//ͼ��У��
Mat stereoMatch(Mat imgL,Mat imgR);//����ƥ��
Mat stereoGc();//ͼ���㷨
Mat savePoint(const string filename,const Mat disp,const Mat Q,Mat texture);//�������ݴ洢
void Delaunay(Mat& img);//Delaunay�����ʷ�
void display(Mat global_xyz,Mat img);