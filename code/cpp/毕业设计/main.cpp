#include "headers.h"

int main()
{
	
	//string chessPath = "F:\\Documents\\Graduation Project\\Matching\\Picture DataSet\\chessPicture\\";
	string objectPath = "F:\\Documents\\Graduation Project\\Picture_dataSet\\teddy\\";
	string pointPath = "F:\\Documents\\Graduation Project\\Matching\\Picture DataSet\\objectPicture\\";
	//chessCapture(0,1,chessPath);
	//cout<<"�궨ͼ�񱣴����"<<endl;
	//int imageNum = 20;
	//cameraCalibrate(chessPath,Size(7,9),imageNum,20.65);//����궨

	Mat imgL = imread(objectPath+"imgL.png");
	Mat imgR = imread(objectPath+"imgR.png");
	/*
	Mat Q;
	Mat disp;
	Mat global_xyz;
	imageRect(imgL,imgR,Q,"calibrationResult.xml");
	*/
	Mat disp = stereoMatch(imgL,imgR);
	imshow("disp",disp);
	waitKey();
	/*
	//�Ӳ�ͼ��ɫ��ʾ
	Mat disp8U = Mat(disp.rows,disp.cols,CV_8UC1);
	normalize(disp,disp8U,0,255,NORM_MINMAX,CV_8UC1);
	imshow("�Ӳ�ͼ",disp8U);
	savePoint(pointPath+"point.txt",disp,Q);
	waitKey();
	*/
}