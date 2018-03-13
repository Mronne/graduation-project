#include "headers.h"


Mat stereoMatch(Mat imgL,Mat imgR,string resultPath)
{
	if (!imgL.data || !imgR.data)//����Ƿ��ȡ�ɹ�
	{printf("��ȡͼƬ����");	}
	
	//�Ƚ�������У��
	Rect roi1,roi2;
	Mat Q;
	Mat rotateMatrix,transMatrix,R1,P1,R2,P2;
	Mat cameraMatrix[2],distCoeffs[2];
	FileStorage fs(resultPath,FileStorage::READ);
	read(fs["cameraMatrix-Left"],cameraMatrix[0]);
	read(fs["cameraMatrix-Right"],cameraMatrix[1]);
	read(fs["distCoeffs-Left"],distCoeffs[0]);
	read(fs["distCoeffs-Right"],distCoeffs[1]);
	read(fs["rotateMatrix"],rotateMatrix);
	read(fs["transMatrix"],transMatrix);

	//alphaȡֵΪ-1ʱ��opencv�Զ��������ź�ƽ��
	cv::stereoRectify(cameraMatrix[0],distCoeffs[0],
		cameraMatrix[1],cameraMatrix[1],
		imgL.size(),rotateMatrix,transMatrix,
		R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,-1,imgL.size(),&roi1,&roi2);

	//��ȡ���������У��ӳ��
	Mat map11,map12,map21,map22;
	initUndistortRectifyMap(cameraMatrix[0],distCoeffs[0],R1,P1,imgL.size(),CV_16SC2,map11,map12);
	initUndistortRectifyMap(cameraMatrix[1],distCoeffs[1],R2,P2,imgL.size(),CV_16SC2,map21,map22);

	//У��ԭʼͼ��
	Mat imgL_rect,imgR_rect;
	remap(imgL,imgL_rect,map11,map12,INTER_LINEAR);
	remap(imgR,imgR_rect,map21,map22,INTER_LINEAR);
	imgL = imgL_rect;
	imgR = imgR_rect;
	
	Mat disp;

	int mindisparity = 0;
	int ndisparities = 64;
	int SADWindowSize = 11;

	//SGBM�㷨ʵ��
	cv::StereoSGBM sgbm = cv::StereoSGBM(mindisparity, ndisparities, SADWindowSize);
	
	sgbm.P1 = 8 * imgL.channels() * SADWindowSize * SADWindowSize;
	sgbm.P2 = 32 * imgL.channels() * SADWindowSize * SADWindowSize;

	sgbm.preFilterCap = 15;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleRange = 2;
	sgbm.speckleWindowSize = 100;
	sgbm.disp12MaxDiff = 1;
	
	sgbm.operator()(imgL,imgR,disp);
	
	disp.convertTo(disp,CV_32F,1.0 / 16);//����16�����ʵ�Ӳ�ֵ
	
	Mat disp8U = Mat(disp.rows,disp.cols,CV_8UC1);
	normalize(disp,disp8U,0,255,NORM_MINMAX,CV_8UC1);
	return disp8U;
}