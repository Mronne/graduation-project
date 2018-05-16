#include "headers.h"

void imageRect(Mat& imgL,Mat& imgR,Mat& Q,string resultPath)
{
	if (!imgL.data || !imgR.data)//����Ƿ��ȡ�ɹ�
	{printf("��ȡͼƬ����");	}
	
	//�Ƚ�������У��
	Rect roi1,roi2;
	Mat rotateMatrix,transMatrix,R1,P1,R2,P2;
	Mat cameraMatrix[2],distCoeffs[2];
	FileStorage fs("calib_result.xml",FileStorage::READ);
	read(fs["cameraMatrix-Left"],cameraMatrix[0]);
	read(fs["cameraMatrix-Right"],cameraMatrix[1]);
	read(fs["distCoeffs-Left"],distCoeffs[0]);
	read(fs["distCoeffs-Right"],distCoeffs[1]);
	read(fs["rotateMatrix"],rotateMatrix);
	read(fs["transMatrix"],transMatrix);

	//alphaȡֵΪ-1ʱ��opencv�Զ��������ź�ƽ��
	cv::stereoRectify(cameraMatrix[0],distCoeffs[0],
		cameraMatrix[1],distCoeffs[1],
		imgL.size(),rotateMatrix,transMatrix,
		R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,0,imgL.size(),&roi1,&roi2);

	//��ȡ���������У��ӳ��
	Mat map11,map12,map21,map22;
	initUndistortRectifyMap(cameraMatrix[0],distCoeffs[0],R1,P1,imgL.size(),CV_32FC1,map11,map12);
	initUndistortRectifyMap(cameraMatrix[1],distCoeffs[1],R2,P2,imgL.size(),CV_32FC1,map21,map22);
	//У��ԭʼͼ��
	Mat imgL_rect,imgR_rect;
	remap(imgL,imgL_rect,map11,map12,INTER_LINEAR);
	remap(imgR,imgR_rect,map21,map22,INTER_LINEAR);
	imgL = imgL_rect;
	imgR = imgR_rect;
	
	//��ʾУ�����ͼ��
	Mat img(imgL.rows, imgR.cols*2, imgL.type());//�޴���IMG���߶�һ�������˫��
	Rect rect1 = Rect(0, 0, imgL.cols, imgL.rows);
	Rect rect2 = Rect(imgR.cols, 0, imgR.cols, imgR.rows);
	
	imgL.copyTo(img(rect1));
	imgR.copyTo(img(rect2));
	for( int i = 0; i < imgL.rows; i += 16 ) //������

        line(img, Point(0, i), Point(img.cols, i), Scalar(0, 255, 0), 1, 8);

    imshow("rectified", img);
	
	imwrite("У������ͼ.bmp",imgL);
	imwrite("У������ͼ.bmp",imgR);
	waitKey();
}

//��������ƥ��
Mat stereoMatch(Mat imgL,Mat imgR)
{
	Mat disp;

	int mindisparity = 0;
	int ndisparities = 64;
	int SADWindowSize = 4;

	//SGBM�㷨ʵ��
	//SGBM�㷨��ʼ��
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
	return disp;
}

//�洢��������
Mat savePoint(const string filename,const Mat disp,const Mat Q,Mat texture)
{
	//������ά����
	Mat global_xyz;
	reprojectImageTo3D(disp,global_xyz,Q,true,-1);
	const double max_z = 1.0e4;
	/*
	//����һ��������
	typedef pcl::PointXYZ PointT;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

	int width = global_xyz.cols;
	int height = global_xyz.rows;

	cloud->width = width*height;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(width * height);
	cout<<"���ƵĴ�СΪ"<<width*height<<endl;
	int i = 0;
	for(int y = 0;y < height;y++)
		{
			for(int x = 0;x < width;x++)
				{
					Vec3f point = global_xyz.at<Vec3f>(y,x);
					cloud->at(i).x = point[0];
					cloud->at(i).y = point[1];
					cloud->at(i).z = point[2];
					i = i+1;
					cout<<"��"<<i<<"�����ݱ������"<<endl;
				}	
		}
	 pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	
	 //��ʾ��
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
 
	//��������ӵ���
	//viewer.showCloud(cloud);

	*/

	ofstream fp(filename);
	ofstream fp1("point_rgb.asc");
	if(!fp.is_open())
	{
		std::cout<<"�򿪵����ļ�ʧ��"<<endl;
		fp.close();

	}
	if(!fp1.is_open())
	{
		std::cout<<"�򿪵���rgb�ļ�ʧ��"<<endl;
		fp1.close();

	}
	//����д��
	for(int y = 0;y < global_xyz.rows;y++)
	{
		for(int x = 0;x < global_xyz.cols;x++)
		{
			Vec3f point = global_xyz.at<Vec3f>(y,x);
			if(fabs(point[2]-max_z) < FLT_EPSILON || fabs(point[2]) > max_z )
				continue;
			fp<<point[0]/1000<<" "<<point[1]/1000<<" "<<point[2]/1000<<endl;
			fp1 << point[0]/1000 << " "<<point[1]/1000<<" "<<point[2]/1000<<" "<<(int)(texture.at<uchar>(y,x))<<endl;
		}
	}
	fp.close();
	fp1.close();
	return global_xyz;
}