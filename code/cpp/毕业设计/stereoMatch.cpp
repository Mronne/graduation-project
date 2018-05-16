#include "headers.h"

void imageRect(Mat& imgL,Mat& imgR,Mat& Q,string resultPath)
{
	if (!imgL.data || !imgR.data)//检测是否读取成功
	{printf("读取图片错误");	}
	
	//先进行立体校正
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

	//alpha取值为-1时，opencv自动进行缩放和平移
	cv::stereoRectify(cameraMatrix[0],distCoeffs[0],
		cameraMatrix[1],distCoeffs[1],
		imgL.size(),rotateMatrix,transMatrix,
		R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,0,imgL.size(),&roi1,&roi2);

	//获取两个相机的校正映射
	Mat map11,map12,map21,map22;
	initUndistortRectifyMap(cameraMatrix[0],distCoeffs[0],R1,P1,imgL.size(),CV_32FC1,map11,map12);
	initUndistortRectifyMap(cameraMatrix[1],distCoeffs[1],R2,P2,imgL.size(),CV_32FC1,map21,map22);
	//校正原始图像
	Mat imgL_rect,imgR_rect;
	remap(imgL,imgL_rect,map11,map12,INTER_LINEAR);
	remap(imgR,imgR_rect,map21,map22,INTER_LINEAR);
	imgL = imgL_rect;
	imgR = imgR_rect;
	
	//显示校正后的图像
	Mat img(imgL.rows, imgR.cols*2, imgL.type());//⑥创建IMG，高度一样，宽度双倍
	Rect rect1 = Rect(0, 0, imgL.cols, imgL.rows);
	Rect rect2 = Rect(imgR.cols, 0, imgR.cols, imgR.rows);
	
	imgL.copyTo(img(rect1));
	imgR.copyTo(img(rect2));
	for( int i = 0; i < imgL.rows; i += 16 ) //画横线

        line(img, Point(0, i), Point(img.cols, i), Scalar(0, 255, 0), 1, 8);

    imshow("rectified", img);
	
	imwrite("校正后左图.bmp",imgL);
	imwrite("校正后右图.bmp",imgR);
	waitKey();
}

//进行立体匹配
Mat stereoMatch(Mat imgL,Mat imgR)
{
	Mat disp;

	int mindisparity = 0;
	int ndisparities = 64;
	int SADWindowSize = 4;

	//SGBM算法实现
	//SGBM算法初始化
	cv::StereoSGBM sgbm = cv::StereoSGBM(mindisparity, ndisparities, SADWindowSize);
	
	sgbm.P1 = 8 * imgL.channels() * SADWindowSize * SADWindowSize;
	sgbm.P2 = 32 * imgL.channels() * SADWindowSize * SADWindowSize;

	sgbm.preFilterCap = 15;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleRange = 2;
	sgbm.speckleWindowSize = 100;
	sgbm.disp12MaxDiff = 1;
	
	sgbm.operator()(imgL,imgR,disp);
	
	disp.convertTo(disp,CV_32F,1.0 / 16);//除以16求得真实视差值
	return disp;
}

//存储点云数据
Mat savePoint(const string filename,const Mat disp,const Mat Q,Mat texture)
{
	//计算三维坐标
	Mat global_xyz;
	reprojectImageTo3D(disp,global_xyz,Q,true,-1);
	const double max_z = 1.0e4;
	/*
	//定义一个点云类
	typedef pcl::PointXYZ PointT;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

	int width = global_xyz.cols;
	int height = global_xyz.rows;

	cloud->width = width*height;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize(width * height);
	cout<<"点云的大小为"<<width*height<<endl;
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
					cout<<"第"<<i<<"行数据保存完毕"<<endl;
				}	
		}
	 pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
	
	 //显示类
	//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
 
	//往窗口添加点云
	//viewer.showCloud(cloud);

	*/

	ofstream fp(filename);
	ofstream fp1("point_rgb.asc");
	if(!fp.is_open())
	{
		std::cout<<"打开点云文件失败"<<endl;
		fp.close();

	}
	if(!fp1.is_open())
	{
		std::cout<<"打开点云rgb文件失败"<<endl;
		fp1.close();

	}
	//遍历写入
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