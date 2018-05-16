#include "headers.h"

Mat result;
Vec3f point;


int main()
{
	
	string chessPath = "F:\\Documents\\Graduation Project\\Dataset\\chessBoard-1\\";
	string objectPath = "F:\\Documents\\Graduation Project\\Dataset\\object-0\\";
	//chessCapture(0,1,chessPath);
	//cout<<"标定图像保存完毕"<<endl;
	int imageNum = 30;
	printf("Start calibration:  Y:yes,N:no");
	char keyBoard;//scanf(&keyBoard);
	//cameraCalibrate(chessPath,Size(7,9),imageNum,20.65);//相机标定
	cout<<"开始校正"<<endl;
	Mat imgL = imread("left_000010.jpg");
	Mat imgR = imread("right_000010.jpg");
	cout<<"校正图像读入成功"<<endl;

	Mat Q;
	Mat disp;
	Mat global_xyz;
	imageRect(imgL,imgR,Q,"calib_result.xml");
	
	//disp = stereoGc();
	disp = stereoMatch(imgL,imgR);
	imwrite("disp.jpg",disp);

	//视差图彩色显示
	Mat disp8U = Mat(disp.rows,disp.cols,CV_8UC1);
	normalize(disp,disp8U,0,255,NORM_MINMAX,CV_8UC1);
	
	cout<<"开始保存数据"<<endl;
	result = savePoint("point.asc",disp,Q,imgL);
	imshow("视差图",disp8U);
	setMouseCallback("视差图",OnMouseAction);  
    waitKey();  
    system("pause");  
	/*
	typedef pcl::PointXYZ PointT;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	cout<<"开始读取文件"<<endl;
	pcl::io::loadPCDFile<PointT>("F:\\Documents\\Graduation Project\\Matching\\Graduation project\\Graduation project\\test_pcd.pcd",*cloud);
	cout<<"读取文件成功"<<endl;
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
 
	//往窗口添加点云
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		
	}
	cout<<"数据保存完毕"<<endl;
	waitKey();
	*/
	
}

void OnMouseAction(int event,int x,int y,int flags,void *ustc)
{
	if(event==CV_EVENT_LBUTTONDOWN)  
    {    
		cout<<"x"<<x<<"y"<<y<<endl;
		point = result.at<Vec3f>(y,x);
		cout<<"global: "<<"x="<<point[0]<<" y="<<point[1]<<" z="<<point[2]<<endl;
    }   
}