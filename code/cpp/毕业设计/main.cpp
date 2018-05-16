#include "headers.h"

Mat result;
Vec3f point;


int main()
{
	
	string chessPath = "F:\\Documents\\Graduation Project\\Dataset\\chessBoard-1\\";
	string objectPath = "F:\\Documents\\Graduation Project\\Dataset\\object-0\\";
	//chessCapture(0,1,chessPath);
	//cout<<"�궨ͼ�񱣴����"<<endl;
	int imageNum = 30;
	printf("Start calibration:  Y:yes,N:no");
	char keyBoard;//scanf(&keyBoard);
	//cameraCalibrate(chessPath,Size(7,9),imageNum,20.65);//����궨
	cout<<"��ʼУ��"<<endl;
	Mat imgL = imread("left_000010.jpg");
	Mat imgR = imread("right_000010.jpg");
	cout<<"У��ͼ�����ɹ�"<<endl;

	Mat Q;
	Mat disp;
	Mat global_xyz;
	imageRect(imgL,imgR,Q,"calib_result.xml");
	
	//disp = stereoGc();
	disp = stereoMatch(imgL,imgR);
	imwrite("disp.jpg",disp);

	//�Ӳ�ͼ��ɫ��ʾ
	Mat disp8U = Mat(disp.rows,disp.cols,CV_8UC1);
	normalize(disp,disp8U,0,255,NORM_MINMAX,CV_8UC1);
	
	cout<<"��ʼ��������"<<endl;
	result = savePoint("point.asc",disp,Q,imgL);
	imshow("�Ӳ�ͼ",disp8U);
	setMouseCallback("�Ӳ�ͼ",OnMouseAction);  
    waitKey();  
    system("pause");  
	/*
	typedef pcl::PointXYZ PointT;
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	cout<<"��ʼ��ȡ�ļ�"<<endl;
	pcl::io::loadPCDFile<PointT>("F:\\Documents\\Graduation Project\\Matching\\Graduation project\\Graduation project\\test_pcd.pcd",*cloud);
	cout<<"��ȡ�ļ��ɹ�"<<endl;
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
 
	//��������ӵ���
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
		
	}
	cout<<"���ݱ������"<<endl;
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