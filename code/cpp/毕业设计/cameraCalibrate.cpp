#include "headers.h"

int chessCapture(int leftcamNum,int rightcamNum,string savePath)
{
	VideoCapture   inputVideoL(leftcamNum),inputVideoR(rightcamNum);  //选定摄像机型号
    //inputVideo.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    //inputVideo.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

    if (!inputVideoL.isOpened())
    {
        cout << "Could not open the left video " << endl;
        return -1;
    }

	if (!inputVideoR.isOpened())
    {
        cout << "Could not open the right video " << endl;
        return -1;
    }
    Mat frameL;
	Mat frameR;
    string imgnameL,imgnameR;
    int f = 1;
    while (f <= MAX_CAPTURE_NUM) //Show the image captured in the window and repeat
    {
		inputVideoL >> frameL; 
		inputVideoR >> frameR;// read
		//if (frameL.empty() || frameR.empty()) break; // check if at end
		imshow("CameraL", frameL);
		imshow("CameraR",frameR);
        char key = waitKey(1);
		if (key == 'B'){break;}
        if (key == 'q' || key == 'Q')
        {
            imgnameL = savePath + "left" + to_string(f) + ".jpg";
			imgnameR = savePath + "right" + to_string(f) + ".jpg";
            imwrite(imgnameL,frameL);cout<<"左图保存完毕";
			imwrite(imgnameR,frameR);cout<<"右图保存完毕";
			f = f + 1;
        }
		
    }
    cout << "Finished writing" << endl;
    return 0;
}

int cameraCalibrate(string path,Size board_size,int imageNum,double squareLength)
{
	vector<string> imageL_path,imageR_path;
	for (int i = 1;i< imageNum + 1;i++)  //读取图像路径
	{
		imageL_path.push_back( path  + "left" + std::to_string(i) + ".jpg");
		imageR_path.push_back( path  + "right" + std::to_string(i) + ".jpg");
	}

	Size image_size;//图像尺寸
	vector<Point2f> image_points_buf[2];//缓存每幅图像上检测到的角点
	vector<vector<Point2f>> image_points_seq[2];//保存检测到的所有角点
	int count = 1;//用于存储角点个数

	while (count <= imageNum)
	{
		cout<< "正读入第" << count << "张图片"<<endl;
		Mat imageInput_L = imread(imageL_path[count-1]);
		Mat imageInput_R = imread(imageR_path[count-1]);
		if (count == 1)//读入第一张图片时获取图像宽高信息
		{	
			image_size.width = imageInput_L.cols;
			image_size.height = imageInput_L.rows;
		}

		if(0 == findChessboardCorners(imageInput_L,board_size,image_points_buf[0]))
		{
			cout << "can not find left chessboard corners!\n";
			exit(1);
		}
		else if (0 == findChessboardCorners(imageInput_R,board_size,image_points_buf[1]))
		{
			cout << "can not find right chessboard corners!\n";
			exit(1);
		}
		else
		{
			Mat view_grayL,view_grayR;
			cvtColor(imageInput_L,view_grayL,CV_RGB2GRAY);
			cvtColor(imageInput_R,view_grayR,CV_RGB2GRAY);
			//亚像素精确化
			find4QuadCornerSubpix(view_grayL,image_points_buf[0],board_size);//对粗提取的角点进行精确化
			find4QuadCornerSubpix(view_grayR,image_points_buf[1],board_size);
			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq[0].push_back(image_points_buf[0]);//保存亚像素角点
			image_points_seq[1].push_back(image_points_buf[1]);//保存亚像素角点
		}
		count = count + 1;
	}

	//摄像机标定开始
	cout<<"开始标定------------"<<endl;
	//棋盘三维信息
	Size square_size = Size(squareLength,squareLength);//实际测量得到的标定板上每个棋盘格的大小 单位mm
	//内外参数
	Mat cameraMatrix[2],distCoeffs[2];//内参矩阵和畸变参数
	vector<Mat> rvecsMat[2];//旋转向量
	vector<Mat> tvecsMat[2];//平移向量
	cameraMatrix[0] =  Mat(3,3,CV_32FC1,Scalar::all(0));//摄像机内参矩阵
	cameraMatrix[1] =  Mat(3,3,CV_32FC1,Scalar::all(0));//摄像机内参矩阵
	distCoeffs[0] = Mat(1,5,CV_32FC1,Scalar::all(0));//畸变系数
	distCoeffs[1] = Mat(1,5,CV_32FC1,Scalar::all(0));//畸变系数
	vector<int> point_counts;//每幅图像中角点的数量
	
	vector<vector<Point3f>> object_points;//保存标定板上角点的三维坐标
	//初始化标定板上角点的三维坐标
	int i,j,t;
	for(t=0;t<imageNum;t++)
	{
		vector<Point3f> tempPointSet;
		for(i=0;i<board_size.height;i++)
		{
			for(j=0;j<board_size.width;j++)
			{
				Point3f realPoint;
				//假设标定板放在世界坐标系中z=0的平面上
				realPoint.x = i*square_size.width;
				realPoint.y = j*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}

		}
		object_points.push_back(tempPointSet);
	}
	cout<<"初始化标定板上角点的三维坐标完毕-------"<<endl;
	//初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
	for(i=0;i<imageNum;i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}

	//开始标定
	cout<<"开始进行单目标定"<<endl;
	calibrateCamera(object_points,image_points_seq[0],image_size,cameraMatrix[0],distCoeffs[0],rvecsMat[0],tvecsMat[0],CV_CALIB_ZERO_TANGENT_DIST);
	calibrateCamera(object_points,image_points_seq[1],image_size,cameraMatrix[1],distCoeffs[1],rvecsMat[1],tvecsMat[1],CV_CALIB_ETALON_CHESSBOARD);
	
	
	//双目标定
	cout<<"开始进行双目标定"<<endl;
	Mat rotateMatrix,transMatrix,E,F;
	stereoCalibrate(object_points,
		image_points_seq[0],image_points_seq[1],
		cameraMatrix[0],distCoeffs[0],
		cameraMatrix[1],distCoeffs[1],
		image_size,rotateMatrix,transMatrix,E,F,
		TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,100,1e-5));
	cout<<"标定结束"<<endl;
	
	//保存标定结果
		FileStorage fs("calibrationResult.xml",FileStorage::WRITE);
		write(fs,"cameraMatrix-Left",cameraMatrix[0]);
		write(fs,"cameraMatrix-Right",cameraMatrix[1]);
		write(fs,"distCoeffs-Left",distCoeffs[0]);
		write(fs,"distCoeffs-Right",distCoeffs[1]);
		write(fs,"rotateMatrix",rotateMatrix);
		write(fs,"transMatrix",transMatrix);
		
	std::cout<<"完成保存"<<endl; 
	return 0;
}
