#include "headers.h"

int chessCapture(int leftcamNum,int rightcamNum,string savePath)
{
	VideoCapture   inputVideoL(leftcamNum),inputVideoR(rightcamNum);  //ѡ��������ͺ�
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
            imwrite(imgnameL,frameL);cout<<"��ͼ�������";
			imwrite(imgnameR,frameR);cout<<"��ͼ�������";
			f = f + 1;
        }
		
    }
    cout << "Finished writing" << endl;
    return 0;
}

int cameraCalibrate(string path,Size board_size,int imageNum,double squareLength)
{
	vector<string> imageL_path,imageR_path;
	for (int i = 1;i< imageNum + 1;i++)  //��ȡͼ��·��
	{
		imageL_path.push_back( path  + "left" + std::to_string(i) + ".jpg");
		imageR_path.push_back( path  + "right" + std::to_string(i) + ".jpg");
	}

	Size image_size;//ͼ��ߴ�
	vector<Point2f> image_points_buf[2];//����ÿ��ͼ���ϼ�⵽�Ľǵ�
	vector<vector<Point2f>> image_points_seq[2];//�����⵽�����нǵ�
	int count = 1;//���ڴ洢�ǵ����

	while (count <= imageNum)
	{
		cout<< "�������" << count << "��ͼƬ"<<endl;
		Mat imageInput_L = imread(imageL_path[count-1]);
		Mat imageInput_R = imread(imageR_path[count-1]);
		if (count == 1)//�����һ��ͼƬʱ��ȡͼ������Ϣ
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
			//�����ؾ�ȷ��
			find4QuadCornerSubpix(view_grayL,image_points_buf[0],board_size);//�Դ���ȡ�Ľǵ���о�ȷ��
			find4QuadCornerSubpix(view_grayR,image_points_buf[1],board_size);
			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq[0].push_back(image_points_buf[0]);//���������ؽǵ�
			image_points_seq[1].push_back(image_points_buf[1]);//���������ؽǵ�
		}
		count = count + 1;
	}

	//������궨��ʼ
	cout<<"��ʼ�궨------------"<<endl;
	//������ά��Ϣ
	Size square_size = Size(squareLength,squareLength);//ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С ��λmm
	//�������
	Mat cameraMatrix[2],distCoeffs[2];//�ڲξ���ͻ������
	vector<Mat> rvecsMat[2];//��ת����
	vector<Mat> tvecsMat[2];//ƽ������
	cameraMatrix[0] =  Mat(3,3,CV_32FC1,Scalar::all(0));//������ڲξ���
	cameraMatrix[1] =  Mat(3,3,CV_32FC1,Scalar::all(0));//������ڲξ���
	distCoeffs[0] = Mat(1,5,CV_32FC1,Scalar::all(0));//����ϵ��
	distCoeffs[1] = Mat(1,5,CV_32FC1,Scalar::all(0));//����ϵ��
	vector<int> point_counts;//ÿ��ͼ���нǵ������
	
	vector<vector<Point3f>> object_points;//����궨���Ͻǵ����ά����
	//��ʼ���궨���Ͻǵ����ά����
	int i,j,t;
	for(t=0;t<imageNum;t++)
	{
		vector<Point3f> tempPointSet;
		for(i=0;i<board_size.height;i++)
		{
			for(j=0;j<board_size.width;j++)
			{
				Point3f realPoint;
				//����궨�������������ϵ��z=0��ƽ����
				realPoint.x = i*square_size.width;
				realPoint.y = j*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}

		}
		object_points.push_back(tempPointSet);
	}
	cout<<"��ʼ���궨���Ͻǵ����ά�������-------"<<endl;
	//��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨��
	for(i=0;i<imageNum;i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}

	//��ʼ�궨
	cout<<"��ʼ���е�Ŀ�궨"<<endl;
	calibrateCamera(object_points,image_points_seq[0],image_size,cameraMatrix[0],distCoeffs[0],rvecsMat[0],tvecsMat[0],CV_CALIB_ZERO_TANGENT_DIST);
	calibrateCamera(object_points,image_points_seq[1],image_size,cameraMatrix[1],distCoeffs[1],rvecsMat[1],tvecsMat[1],CV_CALIB_ETALON_CHESSBOARD);
	
	
	//˫Ŀ�궨
	cout<<"��ʼ����˫Ŀ�궨"<<endl;
	Mat rotateMatrix,transMatrix,E,F;
	stereoCalibrate(object_points,
		image_points_seq[0],image_points_seq[1],
		cameraMatrix[0],distCoeffs[0],
		cameraMatrix[1],distCoeffs[1],
		image_size,rotateMatrix,transMatrix,E,F,
		TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,100,1e-5));
	cout<<"�궨����"<<endl;
	
	//����궨���
		FileStorage fs("calibrationResult.xml",FileStorage::WRITE);
		write(fs,"cameraMatrix-Left",cameraMatrix[0]);
		write(fs,"cameraMatrix-Right",cameraMatrix[1]);
		write(fs,"distCoeffs-Left",distCoeffs[0]);
		write(fs,"distCoeffs-Right",distCoeffs[1]);
		write(fs,"rotateMatrix",rotateMatrix);
		write(fs,"transMatrix",transMatrix);
		
	std::cout<<"��ɱ���"<<endl; 
	return 0;
}
