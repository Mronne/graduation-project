#include<iostream>
#include<fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
#include <pcl/io/ply_io.h> //PCL��Ply��ʽ�ļ����������ͷ�ļ�
#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
#include <pcl/visualization/cloud_viewer.h>//���Ʋ鿴����ͷ�ļ�
#include <pcl/filters/passthrough.h>//�˲�ͷ�ļ�
#include <pcl/filters/voxel_grid.h>//�����˲�
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>//����������
#include <pcl/surface/gp3.h>//�����������ʾ

using namespace std;

void asc2pcd(string pointPath);

