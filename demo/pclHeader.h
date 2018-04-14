#include<iostream>
#include<fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/io/ply_io.h> //PCL的Ply格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件
#include <pcl/filters/passthrough.h>//滤波头文件
#include <pcl/filters/voxel_grid.h>//体素滤波
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>//法向量估计
#include <pcl/surface/gp3.h>//多边形网格显示

using namespace std;

void asc2pcd(string pointPath);

