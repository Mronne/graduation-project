#include "pclHeader.h"


typedef struct tagPOINT_3D
{
	double x;  //mm world coordinate x
	double y;  //mm world coordinate y
	double z;  //mm world coordinate z
	double r;
}POINT_WORLD;

void asc2pcd(string pointPath)
{/////º”‘ÿtxt ˝æ›
	int number_Txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
	ifstream fp;
	fp.open(pointPath);
	float x,y,z;
	while(!fp.eof())
	{	fp>>x;
		fp>>y;
		fp>>z;
		TxtPoint.x = x;
		TxtPoint.y = y;
		TxtPoint.z = z;
		m_vTxtPoints.push_back(TxtPoint);
	}
	fp.close();
	number_Txt = m_vTxtPoints.size();
	pcl::PointCloud<pcl::PointXYZ> cloud;


	// Fill in the cloud data
	cloud.width = number_Txt;
	cloud.height = 1;	
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);


	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = m_vTxtPoints[i].x;
		cloud.points[i].y = m_vTxtPoints[i].y;
		cloud.points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII("points.pcd", cloud);
	fp.close();
}