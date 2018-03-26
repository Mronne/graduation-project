#include "headers.h"
/*
static void draw_subdiv_point(Mat& img,Point2f fp,Scalar color)//绘制点
{
	// draws the circle outline or a solid circle in the image
	circle(img,fp,3,color,CV_FILLED,8,0);
}
*/
static void draw_subdiv( Mat& img,cv::Subdiv2D& subdiv,Scalar delaunay_color)//绘制剖分
{
#if 1 
	vector<Vec6f> triangleList;//用来存储三角形的顶点坐标
	subdiv.getTriangleList(triangleList);//从三角剖分中计算三角形
	vector<Point> pt(3);//存储三角形的顶点

	for (unsigned int i = 0; i < triangleList.size();i++)
	{
		Vec6f t = triangleList[i];
		pt[0] = Point(cvRound(t[0]),cvRound(t[1]));
		pt[1] = Point(cvRound(t[2]),cvRound(t[3]));
		pt[2] = Point(cvRound(t[4]),cvRound(t[5]));
		line(img,pt[0],pt[1],delaunay_color,1,CV_AA,0);
		line(img,pt[1],pt[2],delaunay_color,1,CV_AA,0);
		line(img,pt[2],pt[0],delaunay_color,1,CV_AA,0);
	}
#else
	//绘制边
	vector<Vec4f> edgeList;//用来存储边序号
	subdiv.getEdgeList(edgeList);//得到边序号存入edgeList中
	for(unsigned int i = 0;i < edgeList.size();i++)
	{
		Vec4f e = edgeList[i];
		Point pt0 = Point(cvRound(e[0]),cvRound(e[1]));
		Point pt1 = Point(cvRound(e[2]),cvRound(e[3]));
		line(img,pt0,pt1,delaunay_color,1,CV_AA,0);
	}
#endif
}

/*
static void locate_point(Mat& img,Subdiv2D& subdiv,Point2f fp,Scalar active_color)//定位点
{
	int e0 = 0,vertex = 0;

	subdiv.locate(fp,e0,vertex);

	if(e0 > 0)
	{
		int e = e0;
		do
		{
			Point2f org,dst;//起点和终点
			if(subdiv.edgeOrg(e,&org) > 0 && subdiv.edgeDst(e,&dst) > 0)
				line(img,org,dst,Scalar(0,255,0),3,CV_AA,0);

			e = subdiv.getEdge(e,Subdiv2D::NEXT_AROUND_LEFT);
		}
		while( e!= e0);
	}

	draw_subdiv_point(img,fp,active_color);
}

/*
static void paint_voronoi(Mat& img,Subdiv2D& subdiv)//绘制voronoi图
{
vector<vector<Point2f>> facets;//面序号
vector<Point2f> centers;//存取每个面的中心点坐标
subdiv.getVoronoiFacetList(vector<int>(),facets,centers);

vector<Point> ifacet;
vector<vector<Point>> ifacets[1];

for(unsigned int i = 0; i < facets.size();i++)
{
ifacet.resize(facets[i].size());
for(unsigned int j = 0; j < facets[i].size();j++)
ifacet[j] = facets[i][j];

Scalar color;
color[0] = 255;
color[1] = 0;
color[2] = 0;
fillConvexPoly(img,ifacet,color,8,0);

ifacets[0] = ifacet;
polylines(img,ifacets,true,Scalar(),1,CV_AA,0);
circle(img,centers[i],3,Scalar(),-1,CV_AA,0);
}
}
*/
void Delaunay(Mat& img)
{
	//Scalar active_facet_color(0,0,255);
	Scalar delaunay_color(255,255,255);
	Rect rect(0,0,img.cols,img.rows);
	
	Subdiv2D subdiv(rect);
	for(int i = 0;i< 600;i++)
	{
		Point2f fp((float)(rand()%(rect.width-10)+5),
			(float)(rand()%(rect.height-10)+5));

		//locate_point(img,subdiv,fp,active_facet_color);//确定点的位置
		//imshow(win,img);

		subdiv.insert(fp);//插入生成的点
	}
	draw_subdiv(img,subdiv,delaunay_color);
	//imshow(win,img);
}

