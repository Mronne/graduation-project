#include "headers.h"
/*
static void draw_subdiv_point(Mat& img,Point2f fp,Scalar color)//���Ƶ�
{
	// draws the circle outline or a solid circle in the image
	circle(img,fp,3,color,CV_FILLED,8,0);
}
*/
static void draw_subdiv( Mat& img,cv::Subdiv2D& subdiv,Scalar delaunay_color)//�����ʷ�
{
#if 1 
	vector<Vec6f> triangleList;//�����洢�����εĶ�������
	subdiv.getTriangleList(triangleList);//�������ʷ��м���������
	vector<Point> pt(3);//�洢�����εĶ���

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
	//���Ʊ�
	vector<Vec4f> edgeList;//�����洢�����
	subdiv.getEdgeList(edgeList);//�õ�����Ŵ���edgeList��
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
static void locate_point(Mat& img,Subdiv2D& subdiv,Point2f fp,Scalar active_color)//��λ��
{
	int e0 = 0,vertex = 0;

	subdiv.locate(fp,e0,vertex);

	if(e0 > 0)
	{
		int e = e0;
		do
		{
			Point2f org,dst;//�����յ�
			if(subdiv.edgeOrg(e,&org) > 0 && subdiv.edgeDst(e,&dst) > 0)
				line(img,org,dst,Scalar(0,255,0),3,CV_AA,0);

			e = subdiv.getEdge(e,Subdiv2D::NEXT_AROUND_LEFT);
		}
		while( e!= e0);
	}

	draw_subdiv_point(img,fp,active_color);
}

/*
static void paint_voronoi(Mat& img,Subdiv2D& subdiv)//����voronoiͼ
{
vector<vector<Point2f>> facets;//�����
vector<Point2f> centers;//��ȡÿ��������ĵ�����
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

		//locate_point(img,subdiv,fp,active_facet_color);//ȷ�����λ��
		//imshow(win,img);

		subdiv.insert(fp);//�������ɵĵ�
	}
	draw_subdiv(img,subdiv,delaunay_color);
	//imshow(win,img);
}

