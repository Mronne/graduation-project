#include <gl\glut.h>
#include "headers.h"
GLuint Create3DTexture(Mat &img,vector<Vec3i> &tri,vector<Point2f> pts2DTex,
					   vector<Point3f> &pts3D,Point3f center3D,Vec3f size3D)
{
	GLuint tex = glGenLists(1);
	int error = glGetError();
	if(error != GL_NO_ERROR)
		cout << "An OpenGL error has occured:" << gluErrorString(error) << endl;
	if (tex == 0) return 0;

	Mat texImg;
	cvtColor(img,texImg,CV_BGR2RGB);
	resize(img,texImg,Size(512,512));

	glNewList(tex,GL_COMPILE);
	
	vector<Vec3i>::iterator iterTri = tri.begin();
	Point2f pt2D[3];
	Point3f pt3D[3];

	glDisable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	for(;iterTri != tri.end();iterTri++)
	{
		Vec3i &vertices = *iterTri;
		int ptIdx;
		for(int i = 0; i < 3;i++)
		{	
			ptIdx = vertices[i];
			if(ptIdx == -1) break;
			pt2D[i].x = pts2DTex[ptIdx].x / img.cols;
			pt2D[i].y = pts2DTex[ptIdx].y / img.rows;
			pt3D[i] = (pts3D[ptIdx]-center3D) * (1.f / max(size3D[0],size3D[1]));
		}	

		if (ptIdx != -1)
		{
			Mat TexTri(texImg,pt2D,pt3D);
		}
	}
	glDisable(GL_TEXTURE_2D);

	glEndList();
	return tex;
}
