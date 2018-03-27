#define SIGN(x) ( (x)<0 ? -1:((x)>0?1:0 ) ) 
#include <GL/glut.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <fstream>
using namespace std;

using namespace cv;

////////////////////////////////////////////////////////////////////
//--------------------OpenGLȫ�ֱ�������--------------------------//

Mat xyzdata = imread("F:\\Documents\\Graduation Project\\Matching\\Graduation project\\Graduation project\\ȫ������.jpg");
Mat texture = imread("F:\\Documents\\Graduation Project\\Picture_dataSet\\teddy\\imgL.png");
string pointPath = "F:\\Documents\\Graduation Project\\OpenGL\\textureMap\\tree1.asc";
const int width = xyzdata.cols;
const int height = texture.rows; 
//Mat texture(height,width,CV_32SC3);
//Mat xyzdata(height,width,CV_32SC3);
int glWinWidth = 640, glWinHeight = 480;  
  
double eyex, eyey, eyez, atx, aty, atz;  // eye* - �����λ�ã�at* - ע�ӵ�λ��  
bool leftClickHold = false, rightClickHold = false;  
int mx,my;          // ��갴��ʱ��OpenGL���ڵ�����  
int ry = 90, rx = 90;    // ��������ע�ӵ�Ĺ۲�Ƕ�  
double mindepth, maxdepth;      // ������ݵļ�ֵ   
double radius = 6000.0;     // �������ע�ӵ�ľ���  


/*******************************************************************/
/*                            OpenGL��Ӧ����                        */
/*******************************************************************/
//��갴����Ӧ����
void mouse(int button, int state, int x, int y);
//����˶�����
void motion(int x, int y);
//��άͼ����ʾ��Ӧ����
void renderScene(void);
//������ά����
vector<Point3f> load3dDataToGL(string path);
//����ͼ����������
void loadTextureToGL(Mat img);
//Ѱ������������ֵ
Vec3f maxPoint(vector<Point3f> point);



int main(int argc,char* argv[])
{
	glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowPosition(200,50);
    glutInitWindowSize(800,800);
    glutCreateWindow("Three Window");//ֱ��glutMainLoop����ʾ����
	glPointSize(3.0);
    glLineWidth(1.0);
	//load3dDataToGL(global_xyz);
	//loadTextureToGL(texImg);
	glFlush();
    glutSwapBuffers();
	glutDisplayFunc(&renderScene);
	glutMouseFunc(&mouse);// ��갴����Ӧ
	glutMotionFunc(&motion);// ����ƶ���Ӧ                           
    //glutSpecialFunc(&SpecialKeys);
	glutMainLoop();
    return 0;
}


void renderScene(void)
{
	 glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );  
    // Reset the coordinate system before modifying   
   /*
	 glLoadIdentity();
	// �������λ��  
    atx = 0.0f;  
    aty = 0.0f;  
    atz = ( mindepth - maxdepth ) / 2.0f;  
    eyex = atx + radius * sin( CV_PI * ry / 180.0f ) * cos( CV_PI * rx/ 180.0f );   
    eyey = aty + radius * cos( CV_PI * ry/ 180.0f );   
    eyez = atz + radius * sin( CV_PI * ry / 180.0f ) * sin( CV_PI * rx/ 180.0f );  
    gluLookAt (eyex, eyey, eyez, atx, aty, atz, 0.0, 1.0, 0.0);  
    glRotatef(0,0,1,0);  
    glRotatef(-180,1,0,0); 
	*/
	/*
	//�Ե����������ǻ�
	for(int i = 0;i < height;i++)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for(int j = 0;j < width;j++)
		{
			//�������������꣬Ȼ����ƶ���
			
			�������˳������
			0 ---> 1
				  /
				/
			  /
			2 ---> 3
			
			Vec3f temp_texture;
			Vec3f temp_xyz;
			//���ƶ���0
			glTexCoord2f(0.0f,0.0f);
			temp_texture = texture.at<Vec3f>(i,j);
			temp_xyz = xyzdata.at<Vec3f>(i,j);
			glColor3f(temp_texture[0]/255.0f,temp_texture[1]/255.0f,temp_texture[2]/255.0f);
			glVertex3f(temp_xyz[0],temp_xyz[1],temp_xyz[2]);
			//���ƶ���1
			glTexCoord2f(1.0f,0.0f);
			temp_texture = texture.at<Vec3f>(i+1,j);
			temp_xyz = xyzdata.at<Vec3f>(i+1,j);
			glColor3f(temp_texture[0]/255.0f,temp_texture[1]/255.0f,temp_texture[2]/255.0f);
			glVertex3f(temp_xyz[0],temp_xyz[1],temp_xyz[2]);
			//���ƶ���2
			glTexCoord2f(0.0f,1.0f);
			temp_texture = texture.at<Vec3f>(i,j+1);
			temp_xyz = xyzdata.at<Vec3f>(i,j+1);
			glColor3f(temp_texture[0]/255.0f,temp_texture[1]/255.0f,temp_texture[2]/255.0f);
			glVertex3f(temp_xyz[0],temp_xyz[1],temp_xyz[2]);
			//���ƶ���3
			glTexCoord2f(1.0f,1.0f);
			temp_texture = texture.at<Vec3f>(i+1,j+1);
			temp_xyz = xyzdata.at<Vec3f>(i+1,j+1);
			glColor3f(temp_texture[0]/255.0f,temp_texture[1]/255.0f,temp_texture[2]/255.0f);
			glVertex3f(temp_xyz[0],temp_xyz[1],temp_xyz[2]);
		}
		glEnd();
	}
	*/
	 /*
	//���û��ģʽ
	glEnable(GL_BLEND);
	//����ֻ����Ȼ���
	glDepthMask(GL_FALSE);
	//���û�Ϻ����Ըı�͸����
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);
	//����Ϊ��������Ȼ���ģʽ
	glDepthMask(GL_TRUE);
	//�رջ��ģʽ
	glDisable(GL_BLEND);
	*/
	float x,y,z; 
	vector<Point3f> pdata = load3dDataToGL(pointPath);
    // ����ͼ����� 
    glPointSize(1.0);  
    glBegin(GL_POINTS); 
    for (unsigned int i=0;i<pdata.size();i++)
	{  
		
            // color interpolation 
			//Vec3f temp_texture = texture.at<Vec3f>(i,j);
			//Vec3f temp_xyz = xyzdata.at<Vec3f>(i,j);
            glColor3f(255,255,255); 
            x= pdata[i].x; 
            y= pdata[i].y;  
            z= pdata[i].y;  
            glVertex3f(x,y,z);  
       
    } 
	//glColor3f(255,255,255);
	//glVertex3f(0.2,0.2,0.3);
	//glVertex3f(0.2,0.4,0.3);
    glEnd(); 
	glFlush();  
	glPopMatrix();
    glutSwapBuffers();  
}

/////////////////////////////////////////////////////////////////
//������ά��������  ��ת��Ϊ��׼����
vector<Point3f> load3dDataToGL(string path)
{
	vector<Point3f>  pointData;
	Point3f tempPoint;
	ifstream fp;
	fp.open(path);
	if(!fp.is_open())
	{
		std::cout<<"�򿪵����ļ�ʧ��"<<endl;
		fp.close();
	}
	float x,y,z;
	while(!fp.eof())
	{	fp>>x;
		fp>>y;
		fp>>z;
		tempPoint.x = x;
		tempPoint.y = y;
		tempPoint.z = z;
		pointData.push_back(tempPoint);
	}
	fp.close();
	Vec3f max_point = maxPoint(pointData);
	for (unsigned int i = 0;i<pointData.size();i++)
	{	pointData[i].x = pointData[i].x/max_point[0];
		pointData[i].y = pointData[i].y/max_point[1];
		pointData[i].z = pointData[i].z/max_point[2];
	}
	return pointData;
}
/*
/////////////////////////////////////////////////////////////////
//����ͼ����������
void loadTextureToGL(Mat img)
{
	Vec3f s;
	//accessing the image pixels  
    for (int i=0;i<height;i++)  
    {   
        for (int j=0;j<width;j++)  
        {  
            s=img.at<Vec3f>(i,j); 
			texture.at<Vec3f>(i,j) = s;
        }  
    }     
}  
*/
Vec3f maxPoint(vector<Point3f> point)
{
	float x_max,y_max,z_max;
	Vec3f max_point;
	unsigned int i = 0;
	x_max = 0;
	y_max = 0;
	z_max = 0;
	while(i<point.size())
	{
		if(abs(point[i].x)>x_max)
			x_max = abs(point[i].x);
		else if(abs(point[i].y)>y_max)
			y_max = abs(point[i].y);
		else if(abs(point[i].z)>z_max)
			z_max = abs(point[i].z);
		i = i+1;
	}
	max_point[0] = x_max;
	max_point[1] = y_max;
	max_point[2] = z_max;
	return max_point;
}

/*
void mouse(int button, int state, int x, int y)  
{  
    if(button == GLUT_LEFT_BUTTON)  
    {  
        if(state == GLUT_DOWN)  
        {  
            leftClickHold=true;  
        }  
        else  
        {  
            leftClickHold=false;  
        }  
    }  
    if (button== GLUT_RIGHT_BUTTON)  
    {  
        if(state == GLUT_DOWN)  
        {  
            rightClickHold=true;  
        }  
        else  
        {  
            rightClickHold=false;  
        }  
    }  
}

void motion(int x, int y)  
{  
    int rstep = 5;   
    if(leftClickHold==true)  
    {  
        if( abs(x-mx) > abs(y-my) )  
        {  
            rx += SIGN(x-mx)*rstep;      
        }  
        else  
        {  
            ry -= SIGN(y-my)*rstep;      
        }  
          
        mx=x;  
        my=y;  
        glutPostRedisplay();  
    }  
    if(rightClickHold==true)  
    {  
        if( y-my > 0 )  
        {  
            radius += 100.0;  
        }  
        else if( y-my < 0 )  
        {  
            radius -= 100.0;  
        }  
        radius = std::max( radius, 100.0 );  
        mx=x;  
        my=y;  
        glutPostRedisplay();  
    }  
}
*/

void mouse(int button, int state, int x, int y)  
{  
    if (state == GLUT_DOWN)  
    {  
        leftClickHold = GL_TRUE;  
    }  
    mx = x, my = y;  
}  
  
void motion(int x, int y)  
{  
    if (leftClickHold == GL_TRUE)  
    {       /// �����Ե������ǵ�����ת�ٶȵģ�������ã��ﵽ�Լ���Ҫ�ٶȼ���  
        xrotate -= (x - mousex) / 10.0f;  
        yrotate -= (y - mousey) / 10.0f;  
    }  
    mousex = x, mousey = y;  
    glutPostRedisplay();  
}  