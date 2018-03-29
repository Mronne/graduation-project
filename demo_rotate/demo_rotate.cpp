#include "stdafx.h"
#include <stdlib.h>
#include <GL/glut.h>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

double eyex, eyey, eyez, atx, aty, atz;//相机原点位置
double movex,movey;//物体平移距离
double angle_x,angle_y,angle_z;//旋转角度
double scaleNum;//缩放比例
int cx = 0;int cy = 0;
int rx = 0;int ry = 0;
bool mouseLeftDown,mouseRightDown,mouseMiddleDown;

void init(void)
{
    glClearColor (0.0, 0.0, 0.0, 0.0); //背景黑色
}





void display(void)
{
    glClear (GL_COLOR_BUFFER_BIT);
    glColor3f (1.0, 1.0, 1.0); //画笔白色
    glPointSize(2.0);
    glLoadIdentity();  //加载单位矩阵
    
    gluLookAt(eyex,eyey,eyez,atx,aty,atz,0.0,1.0,0.0);
	//gluLookAt(0,0,5,0,0,0,0.0,1.0,0.0);
	glTranslatef(movex,movey,0);
	glRotatef(angle_x,0,1,0);
	glRotatef(angle_y,1,0,0);
	//glRotatef(angle_z,0,0,1);
	glScalef(scaleNum,scaleNum,scaleNum);
    //glutWireTeapot(2);
	glBegin(GL_POINTS);
	glVertex3f(0,0,0.2);
	glVertex3f(0,0.3,0);
	glVertex3f(0.3,0,0);
	glEnd();
    glutSwapBuffers();
}

void reshape (int w, int h)
{
    glViewport (0, 0, (GLsizei) w, (GLsizei) h);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(60.0, (GLfloat) w/(GLfloat) h, 1.0, 20.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt (eyex,eyey,eyez,atx,aty,atz, 0.0, 1.0, 0.0);
	
	//gluLookAt(0,0,5,0,0,0,0.0,1.0,0.0);
}

void Mouse(int button,int state,int x,int y)
{
	cx = x;
	cy = y;
	switch (button)
	{
		case GLUT_RIGHT_BUTTON://记录下点击时的初始位置
		{	
			if(state == GLUT_DOWN)
				mouseRightDown = true;
			else if(state == GLUT_UP)
				mouseRightDown = false;
		}
		case GLUT_LEFT_BUTTON:
		{
			if(state == GLUT_DOWN)
				mouseLeftDown = true;

			else if(state == GLUT_UP)
				mouseLeftDown = false;
		}
		case GLUT_MIDDLE_BUTTON:
		{
			if(state == GLUT_DOWN)
				mouseMiddleDown = true;
			else if(state == GLUT_UP)
				mouseMiddleDown = false;
		}
		default:
			break;
	}
}
void onMouseMove(int x,int y)
{
	if(mouseRightDown)
	{
		//计算拖动后的偏移量，然后进行xy叠加减
		movex += (x-cx) * 0.01;
		movey -= (y-cy) * 0.01;
	}
	else if(mouseLeftDown)	
	{
		angle_x += (x-cx)*0.5;
		angle_y += (y-cy)*0.5;
		if(angle_x>360)
			angle_x = angle_x - 360;
		if(angle_y>360)
			angle_y = angle_y - 360;
	}
	else if(mouseMiddleDown)
	{	scaleNum += -(y - cy) * 0.002f;}

		//保存好当前拖放后光标坐标点
		cx = x;
		cy = y;
	glutPostRedisplay();	
}

int main(int argc, char** argv)
{
   //参数初始化
	eyex = 0.0;eyey = 0.0;eyez = 5.0;
	atx = 0.0;aty = 0.0;atz = 0.0;
	movex = 0.0;movey = 0.0;
	angle_x = 0.0;angle_y = 0.0;angle_z = 0.0;
	scaleNum = 1;
	glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize (500, 500);
    glutInitWindowPosition (100, 100);
    glutCreateWindow (argv[0]);
    
    init ();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
	glutMouseFunc(Mouse);
	glutMotionFunc(onMouseMove);
    glutMainLoop();

    return 0;
}