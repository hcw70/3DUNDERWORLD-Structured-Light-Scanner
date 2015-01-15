//------------------------------------------------------------------------------------------------------------
//* Copyright © 2010-2013 Immersive and Creative Technologies Lab, Cyprus University of Technology           *
//* Link: http://ict.cut.ac.cy                                                                               *
//* Software developer(s): Kyriakos Herakleous                                                               *
//* Researcher(s): Kyriakos Herakleous, Charalambos Poullis                                                  *
//*                                                                                                          *
//* This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.*
//* Link: http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US                                        *
//------------------------------------------------------------------------------------------------------------

#include "StdAfx.h"
#include "Utilities.h"
#include <conio.h>

Utilities::Utilities(void)
{
}


Utilities::~Utilities(void)
{

}

bool Utilities::XOR(bool val1, bool val2)
{
	if(val1==val2)
		return 0;
	else
		return 1;
}

void Utilities::normalize3d(CvScalar &vec)	
{
	double mag = sqrt( vec.val[0]*vec.val[0] + vec.val[1]*vec.val[1] + vec.val[2]*vec.val[2]);
	
	vec.val[0] /= std::max(0.000001, mag);
	vec.val[1] /= std::max(0.000001, mag);
	vec.val[2] /= std::max(0.000001, mag);
	
	return;
}

void Utilities::normalize3dtable(double vec[3])
{
	double mag = sqrt( vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
	
	vec[0] /= std::max(0.000001, mag);
	vec[1] /= std::max(0.000001, mag);
	vec[2] /= std::max(0.000001, mag);
}

//convert image pixel to image 3d space point
void Utilities::pixelToImageSpace(double p[3], CvScalar fc, CvScalar cc)
{

	p[0]=(p[0]-cc.val[0])/fc.val[0];
	p[1]=(p[1]-cc.val[1])/fc.val[1];
	p[2]=1;

}

void Utilities::pixelToImageSpace(cv::Point3f &p, float ccX,float ccY, float fcX, float fcY)
{

	p.x=(p.x-ccX)/fcX;
	p.y=(p.y-ccY)/fcY;
	p.z=1;

}


void Utilities::undistortPoints( double xi, double yi, VirtualCamera *cam,double *xd,double *yd)
{

    double  k[5]={0,0,0,0,0}, fx, fy, ifx, ify, cx, cy;
  
    int iters = 1;

	k[0]=cam->distortion->data.fl[0];
	k[1]=cam->distortion->data.fl[1];
	k[2]=cam->distortion->data.fl[2];
	k[3]=cam->distortion->data.fl[3];
	k[4]=0;

    iters = 5;

	fx = cam->fc.val[0]; 
    fy = cam->fc.val[1]; 
	
    ifx = 1./fx;
    ify = 1./fy;
    cx = cam->cc.val[0]; 
    cy = cam->cc.val[1]; 


	double x, y, x0, y0;
	x=xi;
	y=yi;

	x0 = x = (x - cx)*ifx;
	y0 = y = (y - cy)*ify;
			
	for(int jj = 0; jj < iters; jj++ )
	{
		double r2 = x*x + y*y;
		double icdist = 1./(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
		double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
		double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
		x = (x0 - deltaX)*icdist;
		y = (y0 - deltaY)*icdist;
	}
	
	*xd=(x*fx)+cx;
	*yd=(y*fy)+cy;
}

//calculate the intersection point of a ray and a plane, given the normal and a point of the plane, and a point and the vector of the ray
CvScalar Utilities::planeRayInter(CvScalar planeNormal,CvScalar planePoint, CvScalar rayVector, CvScalar rayPoint )
{
	double l;
	CvScalar point;

	CvScalar pSub;

	pSub.val[0] = - rayPoint.val[0] + planePoint.val[0];
	pSub.val[1] = - rayPoint.val[1] + planePoint.val[1];
	pSub.val[2] = - rayPoint.val[2] + planePoint.val[2];

	double dotProd1 = pSub.val[0] * planeNormal.val[0] + pSub.val[1] * planeNormal.val[1] + pSub.val[2] * planeNormal.val[2];
	double dotProd2 = rayVector.val[0] * planeNormal.val[0] + rayVector.val[1] * planeNormal.val[1] + rayVector.val[2] * planeNormal.val[2];

	

	
	if(fabs(dotProd2)<0.00001)
	{
		std::cout<<"Error 10\n";
		//getch();
		point.val[0]=0;
		point.val[1]=0;
		point.val[2]=0;
		return point;
	}

	

	l = dotProd1 / dotProd2;

	point.val[0] = rayPoint.val[0] + l * rayVector.val[0]; 
	point.val[1] = rayPoint.val[1] + l * rayVector.val[1]; 
	point.val[2] = rayPoint.val[2] + l * rayVector.val[2]; 

	return point;
}

float Utilities::matGet2D(cv::Mat m, int x, int y)
{
	int type = m.type();

	switch(type)
	{
		case CV_8U:
			return m.at<uchar>(y,x);
			break;
		case CV_8S:
			return m.at<schar>(y,x);
			break;
		case CV_16U:
			return m.at<ushort>(y,x);
			break;
		case CV_16S:
			return m.at<short>(y,x);
			break;
		case CV_32S:
			return m.at<int>(y,x);
			break;
		case CV_32F:
			return m.at<float>(y,x);
			break;
		case CV_64F:
			return m.at<double>(y,x);
			break;
	}

}

void Utilities::matSet2D(cv::Mat m, int x, int y, double val)
{
	int type = m.type();

	switch(type)
	{
		case CV_8U:
			m.at<uchar>(y,x) = (uchar) val;
			break;
		case CV_8S:
			m.at<schar>(y,x) = (schar) val;
			break;
		case CV_16U:
			m.at<ushort>(y,x) = (ushort) val;
			break;
		case CV_16S:
			m.at<short>(y,x) = (short) val;
			break;
		case CV_32S:
			m.at<int>(y,x) = (int) val;
			break;
		case CV_32F:
			m.at<float>(y,x) = (float) val;
			break;
		case CV_64F:
			m.at<double>(y,x) = (double) val;
			break;
	}

}

bool direction(cv::Point p1, cv::Point p2,cv::Point p3)
{
	int p = -p2.x*p1.y + p3.x*p1.y + p1.x*p2.y - p3.x*p2.y - p1.x*p3.y + p2.x*p3.y; 

	if(p<0)
		return false;
	else
		return true;
}


void Utilities::autoContrast(cv::Mat img_in, cv::Mat &img_out)
{

	double min=0,max=0;

	std::vector<cv::Mat> bgr;
	cv::split(img_in,bgr);

	for(int i=0; i<3; i++)
	{
		cv::minMaxIdx(bgr[i],&min,&max);
		min += 255*0.05;
			
		double a = 255/(max-min);
		bgr[i]-=min;
		bgr[i]*=a;
	}
	
	cv::merge(bgr,img_out);
}

void Utilities::autoContrast(IplImage *img_in, IplImage *img_out)
{
	
	cv::Mat tmp_in = img_in;
	cv::Mat tmp_out = img_out;

	autoContrast(tmp_in,tmp_out);
	
}

void Utilities::exportMat(char *path, cv::Mat m)
{

	std:: ofstream out; 
	out.open(path);
	
	for(int i =0; i < m.rows; i++)
	{

		for(int j = 0; j < m.cols; j++)
		{

			out<< Utilities::matGet2D(m,j,i)<<"\t";

		}
		out<<"\n";
	}

}