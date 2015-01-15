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
#include "PointCloudImage.h"

int PointCloudImage::accessPoint(int i,int j)
{
	int r = i*h+j;
	
	while(r>=MAXSIZE){
		r-=MAXSIZE;
	}
	return r;

}

int PointCloudImage::accessTable(int i,int j)
{
	int n = i*h+j;
	int r=0;
	
	while(n>=MAXSIZE){
		n-=MAXSIZE;
		r++;
	}
	return r;

}

PointCloudImage::PointCloudImage(int imageW,int imageH, bool colorFlag)
{
	w=imageW;
	h=imageH;

	numOfTables= ceil((w*h)/(float)MAXSIZE);

	points = new CvScalar *[numOfTables];

	int rem_size=w*h;

	for(int i=0; i<numOfTables; i++)
	{
		if(rem_size<=MAXSIZE)
		{
			points[i]= new CvScalar [rem_size];
		}
		else
		{
			points[i]= new CvScalar [MAXSIZE];
			rem_size -= MAXSIZE;
		}
	}

	if(colorFlag==true)
	{
		color = new CvScalar *[numOfTables];

		rem_size=w*h;

		for(int i=0; i<numOfTables; i++)
		{
			if(rem_size<=MAXSIZE)
			{
				color[i]= new CvScalar [rem_size];
			}
			else
			{
				color[i]= new CvScalar [MAXSIZE];
				rem_size -= MAXSIZE;
			}
		}

	}
	else
		color = NULL;

	numOfPointsForPixel = new int*[numOfTables];

	rem_size=w*h;

	for(int i=0; i<numOfTables; i++)
	{
		if(rem_size<=MAXSIZE)
		{
			numOfPointsForPixel[i]= new int [rem_size];

			for(int ii=0; ii<rem_size; ii++)
				numOfPointsForPixel[i][ii]=0;

		}
		else
		{
			numOfPointsForPixel[i]= new int [MAXSIZE];
			rem_size -= MAXSIZE;
			for(int ii=0; ii<MAXSIZE; ii++)
				numOfPointsForPixel[i][ii]=0;
		}
	}
}

PointCloudImage::~PointCloudImage(void)
{
	for(int i=0; i<numOfTables; i++)
	{
		if(points && points[i])
			delete points[i];
		if(numOfPointsForPixel&&numOfPointsForPixel[i])
			delete numOfPointsForPixel[i];
		if(color && color[i])
			delete color[i];
	}
	if(points)
		delete points;
	if(numOfPointsForPixel)
		delete numOfPointsForPixel;
	if(color)
		delete color;
}

bool PointCloudImage::setPoint(int i_w, int j_h, CvScalar point, CvScalar colorRGB)
{
	if(i_w>w || j_h>h)
		return false;

	setPoint(i_w,j_h,point);

	int p=accessPoint(i_w,j_h);
	int t=accessTable(i_w,j_h);

	if(color)
	{
		color[t][p].val[0] = colorRGB.val[0];
		color[t][p].val[1] = colorRGB.val[1];
		color[t][p].val[2] = colorRGB.val[2];
	}
	else
		return false;

	return true;
}

bool PointCloudImage::setPoint(int i_w, int j_h, CvScalar point)
{
	if(i_w>w || j_h>h)
		return false;

	int p=accessPoint(i_w,j_h);
	int t=accessTable(i_w,j_h);

	points[t][p].val[0] = point.val[0];
	points[t][p].val[1] = point.val[1];
	points[t][p].val[2] = point.val[2];

	numOfPointsForPixel[t][p]=1;

	return true;
}

bool PointCloudImage::getPoint(int i_w, int j_h, CvScalar *pointOut, CvScalar *colorOut)
{
	if(i_w>w || j_h>h)
		return false;

	int p=accessPoint(i_w,j_h);
	int t=accessTable(i_w,j_h);

	if(numOfPointsForPixel[t][p]>0)
	{
		if(pointOut)
		{
			pointOut->val[0]=points[t][p].val[0]/ double (numOfPointsForPixel[t][p]);
			pointOut->val[1]=points[t][p].val[1]/ double (numOfPointsForPixel[t][p]);
			pointOut->val[2]=points[t][p].val[2]/ double (numOfPointsForPixel[t][p]);
		}

		if(colorOut)
		{
			if(color)
			{
				colorOut->val[0]=color[t][p].val[0]/ double (numOfPointsForPixel[t][p]);
				colorOut->val[1]=color[t][p].val[1]/ double (numOfPointsForPixel[t][p]);
				colorOut->val[2]=color[t][p].val[2]/ double (numOfPointsForPixel[t][p]);
			}
			else
			{
				return false;
			}
		}

		return true;
	}
	else
	{
		return false;
	}
	
}

bool PointCloudImage::addPoint(int i_w, int j_h, CvScalar point, CvScalar colorRGB)
{
	if(i_w>w || j_h>h)
		return false;

	int p=accessPoint(i_w,j_h);
	int t=accessTable(i_w,j_h);

	if(numOfPointsForPixel[t][p]==0)
		return setPoint(i_w,j_h,point,colorRGB);

	addPoint(i_w,j_h,point);

	if(color)
	{
		color[t][p].val[0] += colorRGB.val[0];
		color[t][p].val[1] += colorRGB.val[1];
		color[t][p].val[2] += colorRGB.val[2];
	}
	else
		return false;

	return true;
}

bool PointCloudImage::addPoint(int i_w, int j_h, CvScalar point)
{
	if(i_w>w || j_h>h)
		return false;

	int p=accessPoint(i_w,j_h);
	int t=accessTable(i_w,j_h);

	if(numOfPointsForPixel[t][p]==0)
		return setPoint(i_w,j_h,point);

	points[t][p].val[0] += point.val[0];
	points[t][p].val[1] += point.val[1];
	points[t][p].val[2] += point.val[2];

	numOfPointsForPixel[t][p]+=1;

	return true;
}



void PointCloudImage::exportXYZ(char path[], bool exportOffPixels, bool colorFlag)
{
	std::ofstream out1; 
	out1.open(path);
	int load;
	CvScalar p,c;

	std::cout<<"Export "<< path << "...";

	for(int i = 0; i<w; i++)
	{
		for(int j = 0; j<h; j++)
		{

			int pp=accessPoint(i,j);
			int t=accessTable(i,j);

			if(!exportOffPixels && numOfPointsForPixel[t][pp]==0)
				continue;			
			
			getPoint(i,j,&p,&c);

			if(exportOffPixels && numOfPointsForPixel[t][pp]==0)
			{
				p.val[0]=-1;
				p.val[1]=-1;
				p.val[2]=-1;
				c.val[0]=-1;
				c.val[1]=-1;
				c.val[2]=-1;
			}

			out1<<p.val[0]<<" "<<p.val[1]<<" "<<p.val[2];

			if(colorFlag && color)
			{
				out1<<" "<<c.val[2]<<" "<<c.val[1]<<" "<<c.val[0]<<"\n";
			}
			else
			{
				out1<<"\n";
			}
		}
	}

	out1.close();
	std::cout<<"done\n";
}

void PointCloudImage::exportNumOfPointsPerPixelImg(char path[])
{
	
	IplImage *projToCamRays = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U,  1);

	float max=0;

	int maxX,maxY;

	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{
			if(numOfPointsForPixel[accessTable(i,j)][accessPoint(i,j)]>max)
			{
				max=numOfPointsForPixel[accessTable(i,j)][accessPoint(i,j)];
				maxX=i;
				maxY=j;
			}
		}
	} 

	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{
			cvSet2D(projToCamRays,j,i,cvScalar(numOfPointsForPixel[accessTable(i,j)][accessPoint(i,j)]/max*255));
		}
	}

	cvSaveImage("reconstruction/projToCamRays.png",projToCamRays);

	std::ofstream out1;
	std::stringstream txt;
	txt<<path<<".txt";
	out1.open(txt.str().c_str() );

	out1<< "black color = 0\nwhite color = "<< max <<"\nmax Pixel: ("<<maxX<<","<<maxY<<")";

	out1.close();
	cvReleaseImage(&projToCamRays);
}

int PointCloudImage::getWidth()
{
	return w;
}

int PointCloudImage::getHeight()
{
	return h;
}

