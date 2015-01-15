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
#include "MeshCreator.h"


MeshCreator::MeshCreator(PointCloudImage *in)
{
	cloud=in;
	w=cloud->getWidth();
	h=cloud->getHeight();
	pixelNum = new int[w*h];

}

MeshCreator::~MeshCreator(void)
{
	delete pixelNum;
}

void MeshCreator::computeMesh(std::string path)
{

	int count=1;
	bool return_val;
	cv::Point3f point;
	std::ofstream out1; 

	out1.open(path.c_str());

	std::cout<<"Export "<< path << "...";

	for(int i=0; i<w;i++)
	{
		for(int j=0; j<h; j++)
		{
			
			return_val = cloud->getPoint(i,j,point);

			if(return_val)
			{
				pixelNum[access(i,j)]=count;
				out1<<"v "<< point.x<< " "<< point.y<< " "<<point.z<< "\n";  
				count++;
			}
			else
				pixelNum[access(i,j)]=0;
		}
	}

	for(int i=0; i<w;i++)
	{
		for(int j=0; j<h; j++)
		{
			int v1=pixelNum[access(i,j)],v2,v3;
			
			if(i<w-1)
				v2=pixelNum[access(i+1,j)];
			else
				v2=0;

			if(j<h-1)
				v3=pixelNum[access(i,j+1)];
			else
				v3=0;

			if(v1!=0 && v2!=0 && v3!=0)
				out1<<"f "<< v1<<"/"<<v1<< " "<< v2<<"/"<<v2<< " "<<v3<<"/"<<v3<<"\n";  
			
			if(j>0&&i<w-1)
				v3=pixelNum[access(i+1,j-1)];
			else
				v3=0;

			if(v1!=0 && v2!=0 && v3!=0)
				out1<<"f "<< v1<<"/"<<v1<< " "<< v3<<"/"<<v3<< " "<<v2<<"/"<<v2<<"\n";  
		}
	}
	out1.close();
	std::cout<<"done!\n";
}

int MeshCreator::access(int i,int j)
{
	return i*h+j;
}

