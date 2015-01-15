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
#include "VirtualCamera.h"


VirtualCamera::VirtualCamera(void)
{
	distortion=NULL;
	rotationMatrix=NULL;
	translationVector=NULL;
}


VirtualCamera::~VirtualCamera(void)
{
}

void VirtualCamera::loadDistortion(std::string path)
{
	loadMatrix(&distortion,5,1,path);
}

void VirtualCamera::loadCameraMatrix(std::string path)
{
	CvMat* camMatrix=NULL;
	loadMatrix(&camMatrix,3,3,path);

	cc.val[0]=cvGet2D(camMatrix,0,2).val[0];
	cc.val[1]=cvGet2D(camMatrix,1,2).val[0];

	fc.val[0]=cvGet2D(camMatrix,0,0).val[0];
	fc.val[1]=cvGet2D(camMatrix,1,1).val[0];

	cvReleaseMat(&camMatrix);
}

void VirtualCamera::loadRotationMatrix(std::string path)
{
	loadMatrix(&rotationMatrix,3,3,path);
}

void VirtualCamera::loadTranslationVector(std::string path)
{
	loadMatrix(&translationVector,3,1,path);
}

int VirtualCamera::loadMatrix(CvMat **matrix,int s1,int s2 ,std::string file){

	std:: ifstream in1; 
	in1.open(file.c_str());
	
	if(in1==NULL)
	{
		std::cout<<"Error loading file "<<file.c_str()<<"\n";
		return -1;

	}

	if(*matrix!=NULL)
		cvReleaseMat(matrix);

	*matrix=cvCreateMat(s1,s2,CV_32FC1);

	for(int i=0; i<s1; i++)
	{
		for(int j=0; j<s2; j++)
		{
			float val;
			in1>>val;
			CvScalar v;
			v.val[0]=val; 
			if(s2>1)
				cvSet2D(*matrix,i,j,v);
			else
				(*matrix)->data.fl[i]=val;

		}
	}
	return 1;
}