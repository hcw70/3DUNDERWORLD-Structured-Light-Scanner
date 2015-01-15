//------------------------------------------------------------------------------------------------------------
//* Copyright © 2010-2013 Immersive and Creative Technologies Lab, Cyprus University of Technology           *
//* Link: http://ict.cut.ac.cy                                                                               *
//* Software developer(s): Kyriakos Herakleous                                                               *
//* Researcher(s): Kyriakos Herakleous, Charalambos Poullis                                                  *
//*                                                                                                          *
//* This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.*
//* Link: http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US                                        *
//------------------------------------------------------------------------------------------------------------

#pragma once

#include "cv.h"
#include "VirtualCamera.h"

class Utilities
{

	public:
		Utilities(void);
		~Utilities(void);
		static bool XOR(bool val1, bool val2);
		static void normalize3d(CvScalar &vec);
		static void normalize3dtable(double vec[3]);
		static void pixelToImageSpace(double p[3], CvScalar fc, CvScalar cc);
		static void Utilities::pixelToImageSpace(cv::Point3f &p, float ccX,float ccY, float fcX, float fcY);
		static void undistortPoints( double xi, double yi, VirtualCamera *cam,double *xd,double *yd);
		static CvScalar planeRayInter(CvScalar planeNormal,CvScalar planePoint, CvScalar rayVector, CvScalar rayPoint );
		static float matGet2D(cv::Mat m, int x, int y);
		static void matSet2D(cv::Mat m, int x, int y, double val);
		static void autoContrast(cv::Mat img_in, cv::Mat &img_out);
		static void autoContrast(IplImage *img_in, IplImage *img_out);
		static void Utilities::exportMat(char *path, cv::Mat m);

};

