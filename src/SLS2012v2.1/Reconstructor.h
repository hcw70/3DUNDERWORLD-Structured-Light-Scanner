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

#include "stdafx.h"
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <conio.h>
#include "GrayCodes.h"
#include "VirtualCamera.h"
#include "Utilities.h"
#include "PointCloudImage.h"
#include "SLS2012.h"
#include "Sample.h"

#define RECONSTRUCTION_SAMPLING_ON
#define SAMPLES_NUM 25
#define COL_GRAY_OFFSET 0
#define ROW_GRAY_OFFSET 0
#define EXPORT_MASKED_POINTS
#define SCALE 1

class Reconstructor
{
	public:
		Reconstructor(void);
		~Reconstructor(void);

		void loadProjectorAndCamera();
		void runReconstruction();

		VirtualCamera projector;
		VirtualCamera camera;
		PointCloudImage *points3DCamView;
		PointCloudImage *points3DProjView;

		void setImgPath(char path1st[], char path2nd[], char extension[] );
		void saveShadowImg(char path[]);
		void saveDecodedRowImg(char path[]);
		void saveDecodedColImg(char path[]);

		void setBlackThreshold(int val);
		void setWhiteThreshold(int val);

		void enableAutoContrast();
		void disableAutoContrast();
		void enableSavingAutoContrast();
		void disableSavingAutoContrast();
		void enableRaySampling();
		void disableRaySampling();

	private:
		
		void loadCamImgs();
		void unloadCamImgs();
		void findProjectorCenter();
		void computeShadows();
		void proj2camSpace(cv::Point3f &p);
		bool Reconstructor::getProjPixelForCamPixel(int x, int y, cv::Point &p_out);
		void decodePaterns();
		int  access(int i,int j, int h);
		void smoothDecode();
		void camProjPixelsTriangulation();
		void projectorViewImage();
		
		std::stringstream filePath1st;
		std::stringstream filePath2nd;
		std::stringstream fileExtension;

		int numberOfImgs;
		int numOfColBits;
		int numOfRowBits;

		int blackThreshold;
		int whiteThreshold;

		cv::vector<cv::Mat> camImgs;
		cv::Mat mask;					//matrix with vals 0 and 1 , CV_8U , uchar
		cv::Mat maskImg;
		cv::Mat decRows;
		cv::Mat decCols;
		cv::Mat colorImg;

		int col_gray_offset;
		int row_gray_offset;

		bool pathSet;

		cv::Mat decColsMatrix;
		cv::Mat decRowsMatrix;

		bool autoContrast_;
		bool saveAutoContrast_;
		bool raySampling_;
};

