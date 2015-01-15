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

#include <conio.h>
#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "Utilities.h"
#include <direct.h>


class CamProjCalib
{

public:
	CamProjCalib(void);
	~CamProjCalib(void);

	int calibrateProjector();
	int calibrateCamera();

	void loadCameraImgs();
	void unloadCameraImgs();

	void loadProjectorImgs();
	void unloadProjectorImgs();

	void loadProjectedCalibImg();
	void unloadProjectedCalibImg();

	void saveCalibData(char *path);
	void CamProjCalib::exportTxtFiles();
	void loadCalibData(char *path);

	//set data
	void setSquareSize(cv::Size size);
	cv::Size getSquareSize();

	void setNumberOfCameraImgs(int num);
	int getNumberOfCameraImgs();

	void setNumberOfProjectorImgs(int num);
	int getNumberOfProjectorImgs();

	void printData();

	int  findProjectorExtrinsics(int projImgNo=0);

	int  extractCornersCameraCalibration();
	int  extractCornersProjectorCalibration();

	cv::Mat camMatrix;
	cv::Mat projMatrix;

	cv::Mat camDist;
	cv::Mat projDist;

	cv::Mat projRotationMatrix;
	cv::Mat projTranslationVector;

private:

	//functions
	bool						findCornersInCamImg(cv::Mat camImg,cv::vector<cv::Point2f> *camCorners,cv::vector<cv::Point3f> *objCorners);
	int							findProjectorCorners(cv::Mat orgImg, cv::vector<cv::Point2f> &projCorners);
	bool						findCornersInProjectionImg(cv::Mat img,cv::vector<cv::Point2f> &points_out);

	void						drawOutsideOfRectangle(cv::Mat img,cv::vector<cv::Point2f> rectanglePoints, float color);

	cv::vector<cv::Point2f>		manualMarkCheckBoard(cv::Mat img);
	float						markWhite(cv::Mat img);
	void						manualMarkCalibBoardCorners(cv::Mat img,cv::vector<cv::Point2f> &imgPoints_out, cv::vector<cv::Point2f> &objPoints_out);

	void						perspectiveTransformation(cv::vector<cv::Point2f> corners_in,cv::Mat homoMatrix, cv::vector<cv::Point3f> &points_out);
	void						undistortCameraImgPoints(cv::vector<cv::Point2f> points_in,cv::vector<cv::Point2f> &points_out);
	
	bool						findCameraExtrisics(cv::vector<cv::Point2f> imgPoints, cv::vector<cv::Point2f> objPoints,cv::Mat &rMat_out, cv::Mat &tVec_out, cv::Mat &homoMatrix_out);

	//corners data
	cv::vector<cv::Point2f> projectedCalibImgCorners;

	cv::vector<cv::vector<cv::Point2f> > imgPaternCornersProj;
	cv::vector<cv::vector<cv::Point2f> > imgBoardCornersProj; 
	cv::vector<cv::vector<cv::Point2f> > objBoardCornersProj;

	cv::vector<cv::vector<cv::Point2f>> imgBoardCornersCam;
	cv::vector<cv::vector<cv::Point3f>> objBoardCornersCam;
	
	//images
	cv::Vector<cv::Mat> camImgs;
	cv::Vector<cv::Mat> projImgs;
	cv::Mat projCalibImg;

	cv::Size squareSize;
	int numOfCamImgs;
	int numOfProjImgs;

	cv::Size camImageSize;
	cv::Size projImageSize;

	bool camCalibrated;
	bool projCalibrated;

};

