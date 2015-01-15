//------------------------------------------------------------------------------------------------------------
//* Copyright � 2010-2013 Immersive and Creative Technologies Lab, Cyprus University of Technology           *
//* Link: http://ict.cut.ac.cy                                                                               *
//* Software developer(s): Kyriakos Herakleous                                                               *
//* Researcher(s): Kyriakos Herakleous, Charalambos Poullis                                                  *
//*                                                                                                          *
//* This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.*
//* Link: http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US                                        *
//------------------------------------------------------------------------------------------------------------

#pragma once


#include "stdafx.h"
#include "SLS2012.h"
#include <iostream>
#include <fstream>
using std::ofstream;
#include "cv.h"
#include "highgui.h"

#include "EDSDK.h"
#include "EDSDKErrors.h"
#include "EDSDKTypes.h"

#define STRICT
#include <windows.h>
#include <algorithm>
using std::min;
using std::max;
#include <gdiplus.h>
#include "GrayCodes.h"

#include <conio.h>

#include "CameraController.h"
#include "WebCam.h"
#include "CanonCamera.h"
#include <atlimage.h>
#include "Projector.h"

#define SCANNER_USE_WEBCAM true
#define SCANNER_USE_CANON false

#define SCAN_ONLY true
#define SCAN_N_CALIB false

class Scanner
{

public:
	Scanner(bool web);

	~Scanner(void);

	void scan(bool scanOnly);

	void capturePaterns(CameraController *camera[],int camCount);
	
	bool captureCalib(CameraController *camera);

	//capture images and save them on path folder
	bool captureCalib(CameraController *camera, char* path);
	void capturePaterns(CameraController *camera[],int camCount, char* path);

private:

	bool web;

	IplImage* projCalibBoard ;

	CameraController *camera[5];

	GrayCodes *grayCodes;

	Projector *proj;
};

