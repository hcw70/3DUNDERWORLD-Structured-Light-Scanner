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
#include "Scanner.h"


Scanner::Scanner(bool webCam)
{
	web=webCam;
}


Scanner::~Scanner(void)
{

}

bool Scanner::captureCalib(CameraController *camera)
{
	//startCameraLiveView
	camera->startLiveview();

	
	int key;
	int count=0;

	while(true)
	{
		

		camera->UpdateView();

		key = cvWaitKey(10);

		///If enter is pressed then capture the image
		if (key == 13)
		{
			camera->captureImg();
			count++;
			std::cout<<count<<" image(s) have been captured.\n";
		}
		
		//32-> enter / 27-> esc 
		if(key == 32 || key == 27)
		{
			break;
		}

		
	}

	camera->endLiveview();

	cvWaitKey(100);
	if(key == 27)
		return 0;
	else 
		return 1;

}

//capture photos for calibration, photos will saved on path
//this is curently available only for 
bool Scanner::captureCalib(CameraController *camera, char* path)
{

	//if is a canon cam regular captureCalib is called
	if(camera->isCanonCam())
	{
		return captureCalib(camera);
	}


	//startCameraLiveView
	camera->startLiveview();

	
	int key;

	int count=0;

	while(true)
	{
		

		camera->UpdateView();

		key = cvWaitKey(10);

		///If enter is pressed then capture the image
		if (key == 13)
		{
			camera->captureImg(path);
			count++;
			std::cout<<count<<" image(s) have been captured.\n";
		}
		
		//32-> enter / 27-> esc 
		if(key == 32 || key == 27)
		{
			break;
		}

		
	}

	camera->endLiveview();

	cvWaitKey(100);
	if(key == 27)
		return 0;
	else 
		return 1;

}


void Scanner::capturePaterns(CameraController *camera[],int camCount)
{
	
	int grayCount=0;

	for(int i=0; i<camCount; i++)
	{
		camera[i]->resetSaveCount();
	}

	proj->showImg(grayCodes->getImg(0));

	while(true)
	{
		int key;

		for(int i=0; i<camCount; i++)
		{
			if(camera[i]->isWebCam())
				camera[i]->captureImg("scan/dataSet/");
			else
				camera[i]->captureImg();
		}
			
		grayCount++;

		std::cout<<"Capture " << grayCount<<" of "<<grayCodes->getNumOfImgs()<< ".\n";

		if(grayCount==grayCodes->getNumOfImgs())
			break;

		proj->showImg(grayCodes->getNextImg());
			
		key=cvWaitKey(100);
	
		if(key == 27)
			break;
	}

	cvWaitKey(300);

}



void Scanner::scan(bool scanOnly)	
{
	std::cout << "Starting Scanner...\n\n" << std::endl;
	
	int projW = proj_w;
	int projH = proj_h;

	std::cout << "\t-Generate Gray Codes..."  ;

		grayCodes= new GrayCodes(projW,projH);
		grayCodes->generateGrays();
	
	std::cout << "done!\n" << std::endl;
	std::cout << "\t-Load Projector Calibration Image..." ;
	
		projCalibBoard = cvLoadImage("scan/cal.png");

		if(!projCalibBoard)
		{
			std::cout<<"\nError: Projector Calibration Image not found.\n";
			getch();
			exit(-1);
		}

	std::cout << "done!\n" << std::endl;
	std::cout << "\t-Setting up Projector and Camera..." ;

		proj=new Projector(projW,projH);

		EdsInitializeSDK();

		int numOfCams;

		camera[0] = new CameraController(web);
		numOfCams =	camera[0]->getNumOfCams();

		for(int i=1; i<numOfCams; i++)
		{
			camera[i] = new CameraController(web);
		}

	
		proj->showImg(grayCodes->getImg(0));

	std::cout << "done!\n" << std::endl;

	
	bool continue_val=true;

	//take calibration pictures
	if(!scanOnly)
	{
		for(int i=0; i<numOfCams; i++)
		{
			
			cvWaitKey(1);
			
			std::cout << "\nPress 'Enter' to capture photos for camera calibration. When you are done press 'Space'.\n" << std::endl;

			//capture calibration images with camera [i]
			if(camera[i]->isWebCam())
				continue_val = captureCalib(camera[i],"scan/camCalib/");
			else
				continue_val = captureCalib(camera[i]);


			//if user dont want ot continue break
			if(!continue_val)
				break;

			//project calibration patern
			proj->showImg(projCalibBoard);
			cvWaitKey(1);

			//affects only webCams
			camera[i]->resetSaveCount();

			std::cout << "\nPress 'Enter' to capture photos for projector calibration. When you are done press 'Space'.\n" << std::endl;

			//capture projector calibration images
			if(camera[i]->isWebCam())
				continue_val = captureCalib(camera[i],"scan/projCalib/");
			else
				continue_val = captureCalib(camera[i]);
		
			//if user dont want to continue break
			if(!continue_val)
				break;

			
		}
	}


	if(continue_val)
	{

		proj->showImg(grayCodes->getNextImg());
		cvWaitKey(100);

		std::cout<<"System is ready to scan object. Press 'Enter' to start the Automatic Scanning\n";
		
		int key=0;
		
		while(key!=13)
			key = cvWaitKey(10);

		capturePaterns(camera, numOfCams);
	}
	
	for(int i=0; i<numOfCams; i++)
	{
		delete camera[i];
	}

	EdsTerminateSDK();


	///Destroy the window
	delete proj;
	
	
}