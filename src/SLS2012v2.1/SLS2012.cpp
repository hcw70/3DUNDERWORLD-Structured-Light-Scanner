//------------------------------------------------------------------------------------------------------------
//* Copyright © 2010-2013 Immersive and Creative Technologies Lab, Cyprus University of Technology           *
//* Link: http://ict.cut.ac.cy                                                                               *
//* Software developer(s): Kyriakos Herakleous                                                               *
//* Researcher(s): Kyriakos Herakleous, Charalambos Poullis                                                  *
//*                                                                                                          *
//* This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.*
//* Link: http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US                                        *
//------------------------------------------------------------------------------------------------------------

#include "stdafx.h"
#include <stdio.h>
#include <iostream>
#include "SLS2012.h"
#include "GrayCodes.h"
#include "Projector.h"
#include "Scanner.h"
#include "Reconstructor.h"
#include "PointCloudImage.h"
#include "MeshCreator.h"
#include "CamProjCalib.h"
#include <direct.h>


int proj_h;
int proj_w;
int black_threshold;
bool autoContrast;
bool saveAutoContrast;
bool raySampling;
int white_threshold;
int webCamID;
int cam_w;
int cam_h;
cv::Point2i projectorWinPos;


void projectGraysOnly()
{

	std::cout << "Generating Gray Codes..."  ;
	GrayCodes *grayCode= new GrayCodes(proj_w,proj_h);
	grayCode->generateGrays();
	std::cout << "done!\n"  ;
	std::cout << "Press 'Enter to change projected code"  ;

	Projector *proj=new Projector(proj_w,proj_h);

	int i=0;
	int key = cvWaitKey(10);
	while(true)
	{
		key = cvWaitKey(10);
		proj->showImg( grayCode->getImg(i));

		if(key == 13)
			i++;

		if(i == (grayCode->getNumOfImgs()))
			i=0;
		if(key == 27)
			break;
	}

}

int renameDataSet()
{	
	WIN32_FIND_DATA data;
	HANDLE h;

	_chdir("reconstruction/dataSet/");

	char sel = 0;

	while( sel!='j' && sel != 't' && sel != 'p')
	{
		std::cout<<"Please specify image format, 'j' for jpg, 't' for tif or 'p' for png.\n"; 
		std::cin>>sel;
	}
	char *format; 

	if(sel=='j')
	{
		h = FindFirstFile(L"*.jpg",&data);
		format = ".jpg";
	}
	else if(sel=='t')
	{
		format = ".tif";
		h = FindFirstFile(L"*.tif",&data);
	}
	else if(sel=='p')
	{
		format = ".png";
		h = FindFirstFile(L"*.png",&data);
	}



	int count = 1;

	std::vector<std::string> list;

	if( h!=INVALID_HANDLE_VALUE ) 
	{
		int numOfFiles=0;

		do
		{
			char*  nPtr = new char [lstrlen( data.cFileName ) + 1];

			for( int i = 0; i < lstrlen( data.cFileName ); i++ )
				nPtr[i] = char( data.cFileName[i] );

			nPtr[lstrlen( data.cFileName )] = '\0';

			list.push_back(nPtr);

		} 
		while(FindNextFile(h,&data));

		for(int i = 0; i<list.size(); i++)
		{
			

			std::stringstream path1,path2;
			path1 << list[i];
			
			if(count < 10)
				path2<<'0';

			path2 << count << format;
		
			(char *)data.cFileName;

			bool a = rename(path1.str().c_str(), path2.str().c_str());

			std::cout<<path1.str().c_str()<<" to "<<path2.str().c_str()<<"\n";

			count++;

		} 
		while(FindNextFile(h,&data) && count <= numOfFiles);
	} 
	else 
		std::cout << "Error: No such folder." << std::endl;
	
	FindClose(h);
	
	return 0;
}

void createConfigurationFile(char* path)
{
	//set default settings
	projectorWinPos.x = proj_w + 300;
	projectorWinPos.y = -20;
	proj_h = 768;
	proj_w = 1024;
	black_threshold = 40;
	white_threshold = 5;
	webCamID = 0;
	cam_w=1600;
	cam_h=1200;
	autoContrast = true;
	saveAutoContrast = false;
	raySampling = true;

	cv::FileStorage fs(path, cv::FileStorage::WRITE);

	fs << "Projector" << "{:";
		fs << "Width" << proj_w << "Height" << proj_h ;
	fs<<"}";

	fs << "Camera" << "{:";
		fs << "ID"<< webCamID << "Width" << cam_w << "Height" << cam_h ;
	fs<<"}";

	fs << "ProjectorWindow" << "{:";
		fs << "Position" << "{:";
			fs << "x" << projectorWinPos.x << "y" << projectorWinPos.y ;
		fs<<"}";
	fs<<"}";
	
	fs << "Reconstruction" << "{:";
		fs<<"AutoContrast"<<autoContrast;
		fs<<"SaveAutoContrastImages"<<saveAutoContrast;
		fs<<"RaySampling"<<raySampling;
		fs<<"blackThreshold"<<black_threshold;
		fs<<"whiteThreshold"<<white_threshold;
	fs<<"}";

	fs.release();
}


bool loadConfigurations()
{
	cv::FileStorage fs("slsConfig.xml", cv::FileStorage::READ);

	if(!fs.isOpened())
	{
		std::cout << "Failed to open Configuration File. " << std::endl;
		return false;
	}

	cv::FileNode node = fs["Projector"];
	
		node["Width"] >> proj_w;
		node["Height"]>> proj_h;
	
	node= fs["Camera"];
		node["ID"] >> webCamID;
		node["Width"] >> cam_w;
		node["Height"] >> cam_h;
	
	node= fs["ProjectorWindow"];
		node = node["Position"];
			node["x"] >> projectorWinPos.x;
			node["y"] >> projectorWinPos.y;
	
	node= fs["Reconstruction"];
		node["blackThreshold"] >> black_threshold;
		node["whiteThreshold"] >> white_threshold;
		node["AutoContrast"] >> autoContrast;
		node["SaveAutoContrastImages"] >> saveAutoContrast;
		node["RaySampling"] >> raySampling;
	

	fs.release();

	return true;
}

void reconstruct()
{
	//change directory
	_chdir("reconstruction/");

	Reconstructor *reconstructor= new Reconstructor();

	//load projector and camera paramiters
	reconstructor->loadProjectorAndCamera();
	
	char sel=0;

	//load dataset
	while( sel!='j' && sel != 't' && sel != 'p')
	{
		std::cout<<"Please specify image format, 'j' for jpg, 't' for tif or 'p' for png.\n"; 
		std::cin>>sel;
	}

	if(sel=='j')
		reconstructor->setImgPath("dataset/","",".jpg");
	else if(sel=='t')
		reconstructor->setImgPath("dataset/","",".tif");
	else if(sel=='p')
		reconstructor->setImgPath("dataset/","",".png");

	//set reconstuction paramiters
	reconstructor->setBlackThreshold(black_threshold);
	reconstructor->setWhiteThreshold(white_threshold);
	
	if(autoContrast)
		reconstructor->enableAutoContrast();
	else
		reconstructor->disableAutoContrast();

	if(saveAutoContrast)
		reconstructor->enableSavingAutoContrast();
	else
		reconstructor->disableSavingAutoContrast();

	if(raySampling)
		reconstructor->enableRaySampling();
	else
		reconstructor->disableRaySampling();

	//recontruct
	reconstructor->runReconstruction();
	
	//Export mesh
	MeshCreator *meshCreator=new MeshCreator(reconstructor->points3DProjView);
	meshCreator->computeMesh("output/projector_view.obj");

	delete meshCreator;
	delete reconstructor;

}

void generateGrayCodes()
{
	//change directory
	_chdir("gray/");

	std::cout << "Generating Gray Codes..."  ;
	GrayCodes *gray=new GrayCodes(proj_w,proj_h);
	gray->generateGrays();
	std::cout << "done!\n"  ;

	std::cout << "Saving..."  ;
	gray->save();
	std::cout << "done!\n"  ;

	delete gray;
}

void calibration()
{
	//change directory
	_chdir("calibration/");

	CamProjCalib *calib= new CamProjCalib();

	//load images
	calib->loadCameraImgs();
	
	calib->extractCornersCameraCalibration();
	calib->calibrateCamera();

	calib->loadProjectorImgs();
	calib->loadProjectedCalibImg();

	calib->extractCornersProjectorCalibration();
	calib->calibrateProjector();
	
	//save data and results
	calib->saveCalibData("calibData.xml");
	calib->exportTxtFiles();

	// show data on consol
	calib->printData();

}

void printCopyRight()
{
	std::cout<<"\n";
	std::cout<<"---------------------------------------------------------------------\n";
	std::cout<<"* Copyright © 2010-2013 Immersive and Creative Technologies Lab,    *\n";
	std::cout<<"* Cyprus University of Technology                                   *\n";
	std::cout<<"* Link: http://ict.cut.ac.cy                                        *\n";
	std::cout<<"* Software developer(s): Kyriakos Herakleous                        *\n";
	std::cout<<"* Researcher(s): Kyriakos Herakleous, Charalambos Poullis           *\n";
	std::cout<<"*                                                                   *\n";
	std::cout<<"* This work is licensed under a Creative Commons                    *\n";
	std::cout<<"* Attribution-NonCommercial-ShareAlike 3.0 Unported License.        *\n";
	std::cout<<"* Link: http://creativecommons.org/licenses/by-nc-sa/3.0/deed.en_US *\n";
	std::cout<<"---------------------------------------------------------------------\n\n";

}

int _tmain(int argc, _TCHAR* argv[])
{
	
	printCopyRight();

	//load configurations
	if(!loadConfigurations())
	{
		std::cout<<"A new one with default settings has been created.\n\n";
		createConfigurationFile("slsConfig.xml");
		loadConfigurations();
	}

	std::cout<<"Task List\n1. Scan with Web Cam\n2. Reconstruct\n3. Scan With Canon Camera(with Calib)\n4. Scan With Canon Camera(Scan Only)\n5. GrayCodes Projection\n6. Generate and Save gray codes\n7. Calibration \n8. Create Confiquration xml file with default settings\n9. Rename the dataSet  \n\n Pleace select task! ";

	int select;
	std::cin>>select;

	//clear console
	system("cls");
	
	Scanner *scanner;

	switch(select)
	{
		// Scan with Web Cam
		case 1:
			scanner= new Scanner(SCANNER_USE_WEBCAM);
			scanner->scan(SCAN_N_CALIB);
			break;
		//Reconstruction
		case 2:
			reconstruct();
			break;
		//Scan With Canon Camera(with Calib)
		case 3:
			scanner= new Scanner(SCANNER_USE_CANON);
			scanner->scan(SCAN_N_CALIB);
			break;
		//Scan With Canon Camera(Scan Only)
		case 4:
			scanner= new Scanner(SCANNER_USE_CANON);
			scanner->scan(SCAN_ONLY);
			break;
		//GrayCodes Projection
		case 5:
			projectGraysOnly();
			break;
		//Generate gray codes
		case 6:
			generateGrayCodes();
			break;
		//Calibration
		case 7:
			calibration();
			break;
		//Create Default Configuration Settings XML file
		case 8:
			createConfigurationFile("slsConfigDefault.xml");
			break;
		case 9:
			renameDataSet();
			break;
	}

	std::cout<<"\nPress any key to exit.";
	getch();

	return 1;
}



