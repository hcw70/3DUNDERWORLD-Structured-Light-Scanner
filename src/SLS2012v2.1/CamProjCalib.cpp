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
#include "CamProjCalib.h"

CamProjCalib::CamProjCalib(void)
{
	squareSize.width = 0;
	squareSize.height = 0;

	numOfCamImgs = 0;
	numOfProjImgs = 0;

	camCalibrated = false;
	projCalibrated = false;

}

CamProjCalib::~CamProjCalib(void)
{
	unloadCameraImgs();
	unloadProjectedCalibImg();
	unloadProjectorImgs();
}

//----------------------------------------Load & Export Data-------------------------------------- 


void CamProjCalib::loadCalibData(char *path)
{
	cv::FileStorage fs(path, cv::FileStorage::READ);
	
	if(!fs.isOpened())
	{
		std::cout << "Failed to open Calibration Data File. " << std::endl;
		return;
	}

	cv::FileNode node = fs["Camera"];
		node["Calibrated"] >> camCalibrated;
		node["Matrix"] >> camMatrix;
		node["Distortion"]>> camDist;
		node["Height"] >> camImageSize.height;
		node["Width"] >> camImageSize.width;

	node = fs["Projector"];
		node["Calibrated"] >> projCalibrated;
		node["Matrix"] >> projMatrix;
		node["Distortion"] >> projDist;
		node["Rotation"] >> projRotationMatrix;
		node["Translation"] >> projTranslationVector;
		node["Height"] >> projImageSize.height;
		node["Width"] >> projImageSize.width;

	node = fs["BoardSquare"];
	
		node["Height"] >> squareSize.height;
		node["Width"] >> squareSize.width;
	

	cv::FileNode features = fs["ExtractedFeatures"];
	
		cv::FileNode images = features["CameraImages"];
			int size = images["NumberOfImgs"];

				for(int i=0; i<size; i++)
				{
					std::stringstream name;
					name << "Image" << i+1;

					cv::FileNode image = images[name.str()];
					
					cv::vector<cv::Point2f> in2;
					cv::vector<cv::Point3f> in3;

					image["BoardCorners"]>>in2;
					imgBoardCornersCam.push_back(in2);

					image["ObjBoardCorners"]>>in3;
					objBoardCornersCam.push_back(in3);

				}
	
		images = features["ProjectedCalibrationImage"];

			images["Corners"] >> projectedCalibImgCorners; 
	
		images = features["ProjectorImages"];
			size = images["NumberOfImgs"];

				for(int i=0; i<size; i++)
				{
					std::stringstream name;
					name << "Image" << i+1;

					cv::FileNode image = images[name.str()];
					
					cv::vector<cv::Point2f> in2;
					

					image["ProjectedPaternCorners"]>>in2;
					imgPaternCornersProj.push_back(in2);

					image["BoardCorners"]>>in2;
					imgBoardCornersProj.push_back(in2);

					image["ObjBoardCorners"]>>in2;
					objBoardCornersProj.push_back(in2);

				}

	fs.release();
}


void CamProjCalib::saveCalibData(char *path)
{
	cv::FileStorage fs(path, cv::FileStorage::WRITE);
	
	fs << "Camera" << "{:";
		fs<< "Calibrated" << camCalibrated << "Matrix" << camMatrix << "Distortion" << camDist;
		fs<<"Height" << camImageSize.height<<"Width" << camImageSize.width;
	fs<<"}";

	fs << "Projector" << "{:";
		fs << "Calibrated" << projCalibrated<< "Matrix" << projMatrix << "Distortion" << projDist << "Rotation" 
			<< projRotationMatrix << "Translation" << projTranslationVector;
		fs<<"Height" << projImageSize.height<<"Width" << projImageSize.width;
	fs<<"}";
	
	
	fs << "BoardSquare" << "{:";
		fs << "Height" << squareSize.height << "Width" << squareSize.width; 
	fs<<"}";

	fs << "ExtractedFeatures" << "{:";
		
		fs << "CameraImages" << "{:";

			int size = imgBoardCornersCam.size();
			fs << "NumberOfImgs" << size;

				for(int i=0; i<imgBoardCornersCam.size(); i++)
				{

					std::stringstream name;
					name << "Image" << i+1;
					fs<<name.str()<< "{:";

						fs<<"BoardCorners"<<imgBoardCornersCam[i];
						fs<<"ObjBoardCorners"<<objBoardCornersCam[i];

					fs<<"}";

				}
	
		fs<<"}";

		fs << "ProjectedCalibrationImage" << "{:";

			fs << "Corners" << projectedCalibImgCorners; 
	
		fs<<"}";

		fs << "ProjectorImages" << "{:";

			size = imgPaternCornersProj.size();
			fs << "NumberOfImgs" << size;

				for(int i=0; i<imgPaternCornersProj.size(); i++)
				{

					std::stringstream name;
					name << "Image" << i+1;
					fs<<name.str()<< "{:";

						fs<<"ProjectedPaternCorners"<<imgPaternCornersProj[i];
						fs<<"BoardCorners"<<imgBoardCornersProj[i];
						fs<<"ObjBoardCorners"<<objBoardCornersProj[i];

					fs<<"}";

				}
	
		fs<<"}";

		
	
	fs<<"}";

	fs.release();
}

void CamProjCalib::exportTxtFiles()
{
	_chdir("output/");

	Utilities::exportMat("cam_matrix.txt",camMatrix);
	Utilities::exportMat("proj_matrix.txt",projMatrix);
	Utilities::exportMat("proj_rotation_matrix.txt",projRotationMatrix);
	Utilities::exportMat("proj_trans_vectror.txt",projTranslationVector);
	Utilities::exportMat("proj_distortion.txt",projDist);
	Utilities::exportMat("cam_distortion.txt",camDist);

	_chdir("../");
	
}

//-------------------------------------------Tools----------------------------------------------

void CamProjCalib::perspectiveTransformation(cv::vector<cv::Point2f> corners_in,cv::Mat homoMatrix, cv::vector<cv::Point3f> &points_out)
{

	for(int i=0; i<corners_in.size(); i++)
	{
		cv::Point3f p;

		double x = corners_in[i].x, y = corners_in[i].y;
		
		double Z = 1./(homoMatrix.at<double>(6) *x + homoMatrix.at<double>(7)*y + homoMatrix.at<double>(8));
		double X =    (homoMatrix.at<double>(0) *x + homoMatrix.at<double>(1)*y + homoMatrix.at<double>(2))*Z;
		double Y =    (homoMatrix.at<double>(3) *x + homoMatrix.at<double>(4)*y + homoMatrix.at<double>(5))*Z;
		
		p.x = (float) X;
		p.y = (float) Y;
		p.z = 0;

		points_out.push_back(p);

	}

}

void CamProjCalib::undistortCameraImgPoints(cv::vector<cv::Point2f> points_in,cv::vector<cv::Point2f> &points_out)
{
	cv::undistortPoints(points_in,points_out,camMatrix,camDist);

	float fX = camMatrix.at<double>(0,0);
	float fY = camMatrix.at<double>(1,1);
	float cX = camMatrix.at<double>(0,2);
	float cY = camMatrix.at<double>(1,2);

	for(int j=0; j<points_out.size(); j++)
	{

		points_out[j].x = (points_out[j].x*fX)+cX;
		points_out[j].y = (points_out[j].y*fY)+cY;

	}
}

//draw color in area outside the given rectangle
void CamProjCalib::drawOutsideOfRectangle(cv::Mat img,cv::vector<cv::Point2f> rectanglePoints, float color)
{

	std::vector<cv::Point> corners;
	for(int i=0; i<rectanglePoints.size(); i++)
	{
		corners.push_back(rectanglePoints[i]);
	}

	cv::Mat mask(img.size(),img.type());
	cv::Mat background(img.size(),img.type());
	
	mask =  1;
	cv::fillConvexPoly(mask, corners ,cv::Scalar(0));

	background = color;
	background.copyTo(img,mask);
	
}


//-----------------------------------------mouse callbacks for calibration--------------------------------------------


//callback to choose 4 corners on calibration board
void calib_board_corners_mouse_callback( int event, int x, int y, int flags, void* param )
{
	
	cv::vector<cv::Point2f> *corners= (cv::vector<cv::Point2f>*) param;

	int ev = event;
	
	switch( event )
	{
		
		case CV_EVENT_LBUTTONDOWN:
			if(corners->size() ==4)
				break;
			corners->push_back(cv::Point(x,y));
			break;

	}
}

//return image point
void image_point_return( int event, int x, int y, int flags, void* param )
{

	CvScalar *point= (CvScalar*) param;

	switch( event )
	{
		case CV_EVENT_LBUTTONDOWN:
			
			point->val[0]=x;
			point->val[1]=y;
			point->val[2]=1;
			break;
	}

}


//--------------------------------------------------------------------------------------------------------------------

void CamProjCalib::loadCameraImgs()
{

	while(numOfCamImgs == 0)
	{
		std::cout<<"Give number of camera calibration images: ";
		std::cin>>numOfCamImgs;
	}

	std::cout<<"Loading Camera Calibration Images...";

	for(int i=0; i<numOfCamImgs;i++)
	{
		std::stringstream path;

		path<<"camera/"<<i+1<<".jpg";
		
		cv::Mat img = cv::imread(path.str().c_str() );
		
		if(img.empty())
		{
			std::cout<<"Error loading cam image "<<i+1<<"!";
			getch();
			exit(-1);
		}

		camImgs.push_back(img);
	}

	if(!camImgs[0].empty())
		camImageSize = camImgs[0].size();

	std::cout<<"done!\n";

}

void CamProjCalib::unloadCameraImgs()
{

	for(int i=0; i<camImgs.size();i++)
		camImgs[i].release();

}

void CamProjCalib::loadProjectorImgs()
{
	
	while(numOfProjImgs == 0)
	{
		std::cout<<"Give number of projector calibration images: ";
		std::cin>>numOfProjImgs;
	}

	std::cout<<"Loading Projector Calibration Images...";

	for(int i=0; i<numOfProjImgs;i++)
	{
		std::stringstream path;

		path<<"projector/"<<i+1<<".jpg";
		
		cv::Mat img= cv::imread(path.str().c_str() );
		
		if(img.empty())
		{
			std::cout<<"Error loading cam image "<<i+1<<"!";
			getch();
			exit(-1);
		}

		projImgs.push_back(img);
	}

	std::cout<<"done!\n";
}

void CamProjCalib::unloadProjectorImgs()
{
	for(int i=0; i<projImgs.size();i++)
		projImgs[i].release();
}

void CamProjCalib::loadProjectedCalibImg()
{

		std::stringstream path;

		
		path<<"cal.png";
		
		projCalibImg = cv::imread(path.str().c_str() );
		
		if(projCalibImg.empty())
		{
			std::cout<<"Error loading cam image projector's calibration image!";
			getch();
			exit(-1);
		}

		if(!projCalibImg.empty())
			projImageSize = projCalibImg.size();

}

void CamProjCalib::unloadProjectedCalibImg()
{
	projCalibImg.release();			
}

//allow user to select a rectangular area in the image returning the for corners of the area
cv::vector<cv::Point2f>  CamProjCalib::manualMarkCheckBoard(cv::Mat img)
{
	
	cv::vector<cv::Point2f> corners;
		
	cv::namedWindow("Mark Calibration Board",CV_WINDOW_NORMAL);
	cv::resizeWindow("Mark Calibration Board",800,600);

	//Set a mouse callback
	cv::setMouseCallback( "Mark Calibration Board",calib_board_corners_mouse_callback, (void*) &corners);

	bool ok = false;

	while(!ok)
	{
		corners.clear();
		cv::resizeWindow("Mark Calibration Board",800,600);

		int curNumOfCorners=0;
			
		cv::Mat img_copy ;
		img.copyTo(img_copy);

		system("cls");

		std::cout<<"Please click on the 4 extrime corners of the board, starting from the top left corner\n";

		cv::Point2f rectSize(20,20);

		while(corners.size()<4)
		{
			//draw selected corners and conection lines
			if(curNumOfCorners<corners.size())
			{
				int s = corners.size();
					
				cv::rectangle(img_copy,	corners[s-1] - rectSize,corners[s-1] + rectSize,cvScalar(0,0,255),3);
				
				if(!(corners.size()==1))
				{
					cv::line(img_copy, corners[s-1],corners[s-2],cvScalar(0,0,255),3);
				}
				
				curNumOfCorners++;
				
			}

			cv::imshow("Mark Calibration Board", img_copy);
			cv::waitKey(2);
		}

		//Draw corners and lines		
		cv::rectangle( img_copy,	corners[3] - rectSize, corners[3] + rectSize, cvScalar(0,0,255), 3);
		cv::line(img_copy, corners[3],corners[2],cvScalar(0,0,255),10);
		cv::line(img_copy, corners[3],corners[0],cvScalar(0,0,255),10);
		
		system("cls");
		std::cout<<"Press 'Enter' to continue or 'ESC' to select a new area!\n";

		int key = 0;

		//wait for enter or esc key press
		while( key!=27 && key!=13 )
		{
			cv::imshow("Mark Calibration Board", img_copy );
			key = cv::waitKey();
		}

		//if enter set ok as true to stop the loop or repeat the selection process
		if(key == 13)
			ok = true;
		else
			ok = false;

		img_copy.release();
			
	}

	cv::destroyWindow("Mark Calibration Board");
		
	return corners;
}


float CamProjCalib::markWhite(cv::Mat img)
{
	
		float white;
		cv::namedWindow("Mark White",CV_WINDOW_NORMAL);
		cv::resizeWindow("Mark White",800,600);

		cv::Scalar point;

		// Set a mouse callback
		cv::setMouseCallback( "Mark White",image_point_return, (void*) &point);

		bool ok = false;
		
		while(!ok)
		{
			cv::Mat img_copy;
			img.copyTo(img_copy);
			
			cv::resizeWindow("Mark White",800,600);
			
			int pointsCount=0;
			point.val[2]=0;
			
			while(pointsCount==0)
			{
				if(point.val[2]==1)
				{
					cv::rectangle(img_copy, cvPoint(point.val[0]-10,point.val[1]-10),cvPoint(point.val[0]+10,point.val[1]+10),cvScalar(0,0,255),3);
					
					white = img.at<uchar>(point.val[1],point.val[0]);
					
					pointsCount++;
					point.val[2]=0;
				}

				cv::imshow("Mark White", img_copy );
				cv::waitKey(2);
			}
							

			int key = 0;

			while(key != 27 && key != 13)
			{
				cv::imshow("Mark White", img_copy );
				key=cv::waitKey();
			}

			if(key==13)
				ok=true;
			else
				ok=false;

			img_copy.release();
		}

		cvDestroyWindow("Mark White");
		

		return white;
}


bool CamProjCalib:: findCornersInCamImg(cv::Mat img,cv::vector<cv::Point2f> *camCorners,cv::vector<cv::Point3f> *objCorners)
{

	//copy camera img
	cv::Mat img_grey;
	cv::Mat img_copy;
	img.copyTo(img_copy);

	int numOfCornersX;
	int numOfCornersY;

	bool found=false;

	//find the corners
	while(!found)
	{
		//convert the copy to gray
		cv::cvtColor( img, img_grey, CV_RGB2GRAY );
		img.copyTo(img_copy);

		//ask user to mark 4 corners of the checkboard
		cv::vector<cv::Point2f> chessBoardCorners = manualMarkCheckBoard(img_copy);

		//ask user to mark a white point on checkboard
		float color = markWhite(img_grey);

		drawOutsideOfRectangle(img_grey,chessBoardCorners, color);

		//show img to user
		cv::namedWindow("Calibration",CV_WINDOW_NORMAL);
		cv::resizeWindow("Calibration",800,600);

		cv::imshow("Calibration",img_grey);
		cv::waitKey(20);

		system("cls");

		//ask the number of squares in img
		std::cout<<"Give number of squares on x axis: ";
		std::cin>>numOfCornersX;
		std::cout<<"Give number of squares on y axis: ";
		std::cin>>numOfCornersY;

		if(numOfCornersX<=0 || numOfCornersY<=0)
			break;

		if(numOfCornersX<=3 || numOfCornersY<=3)
		{
			std::cout<<"Board size must be >3\n";
			continue;
		}

		numOfCornersX--;
		numOfCornersY--;
		
		
		found=cv::findChessboardCorners(img_grey, cvSize(numOfCornersX,numOfCornersY), *camCorners, CV_CALIB_CB_ADAPTIVE_THRESH );

		std::cout<<"found = "<<camCorners->size()<<"\n";

		cv::drawChessboardCorners(img_copy, cvSize(numOfCornersX,numOfCornersY), *camCorners, found);

		int key = cv::waitKey(1);

		if(key==13)
			break;

		std::cout<<"\nPress 'Enter' to continue or 'ESC' to repeat the procedure.\n";

		while(found)
		{
			cv::imshow("Calibration", img_copy );

			key = cv::waitKey(1);

			if(key==27)
				found=false;
			if(key==13)
			break;
		}

	}


	//if corners found find subPixel
	if(found)
	{

		//convert the copy to gray
		cv::cvtColor( img, img_grey, CV_RGB2GRAY );

		//find sub pix of the corners
		cv::cornerSubPix(img_grey, *camCorners, cvSize(20,20), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

		system("cls");

		if(squareSize.height == 0)
		{
			std::cout<<"Give square height in mm: ";
			std::cin>>squareSize.height;

			std::cout<<"Give square width in mm: ";
			std::cin>>squareSize.width;
		}

		for(int i=0; i<numOfCornersY ; i++)
		{
			for(int j=0; j<numOfCornersX; j++)
			{
				cv::Point3f p;
				p.x = j*squareSize.width;
				p.y = i*squareSize.height;
				p.z = 0;
				objCorners->push_back(p);
			}
		}

	}

	cv::destroyWindow("Calibration");
	
	
	return found;
}


int CamProjCalib::extractCornersCameraCalibration()
{

	if(camImgs.size()==0)
		loadCameraImgs();

	imgBoardCornersCam.clear();
	objBoardCornersCam.clear();

	for(int i=0; i<numOfCamImgs; i++)
	{
		int cornersReturn;

		cv::vector<cv::Point2f> cCam;
		cv::vector<cv::Point3f> cObj;

		findCornersInCamImg(camImgs[i], &cCam, &cObj );
		
		if(cCam.size())
		{
			imgBoardCornersCam.push_back(cCam);
			objBoardCornersCam.push_back(cObj);
		}
		
	}

	return 1;
}


int CamProjCalib::calibrateCamera()
{
	//check if corners for camera calib has been extracted
	if(imgBoardCornersCam.size() == 0)
		extractCornersCameraCalibration();

	cv::vector<cv::Mat> camRotationVectors;
  	cv::vector<cv::Mat> camTranslationVectors;

	cv::calibrateCamera(objBoardCornersCam,imgBoardCornersCam,camImageSize,camMatrix, camDist, camRotationVectors,camTranslationVectors,0,
		cv::TermCriteria( (cv::TermCriteria::COUNT)+(cv::TermCriteria::EPS), 30, DBL_EPSILON) );

	camCalibrated = true;

	return 1;
}

int CamProjCalib::findProjectorCorners(cv::Mat orgImg, cv::vector<cv::Point2f> &projCorners)
{

	std::vector<cv::Mat> rgb;
	cv::Mat img;

	orgImg.copyTo(img);
		
	cv::split(img,rgb);
		
	cv::Mat dst=rgb[0]-rgb[1]-rgb[2];

	dst=255-dst;
	cv::equalizeHist(dst,dst);

	cv::namedWindow("rgb",CV_WINDOW_NORMAL);
	cv::resizeWindow("rgb",800,600);
	
	cv::namedWindow("patern",CV_WINDOW_NORMAL);
	cv::resizeWindow("patern",800,600);

	cv::imshow("patern",dst);

	cv::imshow("rgb",img);

	int key = cv::waitKey(0);

	if(key == 27) 
		return 0;;

	bool found=cv::findChessboardCorners(dst, cvSize(9,6), projCorners, CV_CALIB_CB_ADAPTIVE_THRESH );

	std::cout<<" found = "<<projCorners.size()<<"\n";

	if(found)
		cv::cornerSubPix(rgb[2], projCorners, cvSize(20,20), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

	cv::drawChessboardCorners(img, cvSize(9,6), projCorners, found);

	cv::imshow("rgb",img);

	cv::waitKey(0);

	cv::destroyWindow("rgb");
	cv::destroyWindow("patern");

	return found;

}


void CamProjCalib::manualMarkCalibBoardCorners(cv::Mat img,cv::vector<cv::Point2f> &imgPoints_out, cv::vector<cv::Point2f> &objPoints_out)
{

	cv::Mat img_copy;
	cv::Mat img_grey;

	img.copyTo(img_copy);
	cv::cvtColor( img, img_grey, CV_BGR2GRAY );
	
	//get calibration board corners
	cv::vector<cv::Point2f> imgPoints = manualMarkCheckBoard(img_copy);
	cv::cornerSubPix(img_grey, imgPoints, cvSize(15,15), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

	//draw on image	
	cv::rectangle(img_copy,	imgPoints[0] - cv::Point2f(10,10),imgPoints[0] + cv::Point2f(10,10),cvScalar(0,0,255),3);
	
	for(int i=0; i<4; i++)
	{
		cv::line(img_copy, cv::Point2f(imgPoints[i].x-20,imgPoints[i].y),cv::Point2f(imgPoints[i].x+20,imgPoints[i].y),cvScalar(255,0,0),3);
		cv::line(img_copy, cv::Point2f(imgPoints[i].x,imgPoints[i].y+20),cv::Point2f(imgPoints[i].x,imgPoints[i].y-20),cvScalar(255,0,0),3);
	}

	cv::line(img_copy, imgPoints[0],imgPoints[1],cvScalar(255,255,255),4);
	cv::line(img_copy, imgPoints[0],imgPoints[1],cvScalar(0,0,255),3);
	cv::line(img_copy, imgPoints[3],imgPoints[0],cvScalar(255,255,255),4);
	cv::line(img_copy, imgPoints[3],imgPoints[0],cvScalar(0,255,0),3);
		
	
	cv::namedWindow("Marked Board",CV_WINDOW_NORMAL);
	cv::resizeWindow("Marked Board",800,600);
	cv::imshow("Marked Board", img_copy);

	cv::waitKey(10);
	cv::waitKey(10);

	system("cls");

	float xS,yS;
	std::cout<< "Give number of squares on x axis: ";
	std::cin>>xS;
	std::cout<< "Give number of squares on y axis: ";
	std::cin>>yS;

	if(squareSize.height == 0)
	{
		std::cout<<"Give square height in mm: ";
		std::cin>>squareSize.height;

		std::cout<<"Give square width in mm: ";
		std::cin>>squareSize.width;
	}

	xS=xS*squareSize.width;
	yS=yS*squareSize.height;

	//set object points real world 2D
	cv::vector<cv::Point2f> objPoints;
	objPoints.push_back(cv::Point2f(0,0));
	objPoints.push_back(cv::Point2f(xS,0));
	objPoints.push_back(cv::Point2f(xS,yS));
	objPoints.push_back(cv::Point2f(0,yS));

	imgPoints_out = imgPoints;
	objPoints_out = objPoints;

	cv::destroyWindow("Marked Board");
}

bool CamProjCalib::findCameraExtrisics(cv::vector<cv::Point2f> imgPoints, cv::vector<cv::Point2f> objPoints2D,cv::Mat &rMat_out, cv::Mat &tVec_out, cv::Mat &homoMatrix_out)
{

	//find homography
	homoMatrix_out = cv::findHomography(imgPoints,objPoints2D);	
	
	cv::Mat rVec;
	
	//set object points of real world in 3D
	cv::vector<cv::Point3f> objPoints3D;
	
	for(int i=0; i<objPoints2D.size(); i++)
	{
		objPoints3D.push_back( cv::Point3f(objPoints2D[i].x ,objPoints2D[i].y ,0) );
	}

	//find extrinsics rotation & translation
	cv::solvePnP(objPoints3D,imgPoints,camMatrix,camDist,rVec,tVec_out);
	cv::Rodrigues(rVec,rMat_out);

	std::cout<<rMat_out<<"\n\n\n"<<tVec_out<<"\n\n\n";

	if(homoMatrix_out.empty())
		return false;
	else
		return true;

}

bool CamProjCalib::findCornersInProjectionImg(cv::Mat img,cv::vector<cv::Point2f> &points_out)
{

	cv::Mat img_grey;
	cv::cvtColor( img, img_grey, CV_RGB2GRAY );
	img_grey= 150 - img_grey;

	cv::namedWindow("Projected Image",CV_WINDOW_NORMAL);
	cv::resizeWindow("Projected Image",800,600);
	
	cv::imshow("Projected Image",img_grey);
	
	cv::waitKey(0);
	
	bool found=cv::findChessboardCorners(img_grey, cvSize(9,6), points_out, CV_CALIB_CB_ADAPTIVE_THRESH );

	
	if(found)
		cv::cornerSubPix(img_grey, points_out, cvSize(20,20), cvSize(-1,-1), cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

	cv::Mat img_copy;
	img.copyTo(img_copy);
	cv::drawChessboardCorners(img_copy, cvSize(9,6), points_out, found);
	

	cv::imshow("Projected Image",img_copy);

	cv::waitKey(0);

	cv::destroyWindow("Projected Image");
	
	return found;

}

int CamProjCalib::extractCornersProjectorCalibration()
{

	//check if projector Images are loaded
	if(projImgs.size()==0)
	{
		loadProjectorImgs();
	}

	//check if projected image corners have been extracted
	if(projectedCalibImgCorners.empty())
	{

		if(projCalibImg.empty())
		{
			loadProjectedCalibImg();
		}

		if(!findCornersInProjectionImg(projCalibImg,projectedCalibImgCorners))
		{
			std::cout<<"Projector's Calibration Image's corners could not be found.\n";
			return(-1);
		}
	}

	int numOfImgs = projImgs.size();

	imgPaternCornersProj.clear();
	imgBoardCornersProj.clear();
	objBoardCornersProj.clear();

	//extract corners for each projector picture
	for(int i=0; i<numOfImgs; i++)
	{

		cv::vector<cv::Point2f> projCorners;
		cv::vector<cv::Point2f> imgPoints,objPoints;

		//extract projected patern coreners
		if(!findProjectorCorners(projImgs[i], projCorners))
			continue;
		
		//extract calibration board's corners
		manualMarkCalibBoardCorners(projImgs[i],imgPoints,objPoints);

		//store board corners
		imgPaternCornersProj.push_back(projCorners);
		imgBoardCornersProj.push_back(imgPoints);
		objBoardCornersProj.push_back(objPoints);
		
	}
	
	return 1;
}

int CamProjCalib::calibrateProjector()
{
	//check if camera is calibrated
	if(!camCalibrated)
		calibrateCamera();

	//check if corners are extracted from projector images
	if(imgBoardCornersProj.size() == 0 || objBoardCornersProj.size() == 0 || imgPaternCornersProj.size() == 0)
		extractCornersProjectorCalibration();

	//check if projected image corners have been extracted
	if(projectedCalibImgCorners.empty())
	{

		if(projCalibImg.empty())
		{
			loadProjectedCalibImg();
		}

		if(!findCornersInProjectionImg(projCalibImg,projectedCalibImgCorners))
		{
			std::cout<<"Projector's Calibration Image's corners could not be found.\n";
			return(-1);
		}
	}

	cv::vector<cv::vector<cv::Point3f>> proj3DPoints;
	cv::vector<cv::vector<cv::Point2f>> calibImgPoints;
	
	for(int i=0; i<imgPaternCornersProj.size(); i++)
	{
		cv::vector<cv::Point2f> undistProjCorners;
		cv::Mat homography, rMat, tVec;
		cv::vector<cv::Point2f> imgPoints,objPoints,undistImgPoints;

		undistortCameraImgPoints(imgPaternCornersProj[i],undistProjCorners);

		imgPoints = imgBoardCornersProj[i];
		objPoints = objBoardCornersProj[i];

		undistortCameraImgPoints(imgPoints,undistImgPoints);

		if(!findCameraExtrisics(undistImgPoints,objPoints,rMat,tVec,homography))
			continue;
		
		cv::vector<cv::Point3f> proj3DP;
		
		perspectiveTransformation(undistProjCorners,homography,proj3DP);

		proj3DPoints.push_back(proj3DP);
		calibImgPoints.push_back(projectedCalibImgCorners);

	}
	
	cv::vector<cv::Mat> projRotationVectors;
  	cv::vector<cv::Mat> projTranslationVectors;
	
	//calibrate projector
	cv::calibrateCamera(proj3DPoints, calibImgPoints, projImageSize, projMatrix, projDist, projRotationVectors, projTranslationVectors,0,cv::TermCriteria( cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON) );

	//find extrinsics
	findProjectorExtrinsics();

	projCalibrated=true;

	return 1;
}

int CamProjCalib::findProjectorExtrinsics(int projImgNo)
{
	
	if(projectedCalibImgCorners.empty())
	{
		if(!findCornersInProjectionImg(projCalibImg,projectedCalibImgCorners))
		{
			std::cout<<"Projector's Calibration Image's corners could not be found.\n";
			return(-1);
		}
	}

	cv::vector<cv::Point2f> imgCornersGeneral;
	cv::vector<cv::Point3f> objCornersCamSpace;

	for(int i=0; i<imgPaternCornersProj.size(); i++)
	{
		
		cv::Mat camRotationMatrix,camTranslationVector,homoMatrix;

		cv::vector<cv::Point2f>  undistImgBoardCorners;

		undistortCameraImgPoints(imgBoardCornersProj[i],undistImgBoardCorners);

		if(!findCameraExtrisics(undistImgBoardCorners, objBoardCornersProj[i], camRotationMatrix, camTranslationVector, homoMatrix))
			return -1;

		cv::vector<cv::Point2f> projCorners;
		projCorners = imgPaternCornersProj[i];

		undistortCameraImgPoints(projCorners,projCorners);
		
		
		cv::vector<cv::Point3f> proj3DP;
				
		perspectiveTransformation(projCorners,homoMatrix,proj3DP);
		
		//convert corners to camera space
		for(int i=0; i<proj3DP.size(); i++)
		{
			cv::Mat p(3,1,CV_64F);
			p.at<double>(0,0) = proj3DP[i].x;
			p.at<double>(1,0) = proj3DP[i].y;
			p.at<double>(2,0) = proj3DP[i].z;

			p = camRotationMatrix*p;
		
			p.at<double>(0,0) = p.at<double>(0,0) + camTranslationVector.at<double>(0,0);
			p.at<double>(1,0) = p.at<double>(1,0) + camTranslationVector.at<double>(1,0);
			p.at<double>(2,0) = p.at<double>(2,0) + camTranslationVector.at<double>(2,0);

			proj3DP[i].x = p.at<double>(0,0);
			proj3DP[i].y = p.at<double>(1,0);
			proj3DP[i].z = p.at<double>(2,0);

			objCornersCamSpace.push_back(proj3DP[i]);
			imgCornersGeneral.push_back(projectedCalibImgCorners[i]);
		}

	}

	cv::Mat rVec;
	cv::solvePnP(objCornersCamSpace,imgCornersGeneral, projMatrix, projDist,rVec,projTranslationVector);
	cv::Rodrigues(rVec,projRotationMatrix);

}


void CamProjCalib::setSquareSize(cv::Size size_in_mm)
{
	squareSize = size_in_mm;
}

cv::Size CamProjCalib::getSquareSize()
{
	return squareSize;
}

void CamProjCalib::setNumberOfCameraImgs(int num)
{
	numOfCamImgs = num;
}

int CamProjCalib::getNumberOfCameraImgs()
{
	return numOfCamImgs;
}

void CamProjCalib::setNumberOfProjectorImgs(int num)
{
	numOfProjImgs = num;
}

int CamProjCalib::getNumberOfProjectorImgs()
{
	return numOfProjImgs;
}

void CamProjCalib::printData()
{
	system("cls");

	std::cout<<"-----Camera Matrix------\n";
	std::cout<<camMatrix<<"\n\n";

	std::cout<<"-----Camera Distortion------\n";
	std::cout<<camDist<<"\n\n";

	std::cout<<"-----Projector Matrix------\n";
	std::cout<<projMatrix<<"\n\n";

	std::cout<<"-----Projector Distortion------\n";
	std::cout<<projDist<<"\n\n";

	std::cout<< "------Projector Rotation------\n";
	std::cout << projRotationMatrix << "\n\n";

	std::cout<< "------Projector Translation------\n";
	std::cout << projTranslationVector << "\n\n";

	
	

}