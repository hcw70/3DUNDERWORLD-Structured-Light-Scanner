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
#include "Reconstructor.h"


Reconstructor::Reconstructor(void)
{
	col_gray_offset=0;
	row_gray_offset=0;

	pathSet=false;

	mask = NULL;
	maskImg = NULL;
	decRows = NULL;
	decCols = NULL;
	colorImg = NULL;
	
	points3DCamView = NULL;
	points3DProjView = NULL;

	decColsMatrix = NULL;
	decRowsMatrix = NULL;

	autoContrast_ = false;
	saveAutoContrast_ = false;

}


Reconstructor::~Reconstructor(void)
{

	unloadCamImgs();
	
	if(points3DCamView)
		delete points3DCamView ;
	if(points3DProjView)
		delete points3DProjView ;

	if(!decColsMatrix.empty())
		decColsMatrix.release();

	if(!decRowsMatrix.empty())
		decRowsMatrix.release();
}

void Reconstructor::enableAutoContrast()
{
	autoContrast_ = true;
}

void Reconstructor::disableAutoContrast()
{
	autoContrast_ = false;
}

void Reconstructor::enableSavingAutoContrast()
{
	saveAutoContrast_ = true;
}

void Reconstructor::disableSavingAutoContrast()
{
	saveAutoContrast_ = false;
}

void Reconstructor::enableRaySampling()
{
	raySampling_ = true;
}

void Reconstructor::disableRaySampling()
{
	raySampling_ = false;
}

void Reconstructor::setBlackThreshold(int val)
{
	blackThreshold = val;
}

void Reconstructor::setWhiteThreshold(int val)
{
	whiteThreshold = val;
}

int Reconstructor::access(int i,int j, int h)
{
	return i*h+j;
}

void Reconstructor::decodePaterns()
{
	int w=camera.width;
	int h=camera.height;

	//initialize matrices
	if(!decColsMatrix.empty())
		decColsMatrix.release();

	decColsMatrix =  cv::Mat(h,w,CV_32F);

	if(!decRowsMatrix.empty())
		decRowsMatrix.release();

	decRowsMatrix =  cv::Mat(h,w,CV_32F);


	cv::Point projPixel;
	
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if(mask.at<uchar>(j,i))
			{

				

				//get the projector pixel for camera (i,j) pixel
				bool error = getProjPixelForCamPixel(i,j,projPixel);

				if(error)
				{
					mask.at<uchar>(j,i)=0;
					continue;
				}

				decColsMatrix.at<float>(j,i) = projPixel.x;
				decRowsMatrix.at<float>(j,i) = projPixel.y;
				
			}
		}

	}

}

void Reconstructor::projectorViewImage()
{

	int w=camera.width;
	int h=camera.height;

	cv::Mat projViewImg(projector.height,projector.width,CV_8UC3, cv::Scalar::all(0));
	cv::Mat numOfVals(projector.height,projector.width,CV_8UC1, cv::Scalar::all(0));
		 
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if(mask.at<uchar>(j,i))
			{
				int x,y;

				x = decColsMatrix.at<float>(j,i);
				y = decRowsMatrix.at<float>(j,i);

				if(x>=projector.width || y>=projector.height)
					continue;
				
				projViewImg.at<cv::Point3_<uchar>>(x,y).x +=  Utilities::matGet3D(colorImg,j,i,0);
				projViewImg.at<cv::Point3_<uchar>>(x,y).y +=  Utilities::matGet3D(colorImg,j,i,1);
				projViewImg.at<cv::Point3_<uchar>>(x,y).z +=  Utilities::matGet3D(colorImg,j,i,2);

				numOfVals.at<uchar>(x,y) ++;
			}
		}
	}

	for(int i=0; i<projector.width; i++)
	{
		for(int j=0; j<projector.height; j++)
		{
			
			//if the pixel is not shadow reconstruct
			if(numOfVals.at<uchar>(j,i))
			{
			
				projViewImg.at<cv::Point3_<uchar>>(j,i).x = (uchar) projViewImg.at<cv::Point3_<uchar>>(j,i).x / numOfVals.at<uchar>(j,i);
				projViewImg.at<cv::Point3_<uchar>>(j,i).y = (uchar) projViewImg.at<cv::Point3_<uchar>>(j,i).y / numOfVals.at<uchar>(j,i);
				projViewImg.at<cv::Point3_<uchar>>(j,i).z = (uchar) projViewImg.at<cv::Point3_<uchar>>(j,i).z / numOfVals.at<uchar>(j,i);
				
			}
		}
	}

	cv::imwrite("projectorView.png",projViewImg);
}


void Reconstructor::smoothDecode()
{

	int w=camera.width;
	int h=camera.height;

	CvScalar projPixel;
	
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if(mask.at<uchar>(j,i))
			{

				int p = (int) Utilities::matGet2D(decRowsMatrix,i,j);

				int ii=1;
				
				
				while(p == (int) Utilities::matGet2D(decRowsMatrix,i,j+ii) )
				{
					ii++;
				}

				if(ii>1)
				{
					float step = 1.0/ (float) ii;

					for(int m=0; m<ii; m++)
					{
						Utilities::matSet2D(decRowsMatrix,i,j+m,p - 0.5 + m*step);
					}

				}

				j = j + ii -1;
			}
		}

	}


	for(int j=0; j<h; j++)
	{
		for(int i=0; i<w; i++)
		{

			//if the pixel is not shadow reconstruct
			if(mask.at<uchar>(j,i))
			{

				int p = (int) Utilities::matGet2D(decColsMatrix,i,j);

				int ii=1;

				while(p == (int) Utilities::matGet2D(decColsMatrix,i+ii,j) )
				{
					ii++;
				}

				if(ii>1)
				{
					float step = 1.0/ (float) ii;

					for(int m=0; m<ii; m++)
					{
						Utilities::matSet2D(decColsMatrix,i+m,j, p -0.5 + m*step);
					}

				}

				i = i + ii -1;
			}
		}
	}


}

void Reconstructor::loadProjectorAndCamera()
{

	camera.loadCameraMatrix("input/cam_matrix.txt");
	camera.loadDistortion("input/cam_distortion.txt");

	camera.height=0;
	camera.width =0;

	projector.loadCameraMatrix("input/proj_matrix.txt");
	projector.loadDistortion("input/cam_distortion.txt");
	projector.loadRotationMatrix("input/proj_rotation_matrix.txt");
	projector.loadTranslationVector("input/proj_trans_vectror.txt");

	projector.height = proj_h;
	projector.width  = proj_w;

}

//load camera images
void Reconstructor::loadCamImgs()
{
	
	cv::Mat tmp;

	std::cout<<"Loading Camera Images...";

	for(int i=0; i<numberOfImgs;i++)
	{
		std::stringstream path;

		path<<filePath1st.str();
		if(i+1<10)
			path<<"0";
		path<<i+1<<filePath2nd.str()<<fileExtension.str();
		
		tmp.release();
		tmp = cv::imread(path.str().c_str());
		
		if(tmp.empty())
		{
			std::cout<<"\nError loading cam image "<<i+1<<". Press Any Key to Exit.";
			getch();
			exit(-1);
		}

		//store color image
		if(i==0)
		{
			colorImg = cv::imread(path.str().c_str());

			//auto contrast
			if(autoContrast)
			{
				Utilities::autoContrast(colorImg,colorImg);	
			}
		}
		
		//auto contrast
		if(autoContrast)
		{
			Utilities::autoContrast(tmp,tmp);

			if(saveAutoContrast)
			{
				std::stringstream p;
				p<<filePath1st.str()<<"AutoContrastSave/"<<i+1<<filePath2nd.str()<<fileExtension.str();

				cv::imwrite(p.str().c_str(),tmp);
			}
		}
		cv::cvtColor(tmp, tmp, CV_BGR2GRAY);

		camImgs.push_back(tmp);
	}

	if(camera.width==0)
	{
		camera.height=camImgs[0].rows;
		camera.width =camImgs[0].cols;
	}

	std::cout<<"done!\n";
	
}

//unload camera images
void Reconstructor::unloadCamImgs()
{

	if(camImgs.size())
	{
		for(int i=0; i<numberOfImgs; i++)
		{
			camImgs[i].release();
		}
	}
	
	camImgs.clear();	
}

void Reconstructor::findProjectorCenter()
{
	
	camera.position=cv::Point3f(0,0,0);
	
	cv::Mat Tmp(3,1,CV_32F);
	cv::Mat Tptmp(3,1,CV_32F);

	Tptmp = - projector.translationVector;

	cv::solve(projector.rotationMatrix,Tptmp,Tmp);
	
	//projector center
	projector.position.x= Utilities::matGet2D(Tmp,0,0);
	projector.position.y= Utilities::matGet2D(Tmp,0,1);
	projector.position.z= Utilities::matGet2D(Tmp,0,2);
	
}

void Reconstructor::computeShadows()
{
	std::cout<<"Estimating Shadows...";

	int w = camera.width;
	int h = camera.height;

	mask = cv::Mat(h,w,CV_8U,cv::Scalar(0));
	maskImg = cv::Mat(cvSize(w, h), CV_8UC3, cv::Scalar(0));

	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{
			float blackVal, whiteVal;
			
			//std::cout<<i<<" "<<j<<"\n";

			blackVal  = (float) Utilities::matGet2D( camImgs[1], i, j);
			whiteVal  = (float) Utilities::matGet2D( camImgs[0], i, j);

			if(whiteVal - blackVal > blackThreshold)
			{
				Utilities::matSet2D(mask,i,j,1);
				Utilities::matSet3D(maskImg,i,j,cv::Vec3f(255,255,255));
			}
			else
			{
				Utilities::matSet2D(mask,i,j,0);
				Utilities::matSet3D(maskImg,i,j,cv::Vec3f(0,0,0));
			}
		}
	}

	std::cout<<"done!\n";
}



void Reconstructor::runReconstruction()
{

	if(projector.distortion.empty() || camera.distortion.empty())
	{
		std::cout<<"camera or projector not initialized";
		exit(-1);
	}

	if(pathSet==false)
	{
		std::cout<<"imgs path not set";
		exit(-1);
	}

	GrayCodes grays(projector.width,projector.height);

	numOfColBits = grays.getNumOfColBits();
	numOfRowBits = grays.getNumOfRowBits();
	numberOfImgs = grays.getNumOfImgs();

	loadCamImgs();

	points3DCamView  = new PointCloudImage( camera.width    , camera.height    , false ); 
	points3DProjView = new PointCloudImage( projector.width , projector.height , false );

	findProjectorCenter();

	computeShadows();

	std::cout<<"Decoding paterns...";
	decodePaterns();
	std::cout<<"done!\n";
	camProjPixelsTriangulation();

	unloadCamImgs();
}

//convert a point from projector to camera space
void Reconstructor::proj2camSpace(cv::Point3f &p)
{
	
	cv::Mat tmp(3,1,CV_32F);
	cv::Mat tmpPoint(3,1,CV_32F);

	tmpPoint.at<float>(0) = p.x;
	tmpPoint.at<float>(1) = p.y;
	tmpPoint.at<float>(2) = p.z;

	tmp = -projector.rotationMatrix.t() * projector.translationVector ;
	tmpPoint = projector.rotationMatrix.t() * tmpPoint;

	p.x = tmp.at<float>(0) + tmpPoint.at<float>(0);
	p.y = tmp.at<float>(1) + tmpPoint.at<float>(1);
	p.z = tmp.at<float>(2) + tmpPoint.at<float>(2);
	
}


//for a (x,y) pixel of the camera returns the corresponding projector pixel
bool Reconstructor::getProjPixelForCamPixel(int x, int y, cv::Point &p_out)
{
	cv::vector<bool> grayCol;
	cv::vector<bool> grayRow;

	bool error = false;
	int error_code = 0;
	int xDec,yDec;

	

	//prosses column images
	for(int count=0; count<numOfColBits; count++)
	{
		//get pixel intensity for regular pattern projection and it's inverse 
		double val1, val2;
		val1 = Utilities::matGet2D(camImgs[count * 2 + 2   ],x,y);
		val2 = Utilities::matGet2D(camImgs[count * 2 + 2 +1],x,y);
		
		//check if intensity deference is in a valid rage
		if(abs(val1-val2) < whiteThreshold && error_code<numOfColBits-count )
			error_code=numOfColBits-count+1;

		//determine if projection pixel is on or off
		if(val1>val2)
			grayCol.push_back(1);
		else
			grayCol.push_back(0);

	}

	xDec = GrayCodes::grayToDec(grayCol) - col_gray_offset;


	//prosses row images
	for(int count=0; count<numOfRowBits; count++)
	{

		double val1, val2;

		val1 = Utilities::matGet2D(camImgs[count*2+2+numOfColBits*2],x,y);
		val2 = Utilities::matGet2D(camImgs[count*2+2+numOfColBits*2+1],x,y);

		if(abs(val1-val2) < whiteThreshold && error_code < numOfRowBits - count )  //check if the difference between the values of the normal and it's inverce projection image is valid
			error_code = numOfRowBits - count + 1;

		if(val1 > val2)
			grayRow.push_back(1);
		else
			grayRow.push_back(0);

	}
	
	//decode
	yDec = GrayCodes::grayToDec(grayRow) - row_gray_offset; 

	if((yDec > projector.height || xDec>projector.width) && error_code==0)
	{
		error_code == 1;	
	}

	if(error_code!=0)
	{
		error=true;

		if( error_code == 1 )
		{
			Utilities::matSet3D(maskImg,x,y,cv::Vec3d(0,255,0));
		}
		else
		{
			Utilities::matSet3D(maskImg,x,y,cv::Vec3d(0 ,0 , 100 + 155 * (float (error_code) / float (numOfRowBits))));
		}
	}
	else
		error=false;

	p_out.x = xDec;
	p_out.y = yDec;

	return error;
}

void Reconstructor::setImgPath(char path1st[], char path2nd[], char extension[] )
{
	fileExtension<<extension;
	filePath1st<<path1st;
	filePath2nd<<path2nd;

	pathSet=true;
}

void Reconstructor::saveShadowImg(char path[])
{
	if(maskImg.empty())
		return;

	cv::imwrite(path,maskImg);

}

void Reconstructor::saveDecodedRowImg(char path[])
{
	if(decRows.empty())
		return;

	cv::imwrite(path,decRows);
}

void Reconstructor::saveDecodedColImg(char path[])
{
	if(decCols.empty())
		return;

	cv::imwrite(path,decCols);
}

void Reconstructor::camProjPixelsTriangulation()
{
	
	std:: ofstream out1; 

	int w = camera.width;
	int h = camera.height;

	//decoding Images
	decRows = cv::Mat(cvSize(w,h), CV_8U);
	decCols = cv::Mat(cvSize(w,h), CV_8U);
	
	//set jetter paramiters for cam rays
	double rangeX,rangeY;
	cv::Point2f p1,p2;
	cv::Point3f up1,up2;

	p1 = Utilities::undistortPoints(cv::Point2f(0,0),camera);
	p2 = Utilities::undistortPoints(cv::Point2f(camera.width,camera.height),camera);

	up1 = Utilities::pixelToImageSpace(p1,camera);
	up2 = Utilities::pixelToImageSpace(p2,camera);

	rangeX = abs(p2.x - p1.x);
	rangeY = abs(p2.y - p1.y);

	//generade Sample values
	cv::Point2d jitterCamRays[SAMPLES_NUM];
	Sample::Jitter(jitterCamRays,SAMPLES_NUM);

	for(int i=0; i<SAMPLES_NUM; i++)
	{
		jitterCamRays[i].x = jitterCamRays[i].x*rangeX / ((double)w*10000);
		jitterCamRays[i].y = jitterCamRays[i].y*rangeY / ((double)h*10000);
	}

	//set jetter paramiters for projector rays
	p1 = Utilities::undistortPoints(cv::Point2f(0,0),projector);
	p2 = Utilities::undistortPoints(cv::Point2f(projector.width,projector.height),projector);

	up1 = Utilities::pixelToImageSpace(p1,projector);
	up2 = Utilities::pixelToImageSpace(p2,projector);

	rangeX = abs( p2.x - p1.x);
	rangeY = abs( p1.y - p2.y);
	

	//generade Sample values
	cv::Point2d jitterProjRays[SAMPLES_NUM];
	Sample::Jitter(jitterProjRays,SAMPLES_NUM);

	for(int i=0; i<SAMPLES_NUM; i++)
	{
		jitterProjRays[i].x = jitterProjRays[i].x*rangeX / ((double)w*10000);
		jitterProjRays[i].y = jitterProjRays[i].y*rangeY / ((double)h*10000);
	}

	//start reconstraction
	int load=0;

	//reconstraction for every camera pixel
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{
			
			if(mask.at<uchar>(j,i)) //if the pixel is not shadow reconstruct
			{

				if(load != (int) (((j+(float)i*h)/((float)w*h))*100))
				{
					load =  (int) (((j+(float)i*h)/((float)w*h))*100);
					system("cls");
					std::cout<<"Computing 3D Cloud "<<load<< "%";
				}
				
				

				cv::Point2f camPixelUD = Utilities::undistortPoints(cv::Point2f(i,j),camera); //camera 3d point p for (i,j) pixel
				
				cv::Point3f camPoint = Utilities::pixelToImageSpace(camPixelUD,camera); //convert camera pixel to image space
				
				cv::Vec3f camVector = (cv::Vec3f)  camPoint; //compute camera vector 

				cv::Point2f projPixel;
				projPixel.x = Utilities::matGet2D(decColsMatrix,i,j);
				projPixel.y = Utilities::matGet2D(decRowsMatrix,i,j);
				
				cv::Point3f projPoint = Utilities::pixelToImageSpace(projPixel,projector);
				proj2camSpace(projPoint);

				cv::Vec3f projVector;

				projVector = projPoint - projector.position;

				Utilities::normalize(projVector);
				Utilities::normalize(camVector);

				cv::Point3f interPoint;
				Utilities::line_lineIntersection(projector.position,projVector,camera.position,camVector,interPoint);

				//sampling rays
				if(raySampling_)
				{
					cv::Point3f interPointTmp;
					for(int i=0; i<SAMPLES_NUM; i++)
					{
						cv::Vec3f camVecTmp = camVector, projVecTmp = projVector;

						//compute camera vector 
						camVecTmp[0] += jitterCamRays[i].x;
						camVecTmp[1] += jitterCamRays[i].y;

						projVecTmp[0] += jitterProjRays[i].x;
						projVecTmp[1] += jitterProjRays[i].y;

						Utilities::line_lineIntersection(projector.position,projVecTmp,camera.position,camVecTmp,interPointTmp);

						interPoint += interPointTmp;

					}
				
					interPoint.x /= (SAMPLES_NUM+1.0);
					interPoint.y /= (SAMPLES_NUM+1.0);
					interPoint.z /= (SAMPLES_NUM+1.0);
				}

				points3DCamView->setPoint(i,j,interPoint);
				
				points3DProjView->addPoint(int (projPixel.x),int (projPixel.y),interPoint);
			
				//set decode imgs
				Utilities::matSet2D(decRows,i,j, projPixel.y/projector.width  * 255);
				Utilities::matSet2D(decCols,i,j, projPixel.x/projector.height * 255);
			}
		}
	}
	
	out1.close();
	system("cls");
	std::cout<<"Computing 3D Cloud  100%\n";
		
}