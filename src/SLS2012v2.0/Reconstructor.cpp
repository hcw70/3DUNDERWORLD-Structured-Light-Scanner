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
	projector = NULL;
	camera = NULL;
	col_gray_offset=0;
	row_gray_offset=0;

	pathSet=false;

	camImgs = NULL;
	mask = NULL;
	maskImg = NULL;
	decRows = NULL;
	decCols = NULL;
	colorImg = NULL;
	
	points3DCamView = NULL;
	points3DProjView = NULL;

	projColPlanesNormals = NULL;
	projRowPlanesNormals = NULL;

	decColsMatrix = NULL;
	decRowsMatrix = NULL;

	autoContrast_ = false;
	saveAutoContrast_ = false;

}


Reconstructor::~Reconstructor(void)
{
	if(projector)
		delete projector;
	
	if(camera)
		delete camera;

	unloadCamImgs();

	if(mask)
		delete mask;
	
	cvReleaseImage( &maskImg );
	cvReleaseImage( &decRows );
	cvReleaseImage( &decCols );
	cvReleaseImage( &colorImg );
	
	if(points3DCamView)
		delete points3DCamView ;
	if(points3DProjView)
		delete points3DProjView ;

	if(projColPlanesNormals)
		delete projColPlanesNormals;
	
	if(projRowPlanesNormals)
		delete projRowPlanesNormals;

	if(decColsMatrix)
		delete decColsMatrix;

	if(decRowsMatrix)
		delete decRowsMatrix;
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
	int w=camera->width;
	int h=camera->height;

	if(!decColsMatrix)
		decColsMatrix = new float[w*h];

	if(!decRowsMatrix)
		decRowsMatrix = new float[w*h];

	CvScalar projPixel;
	
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if(mask[i+j*w])
			{
				bool error=false;

				//get the projector pixel for camera (i,j) pixel
				projPixel = getProjPixelForCamPixel(i,j,&error);

				if(error)
				{
					mask[i+j*w]=false;
					continue;
				}

				decColsMatrix[access(i,j,h)] = projPixel.val[0];
				decRowsMatrix[access(i,j,h)] = projPixel.val[1];
				
			}
		}

	}

}

void Reconstructor::projectorViewImage()
{

	int w=camera->width;
	int h=camera->height;

	cv::Mat *projViewImg = new cv::Mat(projector->height,projector->width,CV_8UC3, cv::Scalar::all(0));
	cv::Mat *numOfVals = new cv::Mat(projector->height,projector->width,CV_8UC1, cv::Scalar::all(0));
		 
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if(mask[i+j*w])
			{
				int x,y;

				x = decColsMatrix[access(i,j,h)];
				y = decRowsMatrix[access(i,j,h)];

				if(x>=projector->width || y>=projector->height)
					continue;

				CvScalar c = cvGet2D(colorImg,j,i);
				
				projViewImg->at<cv::Point3_<uchar>>(x,y).x +=  c.val[0];
				projViewImg->at<cv::Point3_<uchar>>(x,y).y +=  c.val[1];
				projViewImg->at<cv::Point3_<uchar>>(x,y).z +=  c.val[2];

				numOfVals->at<uchar>(x,y) ++;
			}
		}
	}

	for(int i=0; i<projector->width; i++)
	{
		for(int j=0; j<projector->height; j++)
		{
			
			//if the pixel is not shadow reconstruct
			if(numOfVals->at<uchar>(j,i))
			{
			
				projViewImg->at<cv::Point3_<uchar>>(j,i).x = (uchar) projViewImg->at<cv::Point3_<uchar>>(j,i).x / numOfVals->at<uchar>(j,i);
				projViewImg->at<cv::Point3_<uchar>>(j,i).y = (uchar) projViewImg->at<cv::Point3_<uchar>>(j,i).y / numOfVals->at<uchar>(j,i);
				projViewImg->at<cv::Point3_<uchar>>(j,i).z = (uchar) projViewImg->at<cv::Point3_<uchar>>(j,i).z / numOfVals->at<uchar>(j,i);
				
			}
		}
	}

	cv::imwrite("projectorView.png",*projViewImg);
}


void Reconstructor::smoothDecode()
{

	int w=camera->width;
	int h=camera->height;

	CvScalar projPixel;
	
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if(mask[i+j*w])
			{

				int p = (int) decRowsMatrix[access(i,j,h)];

				int ii=1;
				

				while(p == decRowsMatrix[access(i,j+ii,h)] )
				{
					ii++;
				}

				if(ii>1)
				{
					float step = 1.0/ (float) ii;
					for(int m=0; m<ii; m++)
					{
							decRowsMatrix[access(i,j+m,h)] = p - 0.5 + m*step;
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
			if(mask[i+j*w])
			{

				int p = (int) decColsMatrix[access(i,j,h)];

				int ii=1;

				while(p == decColsMatrix[access(i + ii,j,h)] )
				{
					ii++;
				}

				if(ii>1)
				{
					float step = 1.0/ (float) ii;

					for(int m=0; m<ii; m++)
					{
							decColsMatrix[access(i+m,j,h)] = p -0.5 + m*step;
					}

				}

				i = i + ii -1;
			}
		}
	}


}

void Reconstructor::loadProjectorAndCamera()
{

	camera = new VirtualCamera();

	camera->loadCameraMatrix("input/cam_matrix.txt");
	camera->loadDistortion("input/cam_distortion.txt");

	camera->height=0;
	camera->width =0;

	projector = new VirtualCamera();

	projector->loadCameraMatrix("input/proj_matrix.txt");
	projector->loadDistortion("input/cam_distortion.txt");
	projector->loadRotationMatrix("input/proj_rotation_matrix.txt");
	projector->loadTranslationVector("input/proj_trans_vectror.txt");

	projector->height = proj_h;
	projector->width  = proj_w;

}

//load camera images
void Reconstructor::loadCamImgs()
{
	camImgs = new IplImage* [numberOfImgs];

	IplImage* tmp;

	std::cout<<"Loading Camera Images...";

	for(int i=0; i<numberOfImgs;i++)
	{
		std::stringstream path;

		path<<filePath1st.str();
		if(i+1<10)
			path<<"0";
		path<<i+1<<filePath2nd.str()<<fileExtension.str();
		
		tmp = NULL;

		tmp=cvLoadImage(path.str().c_str() );

		if(!tmp)
		{
			std::cout<<"\nError loading cam image "<<i+1<<". Press Any Key to Exit.";
			getch();
			exit(-1);
		}

		if(i==0)
		{
			colorImg=cvLoadImage(path.str().c_str() );

			//auto contrast
			if(autoContrast)
			{
				Utilities::autoContrast(colorImg,colorImg);
				
			}
		}


		camImgs[i]= cvCreateImage(cvSize(tmp->width, tmp->height), IPL_DEPTH_8U,  1);
		
		//auto contrast
		if(autoContrast)
		{
			Utilities::autoContrast(tmp,tmp);
			if(saveAutoContrast)
			{
				std::stringstream p;
				p<<filePath1st.str()<<"AutoContrastSave/"<<i+1<<filePath2nd.str()<<fileExtension.str();

				cvSaveImage(p.str().c_str(),tmp);
			}
		}
		cvCvtColor(tmp, camImgs[i], CV_BGR2GRAY);

		cvReleaseImage(&tmp);
	}

	if(camera->width==0)
	{
		camera->height=camImgs[0]->height;
		camera->width =camImgs[0]->width;
	}

	std::cout<<"done!\n";
	
}

//unload camera images
void Reconstructor::unloadCamImgs()
{
	

	if(camImgs)

	for(int i=0; i<numberOfImgs;i++)
	{
		cvReleaseImage( &camImgs[i]);
	}

	camImgs=NULL;
	
}

void Reconstructor::findProjectorCenter()
{
	
	camera->position=cvScalar(0,0);
	

	CvMat *Tmp=cvCreateMat(3,1,CV_32FC1);
	CvMat *Tptmp=cvCreateMat(3,1,CV_32FC1);

	cvSet1D(Tptmp,0,cvScalar(-cvGet1D(projector->translationVector,0).val[0]));
	cvSet1D(Tptmp,1,cvScalar(-cvGet1D(projector->translationVector,1).val[0]));
	cvSet1D(Tptmp,2,cvScalar(-cvGet1D(projector->translationVector,2).val[0]));

	cvSolve(projector->rotationMatrix,Tptmp,Tmp);
	
	//projector center
	CvScalar projCenter;
	projCenter.val[0]= cvGet1D(Tmp,0).val[0];
	projCenter.val[1]= cvGet1D(Tmp,1).val[0];
	projCenter.val[2]= cvGet1D(Tmp,2).val[0];

	projector->position=projCenter;

	cvReleaseMat(&Tmp);
	cvReleaseMat(&Tptmp);
	
}

void Reconstructor::computeShadows()
{
	std::cout<<"Estimating Shadows...";

	int w=camera->width;
	int h=camera->height;

	mask = new bool [w*h];

	maskImg = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U,  3);

	for(int i=0;i<w;i++)
	{
		for(int j=0; j<h; j++)
		{
			double blackVal,whiteVal;
			
			CvScalar black = cvGet2D(camImgs[1],j,i);
			CvScalar white = cvGet2D(camImgs[0],j,i);

			blackVal= black.val[0];
			whiteVal= white.val[0];

			if(whiteVal-blackVal > blackThreshold)
			{
				mask[i+j*w]=true;
				cvSet2D(maskImg,j,i, cvScalar(255,255,255));
			}
			else
			{
				mask[i+j*w]=false;
				cvSet2D(maskImg,j,i, cvScalar(0,0,0));
			}
		}
	}

	std::cout<<"done!\n";
}

void Reconstructor::computeProjPlanesNormals(double normals[], int numberOfplanes, int planeSize, bool rowFlag)
{
	
	double p1[3],p2[3],v1[3],v2[3],n[3];

	//compute row planes 
	for(int i=0; i<numberOfplanes; i++)
	{
		if(rowFlag)
		{
			p1[0]=0;
			p1[1]=i;

			p2[0]=planeSize;
			p2[1]=i;
		}
		else
		{
			p1[0]=i;
			p1[1]=0;

			p2[0]=i;
			p2[1]=planeSize;
		}

		double xUD,yUD;
		Utilities::undistortPoints(p1[0],p1[1],projector,&xUD,&yUD);

		p1[0]=xUD;
		p1[1]=yUD;

		Utilities::undistortPoints(p2[0],p2[1],projector,&xUD,&yUD);

		p2[0]=xUD;
		p2[1]=yUD;

		Utilities::pixelToImageSpace(p1,projector->fc,projector->cc);
		Utilities::pixelToImageSpace(p2,projector->fc,projector->cc);

		proj2camSpace(p1);
		proj2camSpace(p2);

		v1[0]=projector->position.val[0]-p1[0];
		v1[1]=projector->position.val[1]-p1[1];
		v1[2]=projector->position.val[2]-p1[2];

		Utilities::normalize3dtable(v1);

		v2[0]=projector->position.val[0]-p2[0];
		v2[1]=projector->position.val[1]-p2[1];
		v2[2]=projector->position.val[2]-p2[2];

		Utilities::normalize3dtable(v2);
		
		n[0]=v1[1]*v2[2] - v1[2]*v2[1];
		n[1]=v1[2]*v2[0] - v1[0]*v2[2];
		n[2]=v1[0]*v2[1] - v1[1]*v2[0];

		Utilities::normalize3dtable(n);

		normals[0+i*3]=n[0];
		normals[1+i*3]=n[1];
		normals[2+i*3]=n[2];

	}

}

void Reconstructor::computeProjPlaneNormal(double out[3], float numberOfplane, int planeSize, bool rowFlag)
{
	
	double p1[3],p2[3],v1[3],v2[3],n[3];

	//compute row planes 
	float i = numberOfplane;
	{
		if(rowFlag)
		{
			p1[0]=0;
			p1[1]=i;

			p2[0]=planeSize;
			p2[1]=i;
		}
		else
		{
			p1[0]=i;
			p1[1]=0;

			p2[0]=i;
			p2[1]=planeSize;
		}

		double xUD,yUD;
		Utilities::undistortPoints(p1[0],p1[1],projector,&xUD,&yUD);

		p1[0]=xUD;
		p1[1]=yUD;

		Utilities::undistortPoints(p2[0],p2[1],projector,&xUD,&yUD);

		p2[0]=xUD;
		p2[1]=yUD;

		Utilities::pixelToImageSpace(p1,projector->fc,projector->cc);
		Utilities::pixelToImageSpace(p2,projector->fc,projector->cc);

		proj2camSpace(p1);
		proj2camSpace(p2);

		v1[0]=projector->position.val[0]-p1[0];
		v1[1]=projector->position.val[1]-p1[1];
		v1[2]=projector->position.val[2]-p1[2];

		Utilities::normalize3dtable(v1);

		v2[0]=projector->position.val[0]-p2[0];
		v2[1]=projector->position.val[1]-p2[1];
		v2[2]=projector->position.val[2]-p2[2];

		Utilities::normalize3dtable(v2);
		
		n[0]=v1[1]*v2[2] - v1[2]*v2[1];
		n[1]=v1[2]*v2[0] - v1[0]*v2[2];
		n[2]=v1[0]*v2[1] - v1[1]*v2[0];

		Utilities::normalize3dtable(n);

		out[0]=n[0];
		out[1]=n[1];
		out[2]=n[2];

	}

}

void Reconstructor::runReconstruction()
{

	if(projector==NULL || camera==NULL)
	{
		std::cout<<"camera or projector not initialized";
		return;
	}

	if(pathSet==false)
	{
		std::cout<<"imgs path not set";
		return;
	}

	GrayCodes *grays = new GrayCodes(projector->width,projector->height);

	numOfColBits = grays->getNumOfColBits();
	numOfRowBits = grays->getNumOfRowBits();
	numberOfImgs = grays->getNumOfImgs();

	delete grays;

	loadCamImgs();

	points3DCamView  = new PointCloudImage( camera->width    , camera->height    , false ); 
	points3DProjView = new PointCloudImage( projector->width , projector->height , false );

	findProjectorCenter();
	computeShadows();

	if(projColPlanesNormals == NULL)
		projColPlanesNormals = new double[projector->width*3];

	computeProjPlanesNormals(projColPlanesNormals,projector->width,projector->height,true);

	if(projRowPlanesNormals == NULL)
		projRowPlanesNormals = new double[projector->height*3];

	computeProjPlanesNormals(projRowPlanesNormals,projector->height,projector->width,true);
	
	std::cout<<"Decoding paterns...";
	decodePaterns();
	std::cout<<"done!\n";
	camProjPixelsTriangulation();

	unloadCamImgs();

	delete projColPlanesNormals;
	projColPlanesNormals = NULL;

	delete projRowPlanesNormals;
	projRowPlanesNormals = NULL;

}

//convert a point from projector to camera space
void Reconstructor::proj2camSpace(double p[3])
{
	
	CvMat *Tmp=cvCreateMat(3,1,CV_32FC1);
	CvMat *tmpPoint=cvCreateMat(3,1,CV_32FC1);

	tmpPoint->data.fl[0]=p[0];
	tmpPoint->data.fl[1]=p[1];
	tmpPoint->data.fl[2]=p[2];

	cvGEMM(projector->rotationMatrix, projector->translationVector, -1, NULL, 0, Tmp,CV_GEMM_A_T); 
	cvGEMM(projector->rotationMatrix, tmpPoint, 1, NULL, 0, tmpPoint,CV_GEMM_A_T);

	Tmp->data.fl[0]=cvGet1D(Tmp,0).val[0]+cvGet1D(tmpPoint,0).val[0];
	Tmp->data.fl[1]=cvGet1D(Tmp,1).val[0]+cvGet1D(tmpPoint,1).val[0];
	Tmp->data.fl[2]=cvGet1D(Tmp,2).val[0]+cvGet1D(tmpPoint,2).val[0];

	p[0]=Tmp->data.fl[0];
	p[1]=Tmp->data.fl[1];
	p[2]=Tmp->data.fl[2];
	
	cvReleaseMat(&Tmp);
	cvReleaseMat(&tmpPoint);
	
}



//for a (x,y) pixel of the camera returns the corresponding projector pixel
CvScalar Reconstructor::getProjPixelForCamPixel(int x, int y, bool *error)
{
	bool *grayCol = new bool[numOfColBits];
	bool *grayRow = new bool[numOfRowBits];

	CvScalar pixel;
	double val1, val2;
	int xDec,yDec;

	int error_code =0;

	//prosses column images
	for(int count=0; count<numOfColBits; count++)
	{

		pixel = cvGet2D(camImgs[count*2+2],y,x);
		val1=pixel.val[0];

		pixel = cvGet2D(camImgs[count*2+2+1],y,x);
		val2=pixel.val[0];
		
		if(abs(val1-val2)<whiteThreshold && error_code<numOfColBits-count )
			error_code=numOfColBits-count+1;

		if(val1>val2)
			grayCol[count]=1;
		else
			grayCol[count]=0;

	}

	xDec=GrayCodes::grayToDec(grayCol,numOfColBits) - col_gray_offset;


	if(xDec>projector->width && error_code==0)
	{
		error_code = 1;
	}

	//prosses row images
	for(int count=0; count<numOfRowBits; count++)
	{

		pixel = cvGet2D(camImgs[count*2+2+numOfColBits*2],y,x);
		val1=pixel.val[0];

		pixel = cvGet2D(camImgs[count*2+2+numOfColBits*2+1],y,x);
		val2=pixel.val[0];
		
		if(abs(val1-val2)<whiteThreshold && error_code<numOfRowBits-count )
			error_code=numOfRowBits-count+1;

		if(val1>val2)
			grayRow[count] = 1;
		else
			grayRow[count] = 0;

	}

	yDec = GrayCodes::grayToDec(grayRow,numOfRowBits) - row_gray_offset; //-128 is the datasets offset for rows pls remove

	if(yDec > projector->height && error_code==0)
	{
		error_code==1;	
	}

	if(error_code!=0)
	{
		*error=true;
		if(error_code ==1)
			cvSet2D(maskImg,y,x, cvScalar(0,255,0));
		else
			cvSet2D(maskImg,y,x, cvScalar(0,0,100+155*(float (error_code)/float (numOfRowBits))));
	}
	else
		*error=false;

	delete grayCol;
	delete grayRow;

	return cvScalar(xDec,yDec);
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
	if(!maskImg)
		return;

	cvSaveImage(path,maskImg);

}

void Reconstructor::saveDecodedRowImg(char path[])
{
	if(!decRows)
		return;

	cvSaveImage(path,decRows);
}

void Reconstructor::saveDecodedColImg(char path[])
{
	if(!decCols)
		return;

	cvSaveImage(path,decCols);
}

void Reconstructor::camProjPixelsTriangulation()
{
	
	int w = camera->width;
	int h = camera->height;

	//points3DCamView = new PointCloudImage(camera->width,camera->height,true); 

	decRows = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U,  1);
	decCols = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U,  1);
	
	CvScalar projPixel;
	CvScalar projPlaneNormR;
	CvScalar projPlaneNormC;
	CvScalar camVector;
	CvScalar pCol;
	CvScalar pRow;


	CvScalar p;
	CvScalar color;

	double x1,x2,y1,y2,rangeX,rangeY;

	{
		Utilities::undistortPoints(0,0,camera,&x1,&y1);
		Utilities::undistortPoints(w,h,camera,&x2,&y2);

		double p1[3],p2[3];
		p1[0] = x1;
		p1[1] = y1;
		p1[2] = 1;

		p2[0] = x2;
		p2[1] = y2;
		p2[2] = 1;

		Utilities::pixelToImageSpace(p1,camera->fc,camera->cc);
		Utilities::pixelToImageSpace(p2,camera->fc,camera->cc);

		if(p1[0]>p2[0])
			rangeX=p1[0]-p2[0];
		else
			rangeX=p2[0]-p1[0];

		if(p1[1]>p2[1])
			rangeY=p1[1]-p2[1];
		else
			rangeY=p2[1]-p1[1];

	}
	
	//generade Sample values
	cv::Point2d jitterMatrix[SAMPLES_NUM];
	Sample::Jitter(jitterMatrix,SAMPLES_NUM);

	for(int i=0; i<SAMPLES_NUM; i++)
	{
		jitterMatrix[i].x = jitterMatrix[i].x*rangeX / (double)w;
		jitterMatrix[i].y = jitterMatrix[i].y*rangeY / (double)h;
	}

	int load=0;

	//reconstraction for every camera pixel
	for(int i=0; i<w; i++)
	{
		for(int j=0; j<h; j++)
		{

			//if the pixel is not shadow reconstruct
			if(mask[i+j*w])
			{

				if(load != (int) (((j+(float)i*h)/((float)w*h))*100))
				{
					load =  (int) (((j+(float)i*h)/((float)w*h))*100);
					system("cls");
					std::cout<<"Computing 3D Cloud "<<load<< "%";
				}

				//get pixel color
				color = cvGet2D(colorImg,j,i);

				projPixel.val[0] = decColsMatrix[access(i,j,h)];
				projPixel.val[1] = decRowsMatrix[access(i,j,h)];

				//camera 3d point p for (i,j) pixel
				double xUD,yUD;
				Utilities::undistortPoints(i,j,camera,&xUD,&yUD);

				double pCam[3];
				pCam[0]=xUD;
				pCam[1]=yUD;

				//convert camera pixel to image space
				Utilities::pixelToImageSpace(pCam,camera->fc,camera->cc);

				//compute camera vector 
				camVector.val[0]=-pCam[0];
				camVector.val[1]=-pCam[1];
				camVector.val[2]=-pCam[2];

				double norm[3];

				computeProjPlaneNormal(norm,projPixel.val[0],projector->height, false);

				//ray - col plane intersection 
				projPlaneNormC.val[0] = norm[0];
				projPlaneNormC.val[1] = norm[1];
				projPlaneNormC.val[2] = norm[2];

				pCol = Utilities::planeRayInter(projPlaneNormC, projector->position, camVector, camera->position);// LinePlaneInterPoint(cvScalar(0,0) ,camVector, projColPlanes, projPixel.val[0],4);
				
				computeProjPlaneNormal(norm ,projPixel.val[1] ,projector->width ,true);

				//ray - row plane intersection
				projPlaneNormR.val[0] = norm[0];
				projPlaneNormR.val[1] = norm[1];
				projPlaneNormR.val[2] = norm[2];

				pRow = Utilities::planeRayInter(projPlaneNormR, projector->position, camVector, camera->position);//LinePlaneInterPoint(cvScalar(0,0) ,camVector, projRowPlanes, projPixel.val[1],4);

				//sampling rays
				if(raySampling_)
				{

					CvScalar tmp;

					for(int i=0; i<SAMPLES_NUM; i++)
					{

						//compute camera vector 
						camVector.val[0]=-pCam[0]+jitterMatrix[i].x;
						camVector.val[1]=-pCam[1]+jitterMatrix[i].y;
						camVector.val[2]=-pCam[2];

						tmp = Utilities::planeRayInter(projPlaneNormC, projector->position, camVector, camera->position);

						pCol.val[0]+=tmp.val[0];
						pCol.val[1]+=tmp.val[1];
						pCol.val[2]+=tmp.val[2];

						tmp = Utilities::planeRayInter(projPlaneNormR, projector->position, camVector, camera->position);

						pRow.val[0]+=tmp.val[0];
						pRow.val[1]+=tmp.val[1];
						pRow.val[2]+=tmp.val[2];

					}
				
					pRow.val[0]=pRow.val[0]/(SAMPLES_NUM+1.0);
					pRow.val[1]=pRow.val[1]/(SAMPLES_NUM+1.0);
					pRow.val[2]=pRow.val[2]/(SAMPLES_NUM+1.0);

					pCol.val[0]=pCol.val[0]/(SAMPLES_NUM+1.0);
					pCol.val[1]=pCol.val[1]/(SAMPLES_NUM+1.0);
					pCol.val[2]=pCol.val[2]/(SAMPLES_NUM+1.0);

				}

				//find the average point
				p.val[0]=(pCol.val[0]+pRow.val[0])/2.0;
				p.val[1]=(pCol.val[1]+pRow.val[1])/2.0;
				p.val[2]=(pCol.val[2]+pRow.val[2])/2.0;

				points3DCamView->setPoint(i,j,p,color);
				
				points3DProjView->addPoint(int (projPixel.val[0]),int (projPixel.val[1]),p);
			
				//set decode imgs
				cvSet2D(decRows,j,i,cvScalar(projPixel.val[1]/projector->width  * 255));
				cvSet2D(decCols,j,i,cvScalar(projPixel.val[0]/projector->height * 255));
				
			}
		}

	}
	

	system("cls");
	std::cout<<"Computing 3D Cloud  100%\n";
		
}