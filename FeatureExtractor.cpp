#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>


#include <stdlib.h>
#include <stdio.h>


using namespace std;
using namespace cv;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;
Mat Blur;

vector<vector<Point> > contours;
vector<vector<Point> > contours1;
vector<Vec4i> hierarchy;
vector<Vec4i> hierarchy1;

int edgeThresh = 1;
int lowThreshold=100;
int const max_lowThreshold = 200;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";
char*  window_blur= "blur map";

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */

void FindingAngle(RotatedRect minRect){
	if(minRect.size.width < minRect.size.height){
        printf("oggetto ruotato di :%7.2f gradi rispetto alla verticale\n\n", minRect.angle+180);
    }else{
        printf("oggetto ruotato di :%7.2f gradi rispetto alla verticale\n\n", minRect.angle+90);
    }

}

Scalar FindingMeanColor(Mat img,vector<vector<Point> > contours,int idx){
	Scalar color( 254,254,254 );
Mat imgcopy;
img.copyTo(imgcopy);

Mat Zeros=Mat::zeros(imgcopy.size(), CV_8UC1);
drawContours(Zeros, contours, idx, color, CV_FILLED, 8, hierarchy,0 );
Scalar meanColor=mean(imgcopy,Zeros);
return meanColor;

}

float findHeight(vector<vector<Point> > contours,int idx){
	Point max;
	max.y=0;
	for(int i=0;i<contours[idx].size();i++){
		if(contours[idx][i].y>max.y)
			max=contours[idx][i];}
	
	Point min;
	min.y=max.y;
	for(int i =0;i<contours[idx].size();i++){
		if(contours[idx][i].y<min.y)
			min=contours[idx][i];}
	float f=(abs(max.x - min.x))^2;
	float g=(abs(max.y - min.y))^2;
	float heigth=f+g;
	return heigth;

}

double findWidth(vector<vector<Point> > contours,int idx){
	Point max;
	max=contours[idx][0];
	for(int i=0;i<contours[idx].size();i++){
		if(contours[idx][i].x>max.x)
			max=contours[idx][i];}
	
	Point min;
	min=contours[idx][0];
	for(int i =0;i<contours[idx].size();i++){
		if(contours[idx][i].x<min.x)
			min=contours[idx][i];}
	double f=(abs(max.x - min.x))^2;
	double g=(abs(max.y - min.y))^2;
	double width=sqrt(f+g);
	return width;

}

int FindConvexities(vector<vector<Point> > contours,int idx,Mat Draw){
	vector<int> convexHull_IntIdx;
		int convexieties=0;
		convexHull(contours[idx], convexHull_IntIdx, false);
		vector<Vec4i> convexityDefectsSet; 
		convexityDefects(contours[idx], convexHull_IntIdx, convexityDefectsSet);
		
		for (int cDefIt = 0; cDefIt < convexityDefectsSet.size(); cDefIt++)
			{
			int startIdx = convexityDefectsSet[cDefIt].val[0];
			int endIdx = convexityDefectsSet[cDefIt].val[1];
			int defectPtIdx = convexityDefectsSet[cDefIt].val[2];
			double depth = (double)convexityDefectsSet[cDefIt].val[3]/256.0f;
			//if(sqrt((abs(contours[idx][endIdx].x - contours[idx][startIdx].x)) ^ 2 + (abs(contours[idx][endIdx].y - contours[idx][startIdx].y)) ^ 2))>=100))
			double f=(abs(contours[idx][endIdx].x - contours[idx][startIdx].x))^2;
			double g=(abs(contours[idx][endIdx].y - contours[idx][startIdx].y))^2;
			if(
				sqrt(
					f+g
				)>=8 && depth>=8
				){
					convexieties++;
					circle(Draw,  contours[idx][defectPtIdx], 10, Scalar(0, 0, 255), CV_FILLED, CV_AA);}
			
			}
		return convexieties;

}

void Measurings(vector<vector<Point> > contours,int idx,Mat& image,Scalar color){
	RotatedRect minRect;
		Point2f rect_points[4];
		minRect=minAreaRect(contours[idx]);
		float Area= contourArea(contours[idx]);
		float Perimeter=arcLength(contours[idx],1);
		cout<<"Area: "<<Area<<"\tPerimetro: "<<Perimeter<<"\n\n";
		
		FindingAngle( minRect);

		cout<<"lunghezza = "<<minRect.size.width<<"\t"<< findWidth(contours,idx) << "\t altezza= "<<minRect.size.height<<"\t"<< findHeight(contours,idx)<<"\n\n";
		float aspect_ratio = float (minRect.size.width)/ float(minRect.size.height);
		cout<<"Rapporto larghezza / altezza = "<< aspect_ratio<<"\n\n";
		minRect.points(rect_points);
		for( int j = 0; j < 4; j++ ){
          line( image, rect_points[j], rect_points[(j+1)%4], color, 10, 8 );
			}
}

// Better use findholes1
int findHoles(vector<Vec4i> hierarchy,int idx,int alFoundHoles){ // dobbiamo trovare i nipoti
	int holes= alFoundHoles;
	if(hierarchy[idx][2]!=-1){ //verifichiamo che abbia figli
		
			int d= hierarchy[idx][2]; 
			if(hierarchy[d][2]>=0){	  // se il figlio ha altri figli; in altre parole se c'è un nipote del contrno iniziale
				int x =hierarchy[d][2]; //prendiamo questo figlio alis il nipote
				holes++;
				if(hierarchy[x][0]==-1 && hierarchy[x][2]==-1) // se è figlio unico e non ha figli c'è un solo nipote
					;
				else if(hierarchy[x][0]>=0 &&hierarchy[x][2]!=-1) // se ha un fratello e non ha figli si aggiunge 2 più i nipoti del fratello
					holes=holes+1+findHoles(hierarchy,hierarchy[x][0],0);
				else if(hierarchy[x][0]==-1 &&hierarchy[x][2]>=0) // se NON ha un fratello ma ha un figlio si aggiunge 1 più i nipoti del figlio
					holes=holes +findHoles(hierarchy,hierarchy[x][2],0);
				else if(hierarchy[x][0]>=0 &&hierarchy[x][2]>=0) //se ha un fratello e un figlio si aggiunge 2 più i nipoti del fratello e del figlio
					holes=holes+1+findHoles(hierarchy,hierarchy[x][0],0)+findHoles(hierarchy,hierarchy[x][2],0);
				}
			
			
			}
	return holes;

}
int findHoles1(vector<Vec4i> hierarchy,int idx){
	int holes=0;
	if(hierarchy[idx][2]!=-1){ //verifichiamo che abbia figli
		
			int d= hierarchy[idx][2]; 
			if(hierarchy[d][2]>=0){
				holes++;
				int x =hierarchy[d][2]; //prendiamo questo figlio alis il nipote
				if(hierarchy[x][0]==-1)
					holes++;
			}
		}
	return holes;
}

void CannyThreshold(int,void*)
{
  /// Reduce noise with a kernel 3x3
  GaussianBlur(src_gray, detected_edges, Size(31,31), 1.5,1.5);
 blur( detected_edges, detected_edges, Size(3,3) );
  detected_edges.copyTo(Blur);
  

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold* 3, kernel_size );
  
   
  dilate(detected_edges, detected_edges,getStructuringElement( MORPH_RECT,Size(10,10)));
  erode(detected_edges, detected_edges,getStructuringElement( MORPH_RECT,Size(7,7)));
 
  
  
  findContours(detected_edges.clone(),  contours,hierarchy,  RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0); // it makes dst a matrix full of 0s (black)
  
 
  Mat srcCopy;
  src.copyTo(srcCopy);
  contours1=contours;
  hierarchy1=hierarchy;
  int idx = 0;
  int count=1;
  for( ; idx>=0; idx =hierarchy[idx] [0])
	{
		if( hierarchy[idx][3]==-1 && contourArea(contours[idx])>=9000){

		cout<<"oggetto "<<count<<":\n\n";
		count++;
		
        Scalar color( rand()&255, rand()&255, rand()&255 );
		drawContours( srcCopy, contours, idx, color, CV_FILLED, 8, hierarchy,3 );
		
		Measurings(contours,idx,srcCopy,color);

		int convexityDefects=FindConvexities(contours, idx,srcCopy);
		cout<<"sono presenti "<<convexityDefects<<"convessita\n\n";

		
		Scalar meancolor=FindingMeanColor(src, contours, idx);
		cout<<"colore medio: "<<meancolor[0]<<" "<<meancolor[1]<<" "<<meancolor[2]<<"  \n\n";
		
		
		
		int holes=findHoles( hierarchy, idx,0);
		//int holes=findHoles1( hierarchy, idx);
		
		
		int d=idx;
		/*if(hierarchy[idx][2]!=-1)
			for(int i=hierarchy[idx][0];i!=-1;i=hierarchy[d][0]){
				if(isContourConvex(contours[hierarchy[d][0]])&& contourArea(contours[hierarchy[d][0]])>=contourArea(contours[idx])/100000){
					holes++;
					drawContours( srcCopy, contours, d, Scalar(255,0,0), CV_FILLED,8 , hierarchy,10 );
					
				}d=hierarchy[d][0];
			}*/
		

		cout<<"contiene "<<holes<<" contorni interni \n";
		printf("---\n\n");

		}//end  if(hierarchy[idx][3]==-1)

    }//end for idx
  
  cout<<"\n\n";

  src.copyTo( dst, detected_edges);
  imshow(window_blur,srcCopy);
  imshow( window_name, dst );
 }


/** @function main */
int main( int argc, char** argv )
{
	
  /// Load an image
  src = imread( "Path/To/Img" );

  if( !src.data )
  { return -1; }

  /// Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  /// Convert the image to grayscale
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window
  namedWindow( window_name,CV_WINDOW_NORMAL); // CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO
  namedWindow(window_blur,CV_WINDOW_NORMAL);

  /// Create a Trackbar for user to enter threshold
  createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

  /// Show the image
  CannyThreshold(0,0);

  /// Wait until user exit program by pressing a key
  waitKey(0);

  return 0;
  }