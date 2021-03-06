
//============================================================================
// Name        : simple object detection
// Author      : alfred
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <ros/ros.h>
#include "objdet.h"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string.h>

#include <cv.h>
#include <highgui.h>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/core/core.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <getopt.h>
#include <time.h>

using namespace cv;
using namespace std;


// *************************
// Recognition parameters
// *************************
static const int minHessian = 300;
static const int knn = 3;

// Publisher for object detection
static ros::Publisher obj_pub;
// Publisher for speaker
static ros::Publisher talk_pub;

static image_transport::Subscriber obj_sub;

// Debug flag ** future work
bool verbose_(true);
bool HSV_(false);
int have_gui=1;

// Option structure for arguments
static const struct option long_options[] = {
		{"gui",            no_argument, &have_gui,   1},
		{"object",   required_argument,         0, 'o'},
		{"testimg",  required_argument,         0, 't'},
		{0, 0, 0, 0}
};

// Obj Description structure
typedef struct {
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	int w,h;
	cv::Mat image;
} objDescr_t;

 //ObjDescr for one object
objDescr_t object;

// To sort the distance table
bool compareDist(const cv::DMatch &a, const cv::DMatch &b)
{
  // smallest comes first
  double da = a.distance;
  double db = b.distance;
  return (da < db);
}

//TODO: take pictures from correct angle and distance, of a sphere (tomato)
//	get image from topic, convert from ros image to opencd Mat, send result to topic
//	(maybe also check colors)

/* could maybe use another method for confidence check,
	get the good points and check how far they are being moved by the homography 
	transformation, sum them up and divide by total good
	points, lesser value is better.
 */

// filter everything but red
cv::Mat redFilter(const cv::Mat& src) {

  assert(src.type() == CV_8UC3);
  
  cv::Mat redOnly;
  cv::inRange(src, cv::Scalar(0, 0, 0), cv::Scalar(70, 255, 255), redOnly);
  
  return redOnly;
}

// Copyright 2000 softSurfer, 2012 Dan Sunday
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

// a Point is defined by its coordinates {int x, y;}
//===================================================================

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
inline int isLeft( cv::Point P0, cv::Point P1, cv::Point P2 )
{
  return ( (P1.x - P0.x) * (P2.y - P0.y)
	   - (P2.x -  P0.x) * (P1.y - P0.y) );
}

// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only when P is outside)
int wn_PnPoly( cv::Point P, std::vector<cv::Point2f> V, int n )
{
  int    wn = 0;    // the  winding number counter
  
  // loop through all edges of the polygon
  for (int i=0; i<n; i++) {   // edge from V[i] to  V[i+1]
    if (V[i].y <= P.y) {          // start y <= P.y
      if (V[i+1].y  > P.y)      // an upward crossing
	if (isLeft( V[i], V[i+1], P) < 0)  // P left of  edge
	  ++wn;            // have  a valid up intersect
    }
    else {                        // start y > P.y (no test needed)
      if (V[i+1].y  <= P.y)     // a downward crossing
	if (isLeft( V[i], V[i+1], P) > 0)  // P right of  edge
	  --wn;            // have  a valid down intersect
    }
  }
  return wn;
}
//===================================================================


// Object Detection called for every message received.
double DetectObject(const cv::Mat& scene_data) {
  if (verbose_) std::cerr<<"Detecting object...";
  timespec tstart, tend;
  clock_gettime(CLOCK_MONOTONIC, &tstart);
  
  double Kmatch = 2;//knn;
  
  /* masking, maybe not a good idea ( it was not, at least not when
   * only trying to find the tomato! ) */
  // filter color
  /*
    cv::Mat mask_scene = redFilter(scene_data);
    scene_data.copyTo(img_scene,mask_scene);
  */
  

//  if (HSV_)

    // Create a HSV picture of the input
    cv::Mat img_hsv(scene_data.rows, scene_data.cols, CV_8UC3);
    cv::cvtColor(scene_data,img_hsv,CV_RGB2HSV);
    int from_to[]={2,0};
    cv::Mat img_scene(img_hsv.rows, img_hsv.cols, CV_8UC1);
    cv::mixChannels(&img_hsv,1,&img_scene,1,from_to,1);


//    // Create a grayscale picture of the input
//    cv::Mat img_scene = cvCreateMat(scene_data.rows, scene_data.cols, CV_8UC1);
//    cvtColor(scene_data, img_scene,CV_BGR2GRAY);


  //Get the SURF features for the scene
  cv::SurfFeatureDetector detector (minHessian);
  std::vector<cv::KeyPoint> keypoints_scene;
  detector.detect( img_scene, keypoints_scene );
  
  // Time output
  clock_gettime(CLOCK_MONOTONIC, &tend);
  if (verbose_)std::cerr<<' '<<((tend.tv_nsec-tstart.tv_nsec)/1000L+(tend.tv_sec-tstart.tv_sec)*1000000L)
			<<" keypoints.";
  
  // calculate descriptors (Feature vectors)
  cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors_scene;
  extractor.compute (img_scene, keypoints_scene, descriptors_scene );
  
  // Time output
  clock_gettime(CLOCK_MONOTONIC, &tend);
  if (verbose_)std::cerr<<' '<<((tend.tv_nsec-tstart.tv_nsec)/1000L+(tend.tv_sec-tstart.tv_sec)*1000000L)
			<<" descriptors.";

  // Match using FLANN
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches;
  
  // ****************************
  // If we don't have desciptors - ERROR CHECK!
  // ***************************
  if (descriptors_scene.empty()){
    cout<<"Found no features in pic! :S"<<endl;
    return 0;
  }
  
  // Get the best match for each object descriptor on the scene.
  matcher.match ( object.descriptors, descriptors_scene, matches );
  
  // Time output
  clock_gettime(CLOCK_MONOTONIC, &tend);
  if (verbose_)std::cerr<<' '<<((tend.tv_nsec-tstart.tv_nsec)/1000L+(tend.tv_sec-tstart.tv_sec)*1000000L)
			<<" match.";
  
  // calculate min distance between keypoints
  
  double max_dist = 0; double min_dist = 100;
  int maxdist_i=0; int mindist_i=0;
  for( int i = 0; i < descriptors_scene.rows; i++ ) {
    double dist = matches[i].distance;
    if( dist < min_dist ){ min_dist = dist;mindist_i=i; }
    if( dist > max_dist ){ max_dist = dist;maxdist_i=i; }
  }
  
//   std::sort(matches.begin(), matches.end(), compareDist);
//   double min_dist = matches[0].distance;
  
  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );
  
  // draw only matches whose distance is less than 2*min_dist
  
  // Get the matches that are good according to the threshold.
  std::vector<cv::DMatch> good_matches;
  
  // Number of nearest neighbors.
  for( int i = 0; i < object.descriptors.rows; i++ ) {
    if( matches[i].distance <= Kmatch*min_dist ) {
      good_matches.push_back( matches[i]); }
  }

  //Draw the matches
  cv::Mat img_matches;
  if(have_gui) {
    drawMatches( object.image, object.keypoints, img_scene, keypoints_scene, \
		 good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), \
		 std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  }
  
  // this is for finding the object and putting a box around it
  //-- Localize the object
  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;
  
  // Get the coordenates from the good matches.
  for(size_t i = 0; i < good_matches.size(); i++ ) {
    obj.push_back( object.keypoints[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  
  // If there are not enough points (4) then we just exit.
  if (obj.size()<4){
    cout<<"Can't calculate homography"<<endl;
    return 0;
  }
  // Different method for percentage of correct features.
  vector<uchar> outputMask;
  
  cv::Mat H = cv::findHomography( obj, scene, CV_RANSAC,3, outputMask );
  int inlierCounter=0;
  for (int i =0; i<outputMask.size(); i++){
    if (outputMask[i] ==1)
      inlierCounter++;
  }
  
  // Time output
  clock_gettime(CLOCK_MONOTONIC, &tend);
  if (verbose_)std::cerr<<' '<<((tend.tv_nsec-tstart.tv_nsec)/1000L+(tend.tv_sec-tstart.tv_sec)*1000000L)
			<<" homography.";

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<cv::Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0);
  obj_corners[1] = cvPoint( object.w, 0 );
  obj_corners[2] = cvPoint( object.w, object.h );
  obj_corners[3] = cvPoint( 0, object.h );
  
  std::vector<cv::Point2f> scene_corners(4);
  
  
  cv::perspectiveTransform( obj_corners, scene_corners, H);
  // calculate some kind of confidence 
  // (amount of good points supporting the homography divided by amount of good points)
  double confident_points_polygon = 0;
  double confident_points_rectangle = 0;
  
  // needed for Point inside polygon function, pop back after
  scene_corners.push_back (scene_corners[0]);
  
  for(size_t i = 0; i < good_matches.size(); i++ ) {
    if (wn_PnPoly(keypoints_scene[ good_matches[i].trainIdx ].pt,scene_corners,4) != 0) {
      ++confident_points_polygon;
    }
  }
  
  scene_corners.pop_back();
  
  clock_gettime(CLOCK_MONOTONIC, &tend);
  if (verbose_)std::cerr<<' '<<((tend.tv_nsec-tstart.tv_nsec)/1000L+(tend.tv_sec-tstart.tv_sec)*1000000L)
			<<"µs, done.\n";
  
  // Show lines matching the identified region.
  if(have_gui) {
    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    cv::line( img_matches, scene_corners[0] + cv::Point2f( object.w, 0), \
	      scene_corners[1] + cv::Point2f( object.w, 0), cv::Scalar(0, 255, 0), 4 );
    cv::line( img_matches, scene_corners[1] + cv::Point2f( object.w, 0), \
	      scene_corners[2] + cv::Point2f( object.w, 0), cv::Scalar( 0, 255, 0), 4 );
    cv::line( img_matches, scene_corners[2] + cv::Point2f( object.w, 0), \
	      scene_corners[3] + cv::Point2f( object.w, 0), cv::Scalar( 0, 255, 0), 4 );
    cv::line( img_matches, scene_corners[3] + cv::Point2f( object.w, 0), \
	      scene_corners[0] + cv::Point2f( object.w, 0), cv::Scalar( 0, 255, 0), 4 );
    
    //-- Show detected matches
    cv::namedWindow("matches", CV_WINDOW_NORMAL);
    cv::imshow( "matches", img_matches );
  }
  
  //std::cerr <<"Rectangle: "<< double(confident_points_rectangle)/double(good_matches.size()) << std::endl;
  //std::cerr <<"Polygon: "<< double(confident_points_polygon)/double(good_matches.size()) << std::endl;
  double p=double(confident_points_polygon)/double(good_matches.size());
  double p2=double((float)inlierCounter/(float)outputMask.size());
  //if (verbose_)
   std::cout<<"p: "<<p<<" vs p2 "<<p2<<std::endl;
  return p;
  
}


void DetectObjectHandler(const sensor_msgs::ImageConstPtr &img) {
  // convert from ros message to opencv image
  cout<<"getting data ..."<<endl;
  cv_bridge::CvImageConstPtr cv_ptr;
  static int w=1;
  try
    {
      cv_ptr = cv_bridge::toCvShare(img, img->encoding);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  //cv::namedWindow("matches", CV_WINDOW_NORMAL);
  //cv::imshow( "matches", cv_ptr->image );
  
  // If we want to take the picture of the object
  if(w==0) {
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    cv::imwrite("/home/robo/object-images/pepper4.png",cv_ptr->image,compression_params);
    w=1;
  }

  static double p=0.0;
  static double alpha=1.0;
  p=alpha*DetectObject(cv_ptr->image)+(1.-alpha)*p;
  if (verbose_)std::cerr<<"Confidence: "<<p*100.<<"%\n";
  if(p>0.50) {
    //todo send message to stop the robot
    std_msgs::Empty nothing;
    obj_pub.publish(nothing);//cv_ptr->image);
    std::cout<<"Object detected: PEPPER\n";
    std_msgs::String something;
    something.data="Patrick, I found a pepper.";
    talk_pub.publish(something);
    //exit(0);
  }
}


//std::vector<objDescr_t> object;

int main(int argc, char** argv)
{
    const char *object_file=0, *scene_file=0;
    int option_index = 1;
    while(1) {
    	int c = getopt_long (argc, argv, "o:t:g",
    			long_options, &option_index);
    	if(c == -1) break;
    	switch (c) {
    	case 0:
//    		if (long_options[option_index].flag != 0) break;
    		break;
    	case 'o':
    		object_file=optarg;
    		break;
    	case 't':
    		scene_file=optarg;
    		break;
    	case 'g':
    		have_gui=1;
    		break;
    	case '?':
    		break;
    	default:
    		abort();
    		break;
    	}
    }

	ros::init(argc, argv, "objdetect");
	ros::NodeHandle nh;

	std::cerr<<"Generating object models...";
	timespec tstart, tend;
	clock_gettime(CLOCK_MONOTONIC, &tstart);
	{ 
	  //read reference file
	  cv::Mat img_in = cv::imread ( object_file,1);
	  //cv::Mat mask_object = redFilter(tmp_object);
	  // create a matrix with 8 bits x 3 color info per pixel
	  
	  //if (HSV_)

	    cv::Mat img_hsv(img_in.rows, img_in.cols, CV_8UC3);
	    // Convert the image from RGB --> HSV
	    cv::cvtColor(img_in,img_hsv,CV_RGB2HSV);

	    int from_to[]={2,0};
	    // Create a matrix 8 bits x 1 color info per pixel
	    cv::Mat img_object(img_hsv.rows, img_hsv.cols, CV_8UC1);

	    // Take only the hue value and copy it into the img_object.
	    cv::mixChannels(&img_hsv,1,&img_object,1,from_to,1);


//	    // Grayscale
//	    cv::Mat img_object = cvCreateMat(img_in.rows, img_in.cols, CV_8UC1);
//	    cvtColor(img_in, img_object,CV_BGR2GRAY);

	  
	  object.w=img_object.cols;
	  object.h=img_object.rows;
	  

	  //for (int i=0; i<3;i++){
		  //std::cout<<"i:"<<i<<endl;
		  //std::stringstream convert;
		  //convert << i+3;
		  //string object_file = "home/robo/object-images/pepper" + convert.str() +".png";
//		  cv::Mat img_in = cv::imread ( object_file,1);
//
//		  cv::Mat img_object = cvCreateMat(img_in.rows, img_in.cols, CV_8UC1);
//		  cvtColor(img_in, img_object,CV_BGR2GRAY);
//		  object.w=img_object.cols;
//		  object.h=img_object.rows;

	  cv::SurfFeatureDetector detector (minHessian);
	  detector.detect( img_object, object.keypoints );
	  cv::SurfDescriptorExtractor extractor;
	  extractor.compute (img_object, object.keypoints, object.descriptors );
		  //std::cout<<"all nice"<<i<<endl;
//		  if(i!=3){
//			  object.keypoints.insert(object.keypoints.end(),object2.keypoints.begin(),object2.keypoints.end());
//			  hconcat(object.descriptors,object2.descriptors,object.descriptors);
//		  }
	  //}

	  clock_gettime(CLOCK_MONOTONIC, &tend);
	  if(have_gui) object.image=img_object;

	}
	uint64_t t=(tend.tv_nsec-tstart.tv_nsec)/1000L+(tend.tv_sec-tstart.tv_sec)*1000000L;
	std::cerr<<' '<<t<<"µs, done.\n";

	if(scene_file) {
	  cv::Mat img_scene  = cv::imread ( scene_file,1);
	  double res = DetectObject(img_scene);
	  if(have_gui) cv::waitKey(0);
	  return 0;
	} else {
	  image_transport::ImageTransport it(nh);
	  obj_sub=it.subscribe("objdetect/images",1,DetectObjectHandler);
	  obj_pub=nh.advertise<std_msgs::Empty>("objdetect/spotted",1,false);
	  talk_pub=nh.advertise<std_msgs::String>("robot/talk",1,false);
	}

	ros::Rate loop_rate(100);
	while(ros::ok()) {
	  loop_rate.sleep();
	  ros::spinOnce();
	  if(have_gui) cv::waitKey(1);
	}

	return 0;
}
