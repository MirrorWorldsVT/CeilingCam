/*
 * CeilingCam.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: Ed
 */

#include "CeilingCam.h"
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
//	EPSILON should be set to a realistic tolerance for the system according to camera resolution
//	A higher EPSILON will not improve accuracy, and will waste processing cycles
//	REMEMBER: This is in pixels, NOT in feet!
#define EPSILON 0.1 //Minimum deviation from 0

using namespace std;

//CONSTRUCTORS		//////////////////////////////////////

/*
 * Read in string and initialize camera object
 * PARAM:		String formatted as follows:
 * 					Camera Number,Address,Height,Width,Center x,Center y,Radius,
 * 					(cont'd) Position x,Position y,Position z,Ceiling Height,Info
 * RETURN:		Initialized CeilingCam object
 */
CeilingCam::CeilingCam(std::string input) {
	istringstream stream(input);
	string buf;

	//Camera Number
	getline(stream, buf, ',');
	d_number = ToInt(buf);

	//Address String
	getline(stream, buf, ',');
	d_address = buf;

	//Image Dimensions
	//Width
	getline(stream, buf, ',');
	d_width = ToInt(buf);
	//Height
	getline(stream, buf, ',');
	d_height = ToInt(buf);

	//Center
	//xo
	getline(stream, buf, ',');
	d_xo = ToInt(buf);
	//yo
	getline(stream, buf, ',');
	d_yo = ToInt(buf);
	//Radius
	getline(stream, buf, ',');
	d_radius = ToInt(buf);

	//Camera Position
	//x
	getline(stream, buf, ',');
	d_x = ToDbl(buf);
	//y
	getline(stream, buf, ',');
	d_y = ToDbl(buf);
	//z
	getline(stream, buf, ',');
	d_z = ToDbl(buf);
	//Ceiling Height
	getline(stream, buf, ',');
	d_ceiling = ToDbl(buf);

	//Info String
	getline(stream, buf, ',');
	d_info = buf;

	print();
}

/*
 * Destructor
 */
CeilingCam::~CeilingCam() {}

//MEMBER FUNCTIONS	//////////////////////////////////////

/*
 * Print the CeilingCam object
 * Useful for debugging
 * Prints as follows:
 * 		Camera #1:
 * 		Test Camera
 * 		Position: 12.5, 9.5, 8.125
 * 		Ceiling Height: 8.5
 * 		Image Size: 1920 x 1080
 * 		Center: (720, 540)
 * 		Radius: 600
 * 		Feed: http://username:password@192.168.1.11/video.cgi?camera=25.mjpg
 */
void CeilingCam::print(){
	std::cout << "Camera #" << d_number << ":\n";
	std::cout << d_info << "\nPosition: ";
	std::cout << d_x << ", " << d_y << ", " << d_z << "\n";
	std::cout << "Ceiling Height: " << d_ceiling << "\n";
	std::cout << "Image Size: " << d_width << " x " << d_height << "\n";
	std::cout << "Center: (" << d_xo << ", " << d_yo << ")\n";
	std::cout << "Radius: " << d_radius << "\n";
	std::cout << "Feed: " << d_address << "\n";
}
/*
 * Convert from blob position to world position
 * PARAM: 	Coordinates formatted as follows
 * 				blobX	x coordinate in pixels of blob
 * 				blobY	y coordinate in pixels of blob
 * RETURN:	Vector containing the world coordinates of the blob
 * 			Note that coordinates are in the same units as the height specified in CSV
 */
vector<double> CeilingCam::toCartesian(double blobX, double blobY){
	//initialize
	vector<double> cartesian = {d_x,d_y,d_z};
	//Correct for optical center
	double offsetX = d_xo - (d_width / 2.0);
	double offsetY = d_yo - (d_height / 2.0);
	blobX += offsetX;
	blobY += offsetY;
	//TODO: Implement camera rotation correction
	//Convert blob position to image polar coordinates (d, theta)
	//  A.	Find distance between blob and center (d)
	int distance = hypot(blobX, blobY);

	//	B.	Infinity Cases

	//		Case 0: blob is outside of radius
	//			NOTE: Unlike the other cases,
	//			this case continues on through the code
		if ( distance > (d_radius - EPSILON)){
			distance = d_radius - EPSILON;
		}

	//		Case 1: directly underneath camera
	//			Returns position of camera
	if ( distance < EPSILON ){
		return cartesian;
	}

	//  C.	Find angle between blob and x axis (theta)
	double theta = findTheta(blobX, blobY);

	//	D.	Calculate actual distance

	//		(The diameter of the camera corresponds to pi rad of worldview
	//		The angle "phi" between the vertical and the blob is:
	double phi = (distance / d_radius) * (M_PI / 2);

	//		Phi can then be used along with d_height to find the actual distance with
	distance = d_height * tan(phi);

	//	E.	Convert back to Cartesian (this time with distance corrected)
	cartesian[0] = distance * cos(theta);
	cartesian[1] = distance * sin(theta);
	return cartesian;
}
//	GETTERS		//////////////////////////////////////

/*
 * Get optical center
 * RETURN:	Vector containing coordinates in [x,y]
 */
std::vector<int> CeilingCam::center(){
	std::vector<int> ctr = { d_xo, d_yo };
	return ctr;
}

/*
 * Get camera position
 * RETURN:	Vector containing coordinates in [x,y,z]
 */
std::vector<double> CeilingCam::position(){
	std::vector<double> pos = { d_x, d_y, d_z };
	return pos;
}

 /*
  * Get feed size
  * RETURN:	Vector containing [width,height]
  */
 std::vector<int> CeilingCam::imgSize(){
	std::vector<int> size = { d_width, d_height };
	return size;
}



//TEST FUNCTIONS	//////////////////////////////////////
int main(){
	string testString = "1,http://username:password@192.168.1.11/video.cgi?camera=25.mjpg,1920,1080,720,540,600,12.5,9.5,8.125,8.5,Test Camera";
	CeilingCam test(testString);
}

//HELPER FUNCTIONS	//////////////////////////////////////
/*
 * Convert string to integer
 * PARAM: 		A string containing a number
 * RETURN:		Integer value of first number in string
 */
int CeilingCam::ToInt ( string &Text )//Text not by const reference so that the function can be used with a
{                               //character array as argument
	stringstream ss(Text);
	int result;
	return ss >> result ? result : 0;
}

/*
 * Convert string to double
 * PARAM: 		A string containing a number
 * RETURN:		Double value of first number in string
 */
double CeilingCam::ToDbl ( std::string &Text )//Text not by const reference so that the function can be used with a
{                               //character array as argument
	stringstream ss(Text);
	double result;
	return ss >> result ? result : 0;
}

/*
 * Find polar angle from Cartesian coordinates
 * PARAM: 		Cartesian coordinates (x,y)
 * 				NOTE: Returns NULL when both values are within EPSILON of the axis
 * RETURN:		Polar "theta" with values n where 0 <= n < 2PI
 * 				NOTE: If x or y is closer to axis than EPSILON, the values are
 * 				rounded to 0, and possible infinity cases are circumvented.
 */
double CeilingCam::findTheta( double x, double y){
	double theta = 0;
	// A. Check for near-axis coordinates

	if (( fabs(x) < EPSILON ) && (fabs(y) < EPSILON)){
			return 0;
	}

	// B. atan2 produces values from pi to -pi instead of just pi/2 to -pi/2
	theta = atan2(y,x);
	return theta;
}
