/*
 * CeilingCam.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: Ed
 */

#include "CeilingCam.h"
#include <string>
#include <sstream>
#include <cmath>

//	EPSILON should be set to a realistic tolerance for the system according to camera resolution
//	A higher EPSILON will not improve accuracy, and will waste processing cycles
//	REMEMBER: This is in pixels, NOT in feet!
#define EPSILON 0.1 //Minimum deviation from 0

#define M_PI 3.14159265//Remove for Linux compilation

#include <iostream>
// uncomment to disable assert()
// #define NDEBUG
#include <cassert>

using namespace std;



//CONSTRUCTORS		//////////////////////////////////////

/*
 * Read in string and initialize camera object
 * PARAM:		String formatted as follows:
 * 					Camera Number,Address,Width,Height,Rotation,Center x,Center y,Radius,
 * 					(cont'd) Position x,Position y,Position z,,Ceiling Height,Info
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
	//Rotation
	getline(stream, buf, ',');
	d_rotation = ToDbl(buf);

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
	std::cout << "Camera Rotated by " << d_rotation << " degrees\n";
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
	vector<double> cartesian = {d_x,d_y,d_z-d_ceiling};
	//cout << "(" << cartesian[0] << ", " << cartesian[1] << ", " << cartesian[2] << ")\n";
	//Move origin to optical center
	blobX -= d_xo;
	blobY -= d_yo;
	//Convert blob position to image polar coordinates (d, theta)
	//  A.	Find distance between blob and center (d)
	double distance = hypot(fabs(blobX), fabs(blobY));

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

	//	D.	Compensate for camera rotation
	theta -= d_rotation / 180 * M_PI;

	//	E.	Calculate actual distance

	//		(The diameter of the camera corresponds to pi rad of worldview
	//		The angle "phi" between the vertical and the blob is:
	double phi = (distance / (float)d_radius) * (M_PI / 2.0);
	//cout << "Phi: " << phi << "\n";

	//		Phi can then be used along with the distance to the floor to find the actual distance
	distance = d_ceiling * tan(phi);
	//cout << "Distance: " << distance << "\n";

	//	F.	Convert back to Cartesian (this time with distance corrected)
	//		Note that this is added to the world coordinates
	cartesian[0] += distance * cos(theta);
	cartesian[1] += distance * sin(theta);
	//cout << "(" << cartesian[0] << ", " << cartesian[1] << ", " << cartesian[2] << ")\n\n";
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
	string testString = "1,http://username:password@192.168.1.11/video.cgi?camera=25.mjpg,1920,1080,90,720,500,600,12.5,9.5,8.75,8.5,Test Camera";
	CeilingCam test(testString);
	test.testInitialization();
	cout << "Initialization Success\n";
	test.testGetters(test);
	cout << "Getters Success\n";
	test.testTheta(test);
	cout << "Theta Success\n";
	test.testCartesian(test);
	cout << "Cartesian Success\n";
}

void CeilingCam::testInitialization(){
	assert(d_number==1);
	assert(d_address=="http://username:password@192.168.1.11/video.cgi?camera=25.mjpg");
	assert(d_width==1920);
	assert(d_height==1080);
	assert(d_rotation==90);
	assert(d_xo==720);
	assert(d_yo==500);
	assert(d_radius==600);
	assert(d_x==12.5);
	assert(d_y==9.5);
	assert(d_z==8.75);
	assert(d_ceiling==8.5);
	assert(d_info=="Test Camera");
}

void CeilingCam::testGetters(CeilingCam testObject){
	vector<int> testVector = testObject.center();
	assert(testVector[0]==720);
	assert(testVector[1]==500);

	vector<double> testVectorDbl = testObject.position();
	assert(testVectorDbl[0]==12.5);
	assert(testVectorDbl[1]==9.5);
	assert(testVectorDbl[2]==8.75);
}

void CeilingCam::testTheta(CeilingCam testObject){
	double e = 0.000001;
	assert(testObject.findTheta(0,0) < e);
	assert(testObject.findTheta(1,1)-(M_PI/4) < e);
	assert(testObject.findTheta(-1,1)-(3 * M_PI/4) < e);
	assert(testObject.findTheta(-1,-1)-(-3 * M_PI/4) < e);
	assert(testObject.findTheta(1,-1)-(M_PI/-4) < e);
}

void CeilingCam::testCartesian(CeilingCam testObject){
	double e = 0.0001;
	/*
	 * Test Conditions:
	 * Default Position: (12.5, 9.5, 0.25)
	 * Rotation: 90 degrees
	 * Size: 1920 x 1080
	 * Center: (500, 720)
	 * Radius: 600
	 */
	//test directly under
	vector<double> testVectorDbl = testObject.toCartesian(720,500);
	assert(testVectorDbl[0] - 12.5 < e);
	assert(testVectorDbl[1] - 9.5 < e);
	assert(testVectorDbl[2] - 0.25 < e);

	//Right
	testVectorDbl = testObject.toCartesian(720+180,500);
	assert(testVectorDbl[0] - 12.5 < e);
	assert(testVectorDbl[1] - 5.16903 < e);
	assert(testVectorDbl[2] - 0.25 < e);

	//Up
	testVectorDbl = testObject.toCartesian(720, 500+180);
	assert(testVectorDbl[0] - 16.83097 < e);
	assert(testVectorDbl[1] - 9.5 < e);
	assert(testVectorDbl[2] - 0.25 < e);

	//Angle
	testVectorDbl = testObject.toCartesian(720-200, 500+342);
	//phi 59.428023692530782324704925256661
	//theta 30.318896200757337252069624378432
	//distance 14.38877245751138506976412588597
	//x 24.920807242208657984933758426808
	//y 16.763629966203893558440794401642
	assert(testVectorDbl[0] - 24.92080724 < e);
	assert(testVectorDbl[1] - 16.76362996 < e);
	assert(testVectorDbl[2] - 0.25 < e);

	//Over
	testVectorDbl = testObject.toCartesian(720+1000, 500);
	//phi 89.985
	//theta -90
	//distance 32467.607648981713009944399854133
	//x 12.5
	//y -32458.107648981713009944399854133
	assert(testVectorDbl[0] - 12.5 < e);
	assert(testVectorDbl[1] + -32458.1076489817 < e);
	assert(testVectorDbl[2] - 0.25 < e);

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
	//cout << "Theta equals: " << theta << "\n";
	return theta;
}
