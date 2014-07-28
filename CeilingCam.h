/*
 * CeilingCam.h
 *
 *  Created on: Jul 17, 2014
 *      Author: Ed
 */

#ifndef CEILINGCAM_H_
#define CEILINGCAM_H_

#include <string>
#include <vector>
#include <iostream>

class CeilingCam {
	//FIELDS
	int d_number;
	int d_xo, d_yo, d_radius, d_height, d_width;
	double d_x, d_y, d_z, d_rotation;
	double d_ceiling;
	std::string d_address;
	std::string d_info;

	//HELPER FUNCTIONS
	int ToInt ( std::string &Text );
	double ToDbl ( std::string &Text );
	double findTheta( double x, double y );

public:
	//CONSTRUCTORS
	CeilingCam(std::string input);
	virtual ~CeilingCam();

	//MEMBER FUNCTIONS
	int number(){ return d_number; }		//Get camera number
	void print(); 							//Print all the info
	std::vector<int> center();				//Get coordinates of the optical center [x,y]
	std::vector<double> position();			//Get coordinates of the camera's position in world
	double height(){ return d_ceiling; }	//Get ceiling height
	std::string feed(){ return d_address; }	//Get feed address
	std::vector<int> imgSize();				//Get the height and width of the feed
	std::vector<double> toCartesian(double blobX, double blobY);
											//Get position of blob in Cartesian world coordinates

	//TEST FUNCTIONS
	void testGetters(CeilingCam testObject);
	void testInitialization();
	void testTheta(CeilingCam testObject);
	void testCartesian(CeilingCam testObject);
};

#endif /* CEILINGCAM_H_ */
