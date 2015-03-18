/*
 * RobotLocation.h
 *
 *  Created on: Mar 13, 2015
 *      Author: VyLong
 */

#ifndef LIBSTORAGE_INC_ROBOTLOCATION_H_
#define LIBSTORAGE_INC_ROBOTLOCATION_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "libmath/inc/Vector2.h"

class RobotLocation
{
public:
	uint32_t ID;
	Vector2<float> vector;
//	float x;
//	float y;

	// Constructors
	//RobotLocation(uint32_t id = 0, Vector2<float> vect2 = NULL) {ID = id, x = vect2.x; y = vect2.y; };
	RobotLocation(uint32_t id = 0, Vector2<float> vect2 = NULL) {ID = id, vector = vect2; };
	//RobotLocation(uint32_t id = 0, float ix = 0, float iy = 0) { ID = id; x = ix; y = iy; };
	RobotLocation(uint32_t id = 0, float ix = 0, float iy = 0) { ID = id; vector.x = ix; vector.y = iy; };

    // Copy Constructor
	//RobotLocation(const RobotLocation &rhs) { ID = rhs.ID, x = rhs.x; y = rhs.y; }
	RobotLocation(const RobotLocation &rhs) { ID = rhs.ID, vector = rhs.vector; }

	// Destructor
	~RobotLocation() {};

    // Operators
	//RobotLocation& operator=(const RobotLocation &rhs) { ID = rhs.ID,  x = rhs.x; y = rhs.y; return *this; }
	RobotLocation& operator=(const RobotLocation &rhs) { ID = rhs.ID, vector = rhs.vector; return *this; }
    bool operator == (const RobotLocation &rhs) const { return (ID == rhs.ID); }
    bool operator != (const RobotLocation &rhs) const { return (ID != rhs.ID); }
};

#endif /* LIBSTORAGE_INC_ROBOTLOCATION_H_ */
