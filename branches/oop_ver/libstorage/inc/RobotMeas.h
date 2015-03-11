/*
 * RobotMeas.h
 *
 *  Created on: Mar 10, 2015
 *      Author: VyLong
 */

#ifndef ROBOTMEAS_H_
#define ROBOTMEAS_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

class RobotMeas
{
public:
	uint32_t ID;
	uint16_t Distance;

	// Constructor
    RobotMeas(uint32_t id, uint16_t dis = 0) { ID = id; Distance = dis; }

    // Copy Constructor
    RobotMeas(const RobotMeas &rhs) {ID = rhs.ID; Distance = rhs.Distance; }

    // Destructor
    ~RobotMeas() {};

    // Operators
    RobotMeas& operator=(const RobotMeas &rhs) { ID = rhs.ID,  Distance = rhs.Distance; return *this; }
    bool operator == (const RobotMeas &rhs) const { return ID == rhs.ID; }
    bool operator != (const RobotMeas &rhs) const { return ID != rhs.ID; }
};

#endif /* ROBOTMEAS_H_ */
