/*
 * DifferentialDriveModel.h
 *
 *  Created on: Apr 26, 2015
 *      Author: VyLong
 */

#ifndef LIBDYNAMIC_DIFFERENTIALDRIVEMODEL_H_
#define LIBDYNAMIC_DIFFERENTIALDRIVEMODEL_H_

class DifferentialDriveModel {
public:
	float vel_r;
	float vel_l;

	DifferentialDriveModel();
	DifferentialDriveModel(float vr, float vl);

	void toUnicycle(float &v, float &w);
};

#endif /* LIBDYNAMIC_DIFFERENTIALDRIVEMODEL_H_ */
