/*
 * UnicycleModel.h
 *
 *  Created on: Apr 26, 2015
 *      Author: VyLong
 */

#ifndef LIBDYNAMIC_UNICYCLEMODEL_H_
#define LIBDYNAMIC_UNICYCLEMODEL_H_

class UnicycleModel {
public:
	float v;
	float w;

	UnicycleModel();
	UnicycleModel(float iv, float iw);

	void toDifferentialDrive(float &vel_r, float &vel_l);
};

#endif /* LIBDYNAMIC_UNICYCLEMODEL_H_ */
