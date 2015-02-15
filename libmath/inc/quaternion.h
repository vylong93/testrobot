/*
 * quaternion.h
 *
 *  Created on: Feb 15, 2015
 *      Author: VyLong
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

class Quaternion {
    public:
        float w;
        float x;
        float y;
        float z;

        Quaternion();
        Quaternion(float nw, float nx, float ny, float nz);

        Quaternion getProduct(Quaternion q);
        Quaternion getConjugate();
        float getMagnitude();
        void normalize();
        Quaternion getNormalized();
};

#endif /* QUATERNION_H_ */
