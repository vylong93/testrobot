/*
 * vector3.h
 *
 *  Created on: Feb 15, 2015
 *      Author: VyLong
 */

#ifndef VECTOR3_H_
#define VECTOR3_H_

#include "math.h"
#include "libmath/inc/quaternion.h"

template <typename T>
class Vector3 {

    public:
        T x;
        T y;
        T z;

        Vector3() { x = 0; y = 0; z = 0; }
        Vector3(T nx, T ny, T nz) { x = nx; y = ny; z = nz; }

        Vector3 operator+(const Vector3& v1)
        {
          Vector3 result;
          result.x = v1.x+this->x;
          result.y = v1.y+this->y;
          result.z = v1.z+this->z;
          return result;
        };

        float getMagnitude()
        {
        	return sqrtf(x*x + y*y + z*z);
        }

        void normalize()
        {
        	float m = getMagnitude();
        	x /= m;
        	y /= m;
        	z /= m;
        }

        Vector3 getNormalized()
        {
        	Vector3 r(x, y, z);
        	r.normalize();
        	return r;
        }

        void rotate(Quaternion q)
        {
        	// http://www.cprogramming.com/tutorial/3d/quaternions.html
        	// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
        	// http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
        	// ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

        	// P_out = q * P_in * conj(q)
        	// - P_out is the output vector
        	// - q is the orientation quaternion
        	// - P_in is the input vector (a*aReal)
        	// - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
        	Quaternion p(0, x, y, z);

        	// quaternion multiplication: q * p, stored back in p
        	p = q.getProduct(p);

        	// quaternion multiplication: p * conj(q), stored back in p
        	p = p.getProduct(q.getConjugate());

        	// p quaternion is now [0, x', y', z']
        	x = p.x;
        	y = p.y;
        	z = p.z;
        }

        Vector3 getRotated(Quaternion q)
        {
        	Vector3 r(x, y, z);
        	r.rotate(q);
        	return r;
        }

};

#endif /* VECTOR3_H_ */
