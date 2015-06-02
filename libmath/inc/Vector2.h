/*
 * Vector2.h
 *
 *  Created on: Mar 13, 2015
 *      Author: VyLong
 */

#ifndef LIBSTORAGE_INC_VECTOR2_H_
#define LIBSTORAGE_INC_VECTOR2_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

template <typename T>
class Vector2
{
public:
	T x;
	T y;

	// Constructors
	Vector2(T ix = 0, T iy = 0) { x = ix; y = iy; };

    // Copy Constructor
	Vector2(const Vector2<T> &rhs) { x = rhs.x; y = rhs.y; }

	// Destructor
	~Vector2() {};

    // Operators
	Vector2<T>& operator=(const T value) { x = value; y = value; return *this; };
	Vector2<T>& operator=(const Vector2<T> &rhs) { x = rhs.x,  y = rhs.y; return *this; }
    bool operator == (const Vector2<T> &rhs) const { return ((x == rhs.x) && (y == rhs.y)); }
    bool operator != (const Vector2<T> &rhs) const { return ((x != rhs.x) || (y != rhs.y)); }
    Vector2<T> operator+(const Vector2<T>& v1);
    Vector2<T> operator-(const Vector2<T>& v1);
    Vector2<T> operator*(const T scale);

    // Methods
    float getMagnitude();
    void normalize();
    Vector2<T> getNormalized();
    void rotate(float angleInRad);
    Vector2<T> getRotate(float angleInRad);
    T DotProduct(const Vector2<T>& v1);
};

//***************
// Add operator *
//***************
template <typename T>
Vector2<T> Vector2<T>::operator+(const Vector2<T>& v1)
{
	Vector2<T> result;
	result.x = v1.x + this->x;
	result.y = v1.y + this->y;
	return result;
};

//********************
// Subtract operator *
//********************
template <typename T>
Vector2<T> Vector2<T>::operator-(const Vector2<T>& v1)
{
	Vector2<T> result;
	result.x = (this->x) - v1.x;
	result.y = (this->y) - v1.y;
	return result;
};

//*******************
// Scale operator * *
//*******************
template <typename T>
Vector2<T> Vector2<T>::operator*(const T scale)
{
	Vector2<T> result;
	result.x = (this->x) * scale;
	result.y = (this->y) * scale;
	return result;
}

//*******************************
// Get Magnitude of the Vector. *
//*******************************
template <typename T>
float Vector2<T>::getMagnitude()
{
	return sqrtf(x*x + y*y);
}

//*************************
// Normalized the Vector. *
//*************************
template <typename T>
void Vector2<T>::normalize()
{
	float m = getMagnitude();
	x /= m;
	y /= m;
}

//****************************************
// Get normalized version of the Vector. *
//****************************************
template <typename T>
Vector2<T> Vector2<T>::getNormalized()
{
	Vector2<T> r(x, y);
	r.normalize();
	return r;
}

//*********************
// Rotate the Vector. *
//*********************
template <typename T>
void Vector2<T>::rotate(float angleInRad)
{
	x = x * cosf(angleInRad) - y * sinf(angleInRad);
	y = x * sinf(angleInRad) + y * cosf(angleInRad);
}

//*************************************
// Get rotated version of the Vector. *
//*************************************
template <typename T>
Vector2<T> Vector2<T>::getRotate(float angleInRad)
{
	Vector2<T> r(x, y);
	r.rotate(angleInRad);
	return r;
}

//*****************************************
// Cal dot product version of the Vector. *
//*****************************************
template <typename T>
T Vector2<T>::DotProduct(const Vector2<T>& v1)
{
	return ((this->x * v1.x) + (this->y * v1.y));
}

#endif /* LIBSTORAGE_INC_VECTOR2_H_ */
