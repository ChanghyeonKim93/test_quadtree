#ifndef _COMMONFUNCTION_H_
#define _COMMONFUNCTION_H_
#include <iostream>
#include "CommonStruct.h"

// 빠른 거리 계산을 위한 inline 처리.
inline double distEuclidean(double& u1, double& u2, double& v1, double& v2)
{
	return (u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2);
};
inline double distEuclidean(Point2<double>& pt1, Point2<double>& pt2)
{
	return (pt1.u - pt2.u)*(pt1.u - pt2.u) + (pt1.v - pt2.v)*(pt1.v - pt2.v);
};

inline double distEuclideanSqrt(double& u1, double& u2, double& v1, double& v2)
{
	return sqrt((u1 - u2)*(u1 - u2) + (v1 - v2)*(v1 - v2));
};
inline double distEuclideanSqrt(Point2<double>& pt1, Point2<double>& pt2)
{
	return sqrt((pt1.u - pt2.u)*(pt1.u - pt2.u) + (pt1.v - pt2.v)*(pt1.v - pt2.v));
};

inline double distManhattan(double& u1, double& u2, double& v1, double& v2)
{
	return abs(u1 - u2) + abs(v1 - v2);
};
inline double distManhattan(Point2<double>& pt1, Point2<double>& pt2)
{
	return abs(pt1.u - pt2.u) + abs(pt1.v - pt2.v);
};
#endif