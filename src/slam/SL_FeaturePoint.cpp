/*
 * SL_ImgPoint.cpp
 *
 *  Created on: 2010-11-21
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_FeaturePoint.h"
#include "SL_MapPoint.h"
#include "SL_error.h"
#include <cstddef>
#include <stdint.h>

FeaturePoint::FeaturePoint() :
		Point2d(), id(0), f(-1), camId(-1), type(0), reprojErr(0), mpt(0), mpt_id(
				-1), pre(0), next(0), cam(0), preFrame(0), nextFrame(0), bKeyFrm(
				false) {
	id = (longInt) this;

}
FeaturePoint::FeaturePoint(int f1, int id, double x1, double y1) :
		Point2d(x1, y1), id(0), f(f1), camId(id), type(0), reprojErr(0), mpt(0), mpt_id(
				-1), pre(0), next(0), cam(0), preFrame(0), nextFrame(0), bKeyFrm(
				false) {
	id = (longInt) this;
}
FeaturePoint::FeaturePoint(const FeaturePoint& other) {
	operator=(other);
}
FeaturePoint& FeaturePoint::operator=(const FeaturePoint& other) {
	if (&other != this) {
		id = other.id;
		f = other.f;
		camId = other.camId;
		x = other.x;
		y = other.y;
		type = other.type;
		reprojErr = other.reprojErr;
		mpt = other.mpt;
		mpt_id = other.mpt_id;
		pre = other.pre;
		next = other.next;
		cam = other.cam;
		preFrame = other.preFrame;
		nextFrame = other.nextFrame;
		bKeyFrm = other.bKeyFrm;
	}
	return *this;
}
FeaturePoint::~FeaturePoint() {
}


