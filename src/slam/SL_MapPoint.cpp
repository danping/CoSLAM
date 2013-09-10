/*
 * SL_MapPoint.cpp
 *
 *  Created on: 2010-11-19
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_MapPoint.h"
#include "SL_FeaturePoint.h"
#include "SL_error.h"
#include <cassert>
/////////////////////////////////////////////////////////////////////////////////////////
//MapPoint
MapPoint::MapPoint() :
		Point3dId(), firstFrame(-1), lastFrame(-1), numVisCam(0), bNewPt(true), state(
				STATE_MAPPOINT_CURRENT), iLocalType(0), bUncertain(false), staticFrameNum(
				0), errorNum(0), staticNum(0), flag(FLAG_MAPPOINT_NORMAL), pre(
				0), next(0) {
	id = (longInt) this;
	memset(pFeatures, 0, sizeof(FeaturePoint*) * SLAM_MAX_NUM);
	fill_n(cov, 9, 0);
	fill_n(color, 3, 0);
}
MapPoint::MapPoint(double a, double b, double c) :
		Point3dId(a, b, c, -1), firstFrame(-1), lastFrame(-1), numVisCam(0), bNewPt(
				true), state(STATE_MAPPOINT_CURRENT), iLocalType(0), bGlobalDyn(
				false), bUncertain(false), staticFrameNum(0), errorNum(0), staticNum(
				0), flag(FLAG_MAPPOINT_NORMAL), pre(0), next(0) {
	id = (longInt) this;
	memset(pFeatures, 0, sizeof(FeaturePoint*) * SLAM_MAX_NUM);
	fill_n(cov, 9, 0);
	fill_n(color, 3, 0);
}
MapPoint::MapPoint(double a, double b, double c, int frame) :
		Point3dId(a, b, c, -1), firstFrame(frame), lastFrame(frame), numVisCam(
				0), bNewPt(true), state(STATE_MAPPOINT_CURRENT), iLocalType(0), bGlobalDyn(
				false), bUncertain(false), staticFrameNum(0), errorNum(0), staticNum(
				0), flag(FLAG_MAPPOINT_NORMAL), pre(0), next(0) {
	id = (longInt) this;
	memset(pFeatures, 0, sizeof(FeaturePoint*) * SLAM_MAX_NUM);
	fill_n(cov, 9, 0);
	fill_n(color, 3, 0);
}
MapPoint::MapPoint(const MapPoint& other) :
		Point3dId(other), firstFrame(other.firstFrame), lastFrame(
				other.lastFrame), numVisCam(other.numVisCam), bNewPt(other.x), state(
				other.state), iLocalType(other.iLocalType), bGlobalDyn(
				other.bGlobalDyn), bUncertain(other.bUncertain), staticFrameNum(
				other.staticFrameNum), errorNum(other.errorNum), staticNum(
				other.staticNum), flag(other.flag), pre(0), next(0) {
	memcpy(pFeatures, other.pFeatures, sizeof(FeaturePoint*) * SLAM_MAX_NUM);
	memcpy(cov, other.cov, sizeof(double) * 9);
}
MapPoint::~MapPoint() {

}
void MapPoint::addFeature(int iCam, FeaturePoint* pt) {
	if (pFeatures[iCam]) {
		if (pFeatures[iCam]->f == pt->f)
			pFeatures[iCam]->mpt = 0;
		pFeatures[iCam] = pt;
		pt->mpt = this;
	} else {
		pFeatures[iCam] = pt;
		pt->mpt = this;
		numVisCam++;
	}
}
void MapPoint::removeFeature(int iCam) {
	if (pFeatures[iCam]) {
		pFeatures[iCam]->mpt = 0;
		pFeatures[iCam] = 0;
		numVisCam--;
	}
}
void MapPoint::removeAllFeatures() {
	for (int i = 0; i < SLAM_MAX_NUM; i++) {
		removeFeature(i);
	}
}
void MapPoint::updateVisCamNum(int frame) {
	numVisCam = 0;
	lastFrame = -1;
	for (int i = 0; i < SLAM_MAX_NUM; i++) {
		if (pFeatures[i]) {
			if (pFeatures[i]->f > lastFrame)
				lastFrame = pFeatures[i]->f;
			if (pFeatures[i]->f == frame)
				numVisCam++;
		}
	}
}
void MapPoint::setFalse() {
	iLocalType = TYPE_MAP_FALSE;
	staticNum = 0;
}
void MapPoint::setLocalDynamic() {
	iLocalType = TYPE_MAP_DYNAMIC;
	staticFrameNum = 0;
	staticNum = 0;
	clearUncertain();
}
void MapPoint::setLocalStatic() {
	iLocalType = TYPE_MAP_STATIC;
	staticFrameNum = 0;
	staticNum = 0;
	clearUncertain();
}
void MapPoint::setUncertain() {
	bUncertain = true;
	staticNum = 0;
}
void MapPoint::clearUncertain() {
	bUncertain = false;
}
void MapPoint::updatePosition(double p[3], double C[9]) {
	memcpy(M, p, sizeof(double) * 3);
	memcpy(cov, C, sizeof(double) * 9);
}

void MapPoint::setColor(uchar r, uchar g, uchar b) {
	color[0] = r;
	color[1] = g;
	color[2] = b;
}
void MapPoint::setColor(uchar rgb[3]) {
	color[0] = rgb[0];
	color[1] = rgb[1];
	color[2] = rgb[2];
}
void MapPoint::print() {
	logInfo("------------------\n");
	logInfo("%d:\n", id);

	if (isFalse())
		logInfo("type : false\n");
	else {
		logInfo("local type:'%s'\n",
				isLocalDynamic() ? "local dynamic" : "local static");
		if (isUncertain())
			logInfo("uncertain\n");
	}

	logInfo("first frame:%d , last frame:%d\n", firstFrame, lastFrame);
	//	logInfo("staticNum:%d, staticFrameNum:%d,errorNum:%d\n", staticNum, staticFrameNum, errorNum);
	logInfo("number of visibility : %d\n", numVisCam);
	logInfo("static frame num:%d\n", staticFrameNum);
//	logInfo("(%lf,%lf,%lf)\n", x, y, z);
//	logInfo("covariance:\n");
//	printMat(3, 3, cov);
//	for (int i = 0; i < SLAM_MAX_NUM; i++) {
//		if (pFeatures[i])
//			logInfo("[%d]:f:%d(%lf,%lf)\n", i, pFeatures[i]->f, pFeatures[i]->x, pFeatures[i]->y);
//	}
	logInfo("==================\n");
}
