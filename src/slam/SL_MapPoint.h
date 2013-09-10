/*
 * SL_Pt3D.h
 *
 *  Created on: 2010-11-19
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_MAPPOINT_H_
#define SL_MAPPOINT_H_

#include "SL_Define.h"
#include "math/SL_Matrix.h"
#include "slam/SL_NCCBlock.h"

#define FLAG_MAPPOINT_NORMAL 0
#define FLAG_MAPPOINT_TEST1 1
#define FLAG_MAPPOINT_TEST2 2
#define FLAG_MAPPOINT_TEST3 3
#define FLAG_MAPPOINT_TEST4 4

#define TYPE_MAP_FALSE	  -2
#define TYPE_MAP_STATIC  0
#define TYPE_MAP_DYNAMIC 1

#define STATE_MAPPOINT_CURRENT 1
#define STATE_MAPPOINT_ACTIVE 0
#define STATE_MAPPOINT_INACTIVE -1

#include <fstream>
#include "geometry/SL_Point.h"

class FeaturePoint;
class MapPoint: public Point3dId {
public:
	//3x3 covariance matrix to describe position uncertainty
	double cov[9];
	uchar color[3];

	//source frame
	int firstFrame;
	int lastFrame;

	//number of 	visi ble cameras
	int numVisCam;

	//corresponding	feature points
	FeaturePoint* pFeatures[SLAM_MAX_NUM];

	//a new map point 
	bool bNewPt;

	//state of map point : current frame, active, inactive
	int state;

	//type of map point : static, dynamic, uncertain, and false
	int iLocalType; //'dynamic', 'static', or 'false'
	bool bGlobalDyn; //'dynamic' or 'static'

	bool bUncertain;

	///for map classification
	//number of frames for keeping stationary 
	int staticFrameNum;
	int errorNum;
	int staticNum;

	//for debug usage
	int flag;

	MapPoint* pre;
	MapPoint* next;

	//store the NCC blocks for each view
	NCCBlock nccBlks[SLAM_MAX_NUM];
public:
	MapPoint();
	MapPoint(double a, double b, double c);
	MapPoint(double a, double b, double c, int frame);
	MapPoint(const MapPoint& other);
	~MapPoint();
public:
	void addFeature(int iCam, FeaturePoint* pt);
	void removeFeature(int iCam);
	void removeAllFeatures();

	void setFalse(); /*false map point caused by incorrect matching*/
	void setLocalDynamic(); /*locally dynamic map point*/
	void setLocalStatic(); /*locally static map point*/
	void setGlobalDynamic(); /*globally dynamic map point*/
	void setGlobalStatic(); /*globally static map point*/

	void setUncertain(); /* inputs to map classfication*/
	void clearUncertain();

	bool isFalse() const {
		return iLocalType == TYPE_MAP_FALSE;
	}
	bool isLocalDynamic() const {
		return iLocalType == TYPE_MAP_DYNAMIC;
	}
	bool isLocalStatic() const {
		return iLocalType == TYPE_MAP_STATIC;
	}
	bool isCertainStatic() const {
		return !bUncertain && iLocalType == TYPE_MAP_STATIC;
	}
	bool isCertainDynamic() const {
		return !bUncertain && iLocalType == TYPE_MAP_DYNAMIC;
	}
	bool isGlobalDynamic() const {
		return bGlobalDyn;
	}
	bool isGlobalStatic() const {
		return !bGlobalDyn;
	}
	bool isUncertain() const {
		return bUncertain;
	}
	void updatePosition(double M[3], double C[9]);
	void updateVisCamNum(int frame);

	void setColor(uchar r, uchar g, uchar b);
	void setColor(uchar rgb[3]);
public:
	//for debug	
	void print();
};

template<class MAP_POINT>
void vecMapPt2Mat(const vector<MAP_POINT*> & mptPts, Mat_d& matPts) {
	int npts = (int) mptPts.size();
	matPts.resize(npts, 3);
	for (int i = 0; i < npts; ++i) {
		matPts.data[3 * i] = mptPts[i]->x;
		matPts.data[3 * i + 1] = mptPts[i]->y;
		matPts.data[3 * i + 2] = mptPts[i]->z;
	}

}
#endif /* SL_MAPPOINT_H_ */
