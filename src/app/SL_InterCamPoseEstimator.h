/*
 * SL_InterCamPoseEstimator.h
 *
 *  Created on: 2011-7-26
 *      Author: zou
 */

#ifndef SL_INTERCAMPOSEESTIMATOR_H_
#define SL_INTERCAMPOSEESTIMATOR_H_

#include "SL_CoSLAM.h"
#include "geometry/SL_BundleAdjust.h"

/* inter-camera pose update */
class InterCamPoseEstimatorParam {
public:
	int maxIter;
	int maxIterEachStep;
	double epsilon;
	double sigma;
	double neighborRadius;
public:
	InterCamPoseEstimatorParam() {
		maxIter = 3;
		maxIterEachStep = 40;
		epsilon = 1e-6;
		sigma = 6.0;
		neighborRadius = 150;
	}
};
class InterCamPoseEstimator: public InterCamPoseEstimatorParam {
protected:
	CoSLAM* m_pCoSLAM;
public:
	//Inputs & outputs - R,T
	std::vector<Mat_d> Ks;
	std::vector<Mat_d> Rs;
	std::vector<Mat_d> Ts;
	std::vector<Point3d> vecPts3D;
	std::vector<std::vector<Meas2D> > vecMeas2D;

	int m_numStatic;
	int m_numDynamic;
protected:
	std::vector<MapPoint*> dynamicMapPoints;

public:
	InterCamPoseEstimator() {
	}
	~InterCamPoseEstimator() {
	}
	void setCoSLAM(CoSLAM* coSLAM) {
		m_pCoSLAM = coSLAM;
	}
	void addMapPoints();
	void apply();
};
#endif /* SL_INTERCAMPOSEESTIMATOR_H_ */
