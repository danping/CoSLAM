/*
 * SL_CamPoseEstimator.h
 *
 *  Created on: 2011-1-22
 *      Author: Danping Zou
 */

#ifndef SL_CAMPOSEESTIMATOR_H
#define SL_CAMPOSEESTIMATOR_H
#include "math/SL_Matrix.h"
#include "geometry/SL_BundleHelper.h"
#include "extern/sba-1.6/sba.h"
#include <vector>

/*
 * camera pose estimation using sba method
 * This method is abandoned
 * use 'intraCamEstimate' method in 'geometry/SL_IntraCamPose.h' instead.
 */
class IntraCamPoseEstimatorParam {
public:
	int maxIter;
	int maxIterEachStep;
	double epsilon;
	double sigma;
	double opts[SBA_OPTSSZ];
public:
	IntraCamPoseEstimatorParam() {
		maxIter = 20;
		maxIterEachStep = 40;
		epsilon = 1e-6;
		sigma = 5.0;

		opts[0] = SBA_INIT_MU * 1e-3;
		opts[1] = SBA_STOP_THRESH * 1e-3;
		opts[2] = SBA_STOP_THRESH * 1e-3;
		opts[3] = SBA_STOP_THRESH * 1e-3;
	}
};
/* intra-camera pose estimator */
class IntraCamPoseEstimator: public IntraCamPoseEstimatorParam {
protected:
	const double* pK;
	const double* pMs;
	const double* pms;
	const double* pR0;
	const double* pT0;
	sbaGlobs globs;
	int numPts;
	Mat_c vmask;
	Mat_d param_vec;
	Mat_d meas_vec;
	Mat_d wt_vec;

	Mat_d m_extraW;
public:
	Mat_d R;
	Mat_d T;
	double info_data[SBA_INFOSZ];
public:
	IntraCamPoseEstimator();
	~IntraCamPoseEstimator();
protected:
	void computeExtraWeights(int nPts, const double* nVisCams, const double* nNeighNums, const double* nStaticFrms, Mat_d& extraW);
public:
	void setIntrinsicMat(const double* K) {
		pK = K;
	}
	void setCorresponds(const Mat_d& Ms, const Mat_d& ms) {
		numPts = Ms.rows;
		pMs = Ms.data;
		pms = ms.data;
	}
	/* nPts : number of correspondences
	 * Ms : 3D points
	 * ms : 2D points
	 * errs : previous re-projection error
	 * nVisCams : number of visible cameras
	 * nStaticFrms : number of frames for keeping static
	 */
	void setCorresponds(const Mat_d& Ms, const Mat_d& ms, const double* errs, const double* nVisCams, const double* nStaticFrms);

	void setInitialPose(const double* R0, const double* T0) {
		pR0 = R0;
		pT0 = T0;
	}
	void apply();
	bool oneStep();
};

class FeaturePoint;
/* compute the number of feature poins in a neighborhood centered at ms[i]*/
void computeNeighborNum(const Mat_d& ms, Mat_d& neighNums, double hw);
void computeDensityWeights(const Mat_d& ms, Mat_d& ws, double hw);
void computeDensityWeights(const std::vector<FeaturePoint*>& ms, Mat_d& ws, double hw);

#endif /* SL_CAMPOSEESTIMATOR_H */
