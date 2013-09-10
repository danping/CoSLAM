/*
 * SL_IntraCamPose.h
 *
 *  Created on: 2011-5-17
 *      Author: tsou
 */

#ifndef SL_INTRACAMPOSE_H_
#define SL_INTRACAMPOSE_H_

#include "math/SL_LinAlg.h"
#include "geometry/SL_Geometry.h"
#include "geometry/SL_Triangulate.h"

#include <cmath>
#include <cassert>
#include <cstdio>

class IntraCamPoseOption {
public:
	int maxIterLM;
	int maxIterRW;
	double epsErrorChangeLM;
	double epsParamChangeLM;
	double epsErrorChangeRW;
	int verboseLM;
	int verboseRW;
public:
	double lambda0;
	double lambda;

	double err0;
	double err;
	double errRW;

	int retTypeLM;
	int npts;

	int nIterLM;
	int nIterRW;
public:
	IntraCamPoseOption() :
			maxIterLM(100), maxIterRW(5), epsErrorChangeLM(1e-7), epsParamChangeLM(
					1e-6), epsErrorChangeRW(1e-6), verboseLM(0), verboseRW(0), lambda0(
					1e-3) {
	}
	void printLM() {
		printf("lambda:%lf -> %lf\n", lambda0, lambda);
		printf("ssd: %lf -> %lf\n", err0, err);
		printf("err: %lf -> %lf\n", sqrt(err0 / npts), sqrt(err / npts));
		printf("npts:%d\n", npts);
		printf("return type:%d\n", retTypeLM);
		printf("number of LM interation:%d\n", nIterLM);
	}
	void printRW() {
	}
};

/**
 * convert parameters in Lie algebra (w) to rotation matrix
 */
void getSO3ExpMap(const double w[3], double R[9]);

/*
 * R_New <--- R_Old * exp(w)
 * T_New <---- T_Old + t
 */

void intraCamUpdatePose(const double ROld[9], const double tOld[3],
		const double param[6], double RNew[9], double tNew[3]);

/*
 * The main process of levenberg-marquart optimization (without weights)
 */
bool intraCamLMProc(const double K[9], const double R0[9], const double t0[3],
		int npts, const double Ms[], const double ms[], double R_opt[9],
		double t_opt[3], IntraCamPoseOption* opt);

/**
 * The main process of levenberg-marquart optimization (with weights)
 */
bool intraCamWeightedLMProc(const double K[9], const double R0[9],
		const double t0[3], int npts, const double Ws[], const double Ms[],
		const double ms[], double R_opt[9], double t_opt[3],
		IntraCamPoseOption* opt);

/**
 * intra-camera pose estimation using M-estimator (Tukey's estimator)
 * R0, t0 : initial camera pose
 * errs : reprojection error from the previous frame
 */
bool intraCamEstimate(const double K[9], const double R0[9], const double t0[3],
		int npts, const double prevErrs[], const double Ms[], const double ms[],
		const double tau, double R_opt[9], double t_opt[3],
		IntraCamPoseOption* opt);
/**
 * intra-camera pose estimation using M-estimator (Tukey's estimator)
 * R0, t0 : initial camera pose
 * errs : reprojection error from the previous frame
 * covs : covariance matrix of the point
 */
bool intraCamCovEstimate(const double K[9], const double R0[9],
		const double t0[3], int npts, const double prevErrs[],
		const double covs[], const double Ms[], const double ms[],
		const double tau, double R_opt[9], double t_opt[3],
		IntraCamPoseOption* opt);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//intra-camera pose estimation using both 3D-2D and 2D-2D correspondences

bool intraCamLMEpiProc(const double K[9], const double invK[9],
		const double R0[9], const double t0[3], int n3D2D, const double Ms[],
		const double ms[], int n2D2D, const double Rpre[], const double tpre[],
		const double umspre[], const double ums[], double R_opt[9],
		double t_opt[3], IntraCamPoseOption* opt);

bool intraCamEstimateEpi(const double K[9], const double R0[9],
		const double t0[3], int n3D2Ds, const double preRepErrs[],
		const double Ms[], const double ms[],

		int n2D2Ds, const double preEpiErrs[], const double Rpre[],
		const double tpre[], const double umspre[], const double ums[],

		const double tau, double R_opt[9], double t_opt[3],
		IntraCamPoseOption* opt);
#endif /* SL_INTRACAMPOSE_H_ */
