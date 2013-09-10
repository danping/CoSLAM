/*
 * SL_IntraCamPose.cpp
 *
 *  Created on: 2011-5-17
 *      Author: tsou
 */

#include "SL_IntraCamPose.h"

void getSO3ExpMap(const double w[3], double R[9]) {
	double theta = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
	if (theta == 0) {
		memset(R, 0, sizeof(double) * 9);
		R[0] = 1.0;
		R[4] = 1.0;
		R[8] = 1.0;
		return;
	}
	double hw[3] = { w[0] / theta, w[1] / theta, w[2] / theta };
	double st = sin(theta);
	double ct = 1 - cos(theta);

	double hw0hw0 = hw[0] * hw[0];
	double hw0hw1 = hw[0] * hw[1];
	double hw0hw2 = hw[0] * hw[2];
	double hw1hw1 = hw[1] * hw[1];
	double hw1hw2 = hw[1] * hw[2];
	double hw2hw2 = hw[2] * hw[2];
	
	R[0] = -ct * hw1hw1 - ct * hw2hw2 + 1;
	R[1] = ct * hw0hw1 - st * hw[2];
	R[2] = st * hw[1] + ct * hw0hw2;
	R[3] = st * hw[2] + ct * hw0hw1;
	R[4] = -ct * hw0hw0 - ct * hw2hw2 + 1;
	R[5] = ct * hw1hw2 - st * hw[0];
	R[6] = ct * hw0hw2 - st * hw[1];
	R[7] = st * hw[0] + ct * hw1hw2;
	R[8] = -ct * hw0hw0 - ct * hw1hw1 + 1;
}
/*
 * use numerical differentiation to compute the Jacobian of rotation parameters
 */
static void _perspectiveSO3JacobiNum(const double K[9],
		const double R[9],
		const double t[3],
		const double M[3],
		const double m[2],
		const double rm0[2],
		double Jw[6]) {

	double w[3] = { 0, 0, 0 };
	double rm[2];

	double dR[9], R1[9];

	const double eps = 1e-8;
	w[0] = eps;
	getSO3ExpMap(w, dR);
	mat33AB(R, dR, R1);
	project(K, R1, t, M, rm);

	Jw[0] = (rm[0] - rm0[0]) / eps;
	Jw[3] = (rm[1] - rm0[1]) / eps;

	w[0] = 0;
	w[1] = eps;
	getSO3ExpMap(w, dR);
	mat33AB(R, dR, R1);
	project(K, R1, t, M, rm);

	Jw[1] = (rm[0] - rm0[0]) / eps;
	Jw[4] = (rm[1] - rm0[1]) / eps;

	w[1] = 0;
	w[2] = eps;
	getSO3ExpMap(w, dR);
	mat33AB(R, dR, R1);
	project(K, R1, t, M, rm);

	Jw[2] = (rm[0] - rm0[0]) / eps;
	Jw[5] = (rm[1] - rm0[1]) / eps;
}

/**
 * use numerical differentiation to compute the Jacobian of translation parameters
 */
static void _perspectiveTJacobiNum(const double K[9],
		const double R[9],
		const double t[3],
		const double M[3],
		const double m[2],
		const double rm0[2],
		double Jt[6]) {

	double rm[2];

	const double eps = 1e-8;
	double t1[3] = { t[0] + eps, t[1], t[2] };
	project(K, R, t1, M, rm);

	Jt[0] = (rm[0] - rm0[0]) / eps;
	Jt[3] = (rm[1] - rm0[1]) / eps;

	t1[0] = t[0];
	t1[1] = t[1] + eps;
	project(K, R, t1, M, rm);

	Jt[1] = (rm[0] - rm0[0]) / eps;
	Jt[4] = (rm[1] - rm0[1]) / eps;

	t1[1] = t[1];
	t1[2] = t[2] + eps;
	project(K, R, t1, M, rm);

	Jt[2] = (rm[0] - rm0[0]) / eps;
	Jt[5] = (rm[1] - rm0[1]) / eps;
}

/**
 * compute levenberg-marquardt step 
 */
inline void mat66sum(const double* A, const double* B, double* C) {
	C[0] = A[0] + B[0];
	C[1] = A[1] + B[1];
	C[2] = A[2] + B[2];
	C[3] = A[3] + B[3];
	C[4] = A[4] + B[4];
	C[5] = A[5] + B[5];
	C[6] = A[6] + B[6];
	C[7] = A[7] + B[7];
	C[8] = A[8] + B[8];
	C[9] = A[9] + B[9];
	C[10] = A[10] + B[10];
	C[11] = A[11] + B[11];
	C[12] = A[12] + B[12];
	C[13] = A[13] + B[13];
	C[14] = A[14] + B[14];
	C[15] = A[15] + B[15];
	C[16] = A[16] + B[16];
	C[17] = A[17] + B[17];
	C[18] = A[18] + B[18];
	C[19] = A[19] + B[19];
	C[20] = A[20] + B[20];
	C[21] = A[21] + B[21];
	C[22] = A[22] + B[22];
	C[23] = A[23] + B[23];
	C[24] = A[24] + B[24];
	C[25] = A[25] + B[25];
	C[26] = A[26] + B[26];
	C[27] = A[27] + B[27];
	C[28] = A[28] + B[28];
	C[29] = A[29] + B[29];
	C[30] = A[30] + B[30];
	C[31] = A[31] + B[31];
	C[32] = A[32] + B[32];
	C[33] = A[33] + B[33];
	C[34] = A[34] + B[34];
	C[35] = A[35] + B[35];
}
inline void mat66scale(double A[], double scale) {
	A[0] *= scale;
	A[1] *= scale;
	A[2] *= scale;
	A[3] *= scale;
	A[4] *= scale;
	A[5] *= scale;
	A[6] *= scale;
	A[7] *= scale;
	A[8] *= scale;
	A[9] *= scale;
	A[10] *= scale;
	A[11] *= scale;
	A[12] *= scale;
	A[13] *= scale;
	A[14] *= scale;
	A[15] *= scale;
	A[16] *= scale;
	A[17] *= scale;
	A[18] *= scale;
	A[19] *= scale;
	A[20] *= scale;
	A[21] *= scale;
	A[22] *= scale;
	A[23] *= scale;
	A[24] *= scale;
	A[25] *= scale;
	A[26] *= scale;
	A[27] *= scale;
	A[28] *= scale;
	A[29] *= scale;
	A[30] *= scale;
	A[31] *= scale;
	A[32] *= scale;
	A[33] *= scale;
	A[34] *= scale;
	A[35] *= scale;
}
inline void mat16sum(const double* A, const double* B, double* C) {
	C[0] = A[0] + B[0];
	C[1] = A[1] + B[1];
	C[2] = A[2] + B[2];
	C[3] = A[3] + B[3];
	C[4] = A[4] + B[4];
	C[5] = A[5] + B[5];
}
inline void mat16scale(double A[], double scale) {
	A[0] *= scale;
	A[1] *= scale;
	A[2] *= scale;
	A[3] *= scale;
	A[4] *= scale;
	A[5] *= scale;
}
/**
 * compute a new parameter for each iterative step
 */
static void intraCamLMStep(const double K[9], const double R[9], //current parameter
		const double t[3],
		int npts,
		const double Ms[], //corresponding 3D-2D points
		const double ms[],
		double param[6], //new parameter
		const double lambda //damping coefficient
) {

	double sA[36], A[36], invSA[36], sB[6], B[6], rm[2], rerr[2];
	double Jw[6], Jt[6];
	memset(sA, 0, sizeof(double) * 36);
	memset(sB, 0, sizeof(double) * 6);

	const double* pMs = Ms;
	const double* pms = ms;
	for (int i = 0; i < npts; i++, pMs += 3, pms += 2) {
		project(K, R, t, pMs, rm);
		_perspectiveSO3JacobiNum(K, R, t, pMs, pms, rm, Jw);
		_perspectiveTJacobiNum(K, R, t, pMs, pms, rm, Jt);

		double J[12] = { Jw[0], Jw[1], Jw[2], Jt[0], Jt[1], Jt[2], Jw[3], Jw[4], Jw[5], Jt[3], Jt[4], Jt[5] };
		matATB(2, 6, 2, 6, J, J, A);
		mat66sum(sA, A, sA);

		rerr[0] = -rm[0] + pms[0];
		rerr[1] = -rm[1] + pms[1];

		matATB(2, 6, 2, 1, J, rerr, B);
		mat16sum(sB, B, sB);
	}

	sA[0] += lambda;
	sA[7] += lambda;
	sA[14] += lambda;
	sA[21] += lambda;
	sA[28] += lambda;
	sA[35] += lambda;

	matInv(6, sA, invSA);
	matAB(6, 6, 6, 1, invSA, sB, param);
}
static void intraCamWeightedLMStep(const double K[9],
		const double R[9],
		const double t[3],
		int npts,
		const double Ws[],
		const double Ms[],
		const double ms[],
		double param[6],
		const double lambda) {

	double sA[36], A[36], invSA[36], sB[6], B[6], rm[2], rerr[2];
	double Jw[6], Jt[6];
	memset(sA, 0, sizeof(double) * 36);
	memset(sB, 0, sizeof(double) * 6);

	const double* pMs = Ms;
	const double* pms = ms;
	for (int i = 0; i < npts; i++, pMs += 3, pms += 2) {
		project(K, R, t, pMs, rm);
		_perspectiveSO3JacobiNum(K, R, t, pMs, pms, rm, Jw);
		_perspectiveTJacobiNum(K, R, t, pMs, pms, rm, Jt);

		double J[12] = { Ws[i] * Jw[0], Ws[i] * Jw[1], Ws[i] * Jw[2], Ws[i] * Jt[0], Ws[i] * Jt[1], Ws[i] * Jt[2],
				Ws[i] * Jw[3], Ws[i] * Jw[4], Ws[i] * Jw[5], Ws[i] * Jt[3], Ws[i] * Jt[4], Ws[i] * Jt[5] };

		matATB(2, 6, 2, 6, J, J, A);
		mat66sum(sA, A, sA);

		rerr[0] = (-rm[0] + pms[0]) * Ws[i];
		rerr[1] = (-rm[1] + pms[1]) * Ws[i];

		matATB(2, 6, 2, 1, J, rerr, B);
		mat16sum(sB, B, sB);
	}

	sA[0] += lambda;
	sA[7] += lambda;
	sA[14] += lambda;
	sA[21] += lambda;
	sA[28] += lambda;
	sA[35] += lambda;

	matInv(6, sA, invSA);
	matAB(6, 6, 6, 1, invSA, sB, param);
}

static void intraCamCovWeightedLMStep(const double K[9],
		const double R[9],
		const double t[3],
		int npts,
		const double covs[],
		const double Ws[],
		const double Ms[],
		const double ms[],
		double param[6],
		const double lambda) {

	double sA[36], A[36], invSA[36], sB[6], B[6], rm[2], rerr[2];
	double Jw[6], Jt[6];
	memset(sA, 0, sizeof(double) * 36);
	memset(sB, 0, sizeof(double) * 6);

	const double* pMs = Ms;
	const double* pcovs = covs;
	const double* pms = ms;
	for (int i = 0; i < npts; i++, pMs += 3, pcovs += 9, pms += 2) {
		double var[4], ivar[4];
		project(K, R, t, pMs, rm);
		getProjectionCovMat(K, R, t, pMs, pcovs, var, 1.0);
		mat22Inv(var, ivar);
		_perspectiveSO3JacobiNum(K, R, t, pMs, pms, rm, Jw);
		_perspectiveTJacobiNum(K, R, t, pMs, pms, rm, Jt);

		//J:\in R ^{2x6}
		double J[12] = { Jw[0], Jw[1], Jw[2], Jt[0], Jt[1], Jt[2], Jw[3], Jw[4], Jw[5], Jt[3], Jt[4], Jt[5] };

		//\sum_i w[i]*(J_\omega^T *ivar[i] * J_\omega)
		double Jivar[12];
		matATB(2, 6, 2, 2, J, ivar, Jivar);
		matAB(6, 2, 2, 6, Jivar, J, A);
		mat66scale(A, Ws[i]);
		mat66sum(sA, A, sA);

		//-\sum_i w[i]*( J_\omega^T *ivar[i] *\epsilon_i)
		rerr[0] = (-rm[0] + pms[0]);
		rerr[1] = (-rm[1] + pms[1]);

		matAB(6, 2, 2, 1, Jivar, rerr, B);
		mat16scale(B, Ws[i]);
		mat16sum(sB, B, sB);
	}

	sA[0] += lambda;
	sA[7] += lambda;
	sA[14] += lambda;
	sA[21] += lambda;
	sA[28] += lambda;
	sA[35] += lambda;

	matInv(6, sA, invSA);
	matAB(6, 6, 6, 1, invSA, sB, param);
}

/*
 * R_New <--- R_Old * exp(w)
 * T_New <---- T_Old + t
 */

void intraCamUpdatePose(const double ROld[9],
		const double tOld[3],
		const double param[6],
		double RNew[9],
		double tNew[3]) {

	double dR[9];
	getSO3ExpMap(param, dR);
	mat33AB(ROld, dR, RNew);

	tNew[0] = tOld[0] + param[3];
	tNew[1] = tOld[1] + param[4];
	tNew[2] = tOld[2] + param[5];
}

/*
 * The main process of levenberg-marquart optimization
 */
bool intraCamLMProc(const double K[9],
		const double R0[9],
		const double t0[3],
		int npts,
		const double Ms[],
		const double ms[],
		double R_opt[9],
		double t_opt[3],
		IntraCamPoseOption* opt) {
	assert(opt);
	double param[6];

	//compute the reprojection error
	opt->npts = npts;
	opt->lambda = opt->lambda0;
	opt->err0 = reprojError2(K, R0, t0, npts, Ms, ms);
	opt->err = opt->err0;
	if (opt->verboseLM > 0)
		printf("[%d]err:%lf, lambda:%lf\n", -1, opt->err, opt->lambda);

	double R[9], t[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	opt->retTypeLM = 1;
	int i = 0;
	for (; i < opt->maxIterLM; i++) {
		intraCamLMStep(K, R, t, npts, Ms, ms, param, opt->lambda);
		intraCamUpdatePose(R, t, param, R_opt, t_opt);
		double err = reprojError2(K, R_opt, t_opt, npts, Ms, ms);
		if (opt->verboseLM > 0)
			printf("[%d]err:%lf, lambda:%lf\n", i, err, opt->lambda);
		if (fabs(err - opt->err) < opt->epsErrorChangeLM) {
			opt->retTypeLM = 0;
			break;
		}
		if (err < opt->err) {
			doubleArrCopy(R, 0, R_opt, 9);
			doubleArrCopy(t, 0, t_opt, 3);
			opt->err = err;
			opt->lambda /= 10;
		} else {
			opt->lambda *= 10;
			if (opt->lambda > 1e+4) {
				//cannot find the optimum 
				opt->retTypeLM = -1;
				break;
			}
		}
	}
	opt->nIterLM = i;
	return opt->retTypeLM >= 0;
}

static double reprojError2Weighted(const double* K,
		const double* R,
		const double* t,
		int npts,
		const double* Ws,
		const double* Ms,
		const double* ms) {
	double rm[2];
	double err = 0;
	int i;
	for (i = 0; i < npts; ++i) {
		project(K, R, t, Ms + 3 * i, rm);
		double dx = ms[2 * i] - rm[0];
		double dy = ms[2 * i + 1] - rm[1];
		err += (dx * dx + dy * dy) * Ws[i];
	}
	return err;
}
static double reprojError2CovWeighted(const double* K,
		const double* R,
		const double* t,
		int npts,
		const double* covs,
		const double* Ws,
		const double* Ms,
		const double* ms) {
	double rm[2], var[4], ivar[4];
	double err = 0;
	for (int i = 0; i < npts; ++i) {
		project(K, R, t, Ms + 3 * i, rm);
		getProjectionCovMat(K, R, t, Ms + 3 * i, covs + 9 * i, var, 1.0);
		mat22Inv(var, ivar);
		err += mahaDist2(rm, ms + 2 * i, ivar);
	}
	return err;
}
bool intraCamWeightedLMProc(const double K[9],
		const double R0[9],
		const double t0[3],
		int npts,
		const double Ws[],
		const double Ms[],
		const double ms[],
		double R_opt[9],
		double t_opt[3],
		IntraCamPoseOption* opt) {
	assert(opt);
	double param[6];

	//compute the reprojection error
	opt->npts = npts;
	opt->lambda = opt->lambda0;
	opt->err0 = reprojError2Weighted(K, R0, t0, npts, Ws, Ms, ms);
	opt->err = opt->err0;
	if (opt->verboseLM > 0)
		printf("[%d]err:%lf, lambda:%lf, (%lf %lf %lf %lf %lf %lf)\n", -1, opt->err, opt->lambda, param[0], param[1],
				param[2], param[3], param[4], param[5]);

	double R[9], t[3], R_tmp[9], t_tmp[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	opt->retTypeLM = 1;
	int i = 0;
	double err = opt->err0;
	for (; i < opt->maxIterLM; i++) {
		intraCamWeightedLMStep(K, R, t, npts, Ws, Ms, ms, param, opt->lambda);
		intraCamUpdatePose(R, t, param, R_opt, t_opt);

		if (param[0] * param[0] + param[1] * param[1] + param[2] * param[2] + param[3] * param[3] + param[4] * param[4]
				+ param[5] * param[5] < opt->epsParamChangeLM) {
			doubleArrCopy(R, 0, R_opt, 9);
			doubleArrCopy(t, 0, t_opt, 3);
			opt->retTypeLM = 0;
			break;
		}

		err = reprojError2Weighted(K, R_opt, t_opt, npts, Ws, Ms, ms);
		if (opt->verboseLM > 0)
			printf("[%d]err:%lf, lambda:%lf, (%lf %lf %lf %lf %lf %lf)\n", i, err, opt->lambda, param[0], param[1],
					param[2], param[3], param[4], param[5]);
		if (fabs(err - opt->err) < opt->epsErrorChangeLM) {
			opt->retTypeLM = 0;
			break;
		}
		if (err <= opt->err) {
			doubleArrCopy(R, 0, R_opt, 9);
			doubleArrCopy(t, 0, t_opt, 3);
			doubleArrCopy(R_tmp, 0, R_opt, 9);
			doubleArrCopy(t_tmp, 0, t_opt, 3);
			opt->err = err;
			opt->lambda /= 10;
		} else {
			opt->lambda *= 10;
			if (opt->lambda > 1e+18) {
				//cannot find the optimum 
				opt->retTypeLM = -1;
				break;
			}
		}
	}
	if (opt->retTypeLM == -1) {
		doubleArrCopy(R_opt, 0, R_tmp, 9);
		doubleArrCopy(t_opt, 0, t_tmp, 3);
	}
	opt->err = err;
	opt->nIterLM = i;
	if (opt->verboseLM)
		opt->printLM();
	return opt->retTypeLM >= 0;
}
bool intraCamCovWeightedLMProc(const double K[9],
		const double R0[9],
		const double t0[3],
		int npts,
		const double covs[],
		const double Ws[],
		const double Ms[],
		const double ms[],
		double R_opt[9],
		double t_opt[3],
		IntraCamPoseOption* opt) {
	assert(opt);
	double param[6];

	//compute the reprojection error
	opt->npts = npts;
	opt->lambda = opt->lambda0;
	opt->err0 = reprojError2CovWeighted(K, R0, t0, npts, covs, Ws, Ms, ms);
	opt->err = opt->err0;
	if (opt->verboseLM > 0)
		printf("[%d]err:%lf, lambda:%lf, (%lf %lf %lf %lf %lf %lf)\n", -1, opt->err, opt->lambda, param[0], param[1],
				param[2], param[3], param[4], param[5]);

	double R[9], t[3], R_tmp[9], t_tmp[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	opt->retTypeLM = 1;
	int i = 0;
	double err = opt->err0;
	for (; i < opt->maxIterLM; i++) {
		intraCamCovWeightedLMStep(K, R, t, npts, covs, Ws, Ms, ms, param, opt->lambda);
		intraCamUpdatePose(R, t, param, R_opt, t_opt);

		if (param[0] * param[0] + param[1] * param[1] + param[2] * param[2] + param[3] * param[3] + param[4] * param[4]
				+ param[5] * param[5] < opt->epsParamChangeLM) {
			doubleArrCopy(R, 0, R_opt, 9);
			doubleArrCopy(t, 0, t_opt, 3);
			opt->retTypeLM = 0;
			break;
		}

		err = reprojError2CovWeighted(K, R_opt, t_opt, npts, covs, Ws, Ms, ms);
		if (opt->verboseLM > 0)
			printf("[%d]err:%lf, lambda:%lf, (%lf %lf %lf %lf %lf %lf)\n", i, err, opt->lambda, param[0], param[1],
					param[2], param[3], param[4], param[5]);
		if (fabs(err - opt->err) < opt->epsErrorChangeLM) {
			opt->retTypeLM = 0;
			break;
		}
		if (err <= opt->err) {
			doubleArrCopy(R, 0, R_opt, 9);
			doubleArrCopy(t, 0, t_opt, 3);
			doubleArrCopy(R_tmp, 0, R_opt, 9);
			doubleArrCopy(t_tmp, 0, t_opt, 3);
			opt->err = err;
			opt->lambda /= 10;
		} else {
			opt->lambda *= 10;
			if (opt->lambda > 1e+18) {
				//cannot find the optimum 
				opt->retTypeLM = -1;
				break;
			}
		}
	}
	if (opt->retTypeLM == -1) {
		doubleArrCopy(R_opt, 0, R_tmp, 9);
		doubleArrCopy(t_opt, 0, t_tmp, 3);
	}
	opt->err = err;
	opt->nIterLM = i;
	if (opt->verboseLM)
		opt->printLM();
	return opt->retTypeLM >= 0;
}
bool intraCamEstimate(const double K[9],
		const double R0[9],
		const double t0[3],
		int npts,
		const double prevErrs[],
		const double Ms[],
		const double ms[],
		const double tau,
		double R_opt[9],
		double t_opt[3],
		IntraCamPoseOption* opt) {

	if (opt->verboseRW)
		printf("=============start of intra-camera pose estimation ===========\n");

	double* Ws = new double[npts];
	for (int i = 0; i < npts; i++) {
		if (prevErrs == 0)
			Ws[i] = 1.0;
		else {
			double e = fabs(prevErrs[i]);
			if (e >= tau) {
				Ws[i] = 0;
			} else {
				e /= tau;
				e = 1 - e * e;
				Ws[i] = e * e;
			}
		}
	}

	double R[9], t[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	bool ret = true;
	int k = 0;
	opt->errRW = -1;
	for (; k < opt->maxIterRW; k++) {
		if (!intraCamWeightedLMProc(K, R, t, npts, Ws, Ms, ms, R_opt, t_opt, opt)) {
			ret = false;
			goto returnPoint;
		}
		opt->lambda0 = opt->lambda;
		if (opt->errRW < 0)
			opt->errRW = opt->err;
		else {
			if (fabs(opt->err - opt->errRW) < opt->epsErrorChangeRW) {
				ret = true;
				goto returnPoint;
			} else
				opt->errRW = opt->err;
		}

		if (opt->verboseRW) {
			printf(">>rw:[%d]:err:%lf\n", k, opt->errRW);
		}

		doubleArrCopy(R, 0, R_opt, 9);
		doubleArrCopy(t, 0, t_opt, 3);

		//compute the new weights
		for (int i = 0; i < npts; i++) {
			double rm[2];
			project(K, R, t, Ms + 3 * i, rm);
			double dx = rm[0] - ms[2 * i];
			double dy = rm[1] - ms[2 * i + 1];
			double e = sqrt(dx * dx + dy * dy);
			if (e >= tau)
				Ws[i] = 0;
			else {
				e /= tau;
				e = 1 - e * e;
				Ws[i] = e * e;
			}
		}
	}
	returnPoint: opt->nIterRW = k;
	delete[] Ws;

	if (opt->verboseRW)
		printf("-------------end of intra-camera pose estimation ---------\n");
	return ret;
}

bool intraCamCovEstimate(const double K[9],
		const double R0[9],
		const double t0[3],
		int npts,
		const double prevErrs[],
		const double covs[],
		const double Ms[],
		const double ms[],
		const double tau,
		double R_opt[9],
		double t_opt[3],
		IntraCamPoseOption* opt) {

	if (opt->verboseRW)
		printf("=============start of intra-camera pose estimation ===========\n");

	double* Ws = new double[npts];
	for (int i = 0; i < npts; i++) {
		if (prevErrs == 0)
			Ws[i] = 1.0;
		else {
			double e = fabs(prevErrs[i]);
			if (e >= tau) {
				Ws[i] = 0;
			} else {
				e /= tau;
				e = 1 - e * e;
				Ws[i] = e * e;
			}
		}
	}

	double R[9], t[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	bool ret = true;
	int k = 0;
	opt->errRW = -1;
	for (; k < opt->maxIterRW; k++) {
		if (!intraCamCovWeightedLMProc(K, R, t, npts, covs, Ws, Ms, ms, R_opt, t_opt, opt)) {
			ret = false;
			goto returnPoint;
		}
		opt->lambda0 = opt->lambda;
		if (opt->errRW < 0)
			opt->errRW = opt->err;
		else {
			if (fabs(opt->err - opt->errRW) < opt->epsErrorChangeRW) {
				ret = true;
				goto returnPoint;
			} else
				opt->errRW = opt->err;
		}

		if (opt->verboseRW) {
			printf(">>rw:[%d]:err:%lf\n", k, opt->errRW);
		}

		doubleArrCopy(R, 0, R_opt, 9);
		doubleArrCopy(t, 0, t_opt, 3);

		//compute the new weights
		for (int i = 0; i < npts; i++) {
			double rm[2];
			project(K, R, t, Ms + 3 * i, rm);
			double dx = rm[0] - ms[2 * i];
			double dy = rm[1] - ms[2 * i + 1];
			double e = sqrt(dx * dx + dy * dy);
			if (e >= tau)
				Ws[i] = 0;
			else {
				e /= tau;
				e = 1 - e * e;
				Ws[i] = e * e;
			}
		}
	}
	returnPoint: opt->nIterRW = k;
	delete[] Ws;

	if (opt->verboseRW)
		printf("-------------end of intra-camera pose estimation ---------\n");
	return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//intra-camera pose estimation using both 3D-2D and 2D-2D correspondences
#include "geometry/SL_5point.h"
#include "geometry/SL_FundamentalMatrix.h"

static void _epiDistSO3JacobiNum(const double invK[9],
		const double Rpre[9],
		const double Tpre[3],
		const double mpre[2],
		const double R[9],
		const double T[3],
		const double m[2],
		const double err0,
		double Jw[3]) {

	double w[3] = { 0, 0, 0 };
	double dR[9], R1[9], E[9], F[9];

	const double eps = 1e-16;
	w[0] = eps;
	getSO3ExpMap(w, dR);
	mat33AB(R, dR, R1);

	formEMat(Rpre, Tpre, R1, T, E);
	getFMat(invK, invK, E, F);

	double err = epipolarError(F, m, mpre);
	Jw[0] = (err - err0) / eps;

	w[0] = 0;
	w[1] = eps;
	getSO3ExpMap(w, dR);
	mat33AB(R, dR, R1);

	formEMat(Rpre, Tpre, R1, T, E);
	getFMat(invK, invK, E, F);

	err = epipolarError(F, m, mpre);
	Jw[1] = (err - err0) / eps;

	w[1] = 0;
	w[2] = eps;
	getSO3ExpMap(w, dR);
	mat33AB(R, dR, R1);

	formEMat(Rpre, Tpre, R1, T, E);
	getFMat(invK, invK, E, F);

	err = epipolarError(F, m, mpre);
	Jw[2] = (err - err0) / eps;
}

static void _epiDistTJacobiNum(const double invK[9],
		const double Rpre[9],
		const double Tpre[3],
		const double mpre[2],
		const double R[9],
		const double T[3],
		const double m[2],
		const double err0,
		double Jt[3]) {

	double E[9], F[9];
	const double eps = 1e-16;
	double T1[3] = { T[0] + eps, T[1], T[2] };

	formEMat(Rpre, Tpre, R, T1, E);
	getFMat(invK, invK, E, F);

	double err = epipolarError(F, m, mpre);
	Jt[0] = (err - err0) / eps;

	T1[0] = T[0];
	T1[1] = T[1] + eps;

	formEMat(Rpre, Tpre, R, T1, E);
	getFMat(invK, invK, E, F);

	err = epipolarError(F, m, mpre);
	Jt[1] = (err - err0) / eps;

	T1[1] = T[1];
	T1[2] = T[2] + eps;

	formEMat(Rpre, Tpre, R, T1, E);
	getFMat(invK, invK, E, F);

	err = epipolarError(F, m, mpre);
	Jt[2] = (err - err0) / eps;
}

static void intraCamEpiLMStep(const double K[9], const double invK[9], const double R[9], //current parameter (rotation and translation)
		const double t[3],
		int n2D3Ds, //number of 2D-3D correspondences
		const double Ms[], //3D points
		const double ms[], //2D points
		int n2D2Ds, //number of 2D-2D correspondences
		const double uRpre[], //camera poses at former frames
		const double utpre[],
		const double umspre[], //unmapped feature points at former frames
		const double ums[], //unmapped feature points the current frames
		double param[6], //new parameter
		const double lambda // damping coefficient
) {

	double sA[36], A[36], invSA[36], sB[6], B[6], rm[2], rerr[2], eerr;
	double Jw[6], Jt[6];

	memset(sA, 0, sizeof(double) * 36);
	memset(sB, 0, sizeof(double) * 6);

	const double* pMs = Ms;
	const double* pms = ms;
	for (int i = 0; i < n2D3Ds; i++, pMs += 3, pms += 2) {
		project(K, R, t, pMs, rm);
		_perspectiveSO3JacobiNum(K, R, t, pMs, pms, rm, Jw);
		_perspectiveTJacobiNum(K, R, t, pMs, pms, rm, Jt);

		double J[12] = { Jw[0], Jw[1], Jw[2], Jt[0], Jt[1], Jt[2], Jw[3], Jw[4], Jw[5], Jt[3], Jt[4], Jt[5] };
		matATB(2, 6, 2, 6, J, J, A);
		mat66sum(sA, A, sA);

		rerr[0] = -rm[0] + pms[0];
		rerr[1] = -rm[1] + pms[1];

		matATB(2, 6, 2, 1, J, rerr, B);
		mat16sum(sB, B, sB);
	}

	const double* pms1 = umspre;
	const double* pms2 = ums;
	const double* pRpre = uRpre;
	const double* ptpre = utpre;

	for (int j = 0; j < n2D2Ds; j++, pms1 += 2, pms2 += 2, pRpre += 9, ptpre += 3) {
		//compute the epipolar error
		double E[9], F[9], Ew[3], Et[3];
		formEMat(R, t, pRpre, ptpre, E);
		getFMat(invK, invK, E, F);
		eerr = epipolarError(F, pms2, pms1);

		_epiDistSO3JacobiNum(invK, pRpre, ptpre, pms1, R, t, pms2, eerr, Ew);
		_epiDistTJacobiNum(invK, pRpre, ptpre, pms1, R, t, pms2, eerr, Et);

		double J[6] = { Ew[0], Ew[1], Ew[2], Et[0], Et[1], Et[2] };
		matATB(1, 6, 1, 6, J, J, A);
		mat66sum(sA, A, sA);

		B[0] = -J[0] * eerr;
		B[1] = -J[1] * eerr;
		B[2] = -J[2] * eerr;
		B[3] = -J[3] * eerr;
		B[4] = -J[4] * eerr;
		B[5] = -J[5] * eerr;

		mat16sum(sB, B, sB);
	}

	sA[0] += lambda;
	sA[7] += lambda;
	sA[14] += lambda;
	sA[21] += lambda;
	sA[28] += lambda;
	sA[35] += lambda;

	matInv(6, sA, invSA);
	matAB(6, 6, 6, 1, invSA, sB, param);
}

double _epiError2(const double invK[],
		int n2D2D,
		const double Rpre[],
		const double tpre[],
		const double umspre[],
		const double R[],
		const double t[],
		const double ums[]) {

	const double* pms1 = umspre;
	const double* pms2 = ums;
	const double* pRpre = Rpre;
	const double* ptpre = tpre;

	double s = 0;
	for (int j = 0; j < n2D2D; j++, pms1 += 2, pms2 += 2, pRpre += 9, ptpre += 3) {
		//compute the epipolar error
		double E[9], F[9];
		formEMat(R, t, pRpre, ptpre, E);
		getFMat(invK, invK, E, F);
		double err = epipolarError(F, pms2, pms1);
		s += err * err;
	}
	return s;
}
double _epiError2Weighted(const double invK[],
		int n2D2D,
		const double uWs[],
		const double Rpre[],
		const double tpre[],
		const double umspre[],
		const double R[],
		const double t[],
		const double ums[]) {

	const double* pms1 = umspre;
	const double* pms2 = ums;
	const double* pRpre = Rpre;
	const double* ptpre = tpre;

	double E[9], F[9], l[3];
	formEMat(R, t, pRpre, ptpre, E);
	getFMat(invK, invK, E, F);

	double s = 0;
	for (int j = 0; j < n2D2D; j++, pms1 += 2, pms2 += 2) {
		computeEpipolarLine(F, pms1[0], pms1[1], l);

		double A = l[0];
		double B = l[1];
		double C = l[2];

		double x0 = pms2[0];
		double y0 = pms2[1];

		double lambda = (-A * x0 - B * y0 - C);
		double err2 = lambda * lambda / (A * A + B * B);

		s += err2 * uWs[j];
	}
	return s;
}
/*
 * The main process of levenberg-marquart optimization
 */
bool intraCamLMEpiProc(const double K[9],
		const double invK[9],
		const double R0[9],
		const double t0[3],
		int n3D2D,
		const double Ms[],
		const double ms[],
		int n2D2D,
		const double Rpre[],
		const double tpre[],
		const double umspre[],
		const double ums[],
		double R_opt[9],
		double t_opt[3],
		IntraCamPoseOption* opt) {
	assert(opt);
	double param[6];

	//compute the reprojection error
	opt->npts = n3D2D;
	opt->lambda = opt->lambda0;
	opt->err0 = reprojError2(K, R0, t0, n3D2D, Ms, ms) + _epiError2(invK, n2D2D, Rpre, tpre, umspre, R0, t0, ums);
	opt->err = opt->err0;
	if (opt->verboseLM > 0)
		printf("[%d]err:%lf, lambda:%lf\n", -1, opt->err, opt->lambda);

	double R[9], t[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	opt->retTypeLM = 1;
	int i = 0;
	for (; i < opt->maxIterLM; i++) {
		intraCamEpiLMStep(K, invK, R, t, n3D2D, Ms, ms, n2D2D, Rpre, tpre, umspre, ums, param, opt->lambda);
		intraCamUpdatePose(R, t, param, R_opt, t_opt);
		double err = reprojError2(K, R_opt, t_opt, n3D2D, Ms, ms) + _epiError2(invK, n2D2D, Rpre, tpre, umspre, R_opt,
				t_opt, ums);
		if (opt->verboseLM > 0)
			printf("[%d]err:%lf, lambda:%lf\n", i, err, opt->lambda);
		if (fabs(err - opt->err) < opt->epsErrorChangeLM) {
			opt->retTypeLM = 0;
			break;
		}
		if (err < opt->err) {
			doubleArrCopy(R, 0, R_opt, 9);
			doubleArrCopy(t, 0, t_opt, 3);
			opt->err = err;
			opt->lambda /= 10;
		} else {
			opt->lambda *= 10;
			if (opt->lambda > 1e+4) {
				//cannot find the optimum 
				opt->retTypeLM = -1;
				break;
			}
		}
	}
	opt->nIterLM = i;
	return opt->retTypeLM >= 0;
}

static void intraCamEpiWeightedLMStep(const double K[9], const double invK[9], const double R[9], //current parameter (rotation and translation)
		const double t[3],
		int n2D3Ds, //number of 2D-3D correspondences
		const double Ws[],
		const double Ms[], //3D points
		const double ms[], //2D points
		int n2D2Ds, //number of 2D-2D correspondences
		const double uRpre[], //camera poses at former frames
		const double utpre[],
		const double uWs[],
		const double umspre[], //unmapped feature points at former frames
		const double ums[], //unmapped feature points the current frames
		double param[6], //new parameter
		const double lambda // damping coefficient
) {
	double sA[36], A[36], invSA[36], sB[6], B[6], rm[2], rerr[2], eerr;
	double Jw[6], Jt[6];

	memset(sA, 0, sizeof(double) * 36);
	memset(sB, 0, sizeof(double) * 6);

	const double* pMs = Ms;
	const double* pms = ms;
	for (int i = 0; i < n2D3Ds; i++, pMs += 3, pms += 2) {
		project(K, R, t, pMs, rm);
		_perspectiveSO3JacobiNum(K, R, t, pMs, pms, rm, Jw);
		_perspectiveTJacobiNum(K, R, t, pMs, pms, rm, Jt);

		double J[12] = { Ws[i] * Jw[0], Ws[i] * Jw[1], Ws[i] * Jw[2], Ws[i] * Jt[0], Ws[i] * Jt[1], Ws[i] * Jt[2],
				Ws[i] * Jw[3], Ws[i] * Jw[4], Ws[i] * Jw[5], Ws[i] * Jt[3], Ws[i] * Jt[4], Ws[i] * Jt[5] };

		matATB(2, 6, 2, 6, J, J, A);
		mat66sum(sA, A, sA);

		rerr[0] = (-rm[0] + pms[0]) * Ws[i];
		rerr[1] = (-rm[1] + pms[1]) * Ws[i];

		matATB(2, 6, 2, 1, J, rerr, B);
		mat16sum(sB, B, sB);
	}

	const double* pms1 = umspre;
	const double* pms2 = ums;
	const double* pRpre = uRpre;
	const double* ptpre = utpre;

	for (int j = 0; j < n2D2Ds; j++, pms1 += 2, pms2 += 2, pRpre += 9, ptpre += 3) {
		//compute the epipolar error
		double E[9], F[9], Ew[3], Et[3];
		formEMat(pRpre, ptpre, R, t, E);
		getFMat(invK, invK, E, F);
		eerr = epipolarError(F, pms2, pms1) * uWs[j];

		_epiDistSO3JacobiNum(invK, pRpre, ptpre, pms1, R, t, pms2, eerr, Ew);
		_epiDistTJacobiNum(invK, pRpre, ptpre, pms1, R, t, pms2, eerr, Et);

		double J[6] =
				{ Ew[0] * uWs[j], Ew[1] * uWs[j], Ew[2] * uWs[j], Et[0] * uWs[j], Et[1] * uWs[j], Et[2] * uWs[j] };

		matATB(1, 6, 1, 6, J, J, A);
		mat66sum(sA, A, sA);

		B[0] = -J[0] * eerr;
		B[1] = -J[1] * eerr;
		B[2] = -J[2] * eerr;
		B[3] = -J[3] * eerr;
		B[4] = -J[4] * eerr;
		B[5] = -J[5] * eerr;

		mat16sum(sB, B, sB);
	}

	sA[0] += lambda;
	sA[7] += lambda;
	sA[14] += lambda;
	sA[21] += lambda;
	sA[28] += lambda;
	sA[35] += lambda;

	matInv(6, sA, invSA);
	matAB(6, 6, 6, 1, invSA, sB, param);
}
bool intraCamLMEpiWeightedProc(const double K[9],
		const double invK[9],
		const double R0[9],
		const double t0[3],
		int n3D2D,
		const double Ws[],
		const double Ms[],
		const double ms[],
		int n2D2D,
		const double Rpre[],
		const double tpre[],
		const double uWs[],
		const double umspre[],
		const double ums[],
		double R_new[9],
		double t_new[3],
		IntraCamPoseOption* opt) {
	assert(opt);
	double param[6];

	//compute the reprojection error
	opt->npts = n3D2D;
	opt->lambda = opt->lambda0;

	double err1 = reprojError2Weighted(K, R0, t0, n3D2D, Ws, Ms, ms);
	double err2 = _epiError2Weighted(invK, n2D2D, uWs, Rpre, tpre, umspre, R0, t0, ums);

	opt->err0 = err1 + err2;
	opt->err = opt->err0;

	if (opt->verboseLM > 0) {
		printf("[%d]err:%lf (%lf,%lf), lambda:%lf\n", -1, opt->err, err1, err2, opt->lambda);
	}

	double R[9], t[3], R_opt[9], t_opt[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	opt->retTypeLM = 1;
	int i = 0;
	for (; i < opt->maxIterLM; i++) {
		intraCamEpiWeightedLMStep(K, invK, R, t, n3D2D, Ws, Ms, ms, n2D2D, Rpre, tpre, uWs, umspre, ums, param,
				opt->lambda);
		intraCamUpdatePose(R, t, param, R_new, t_new);

		err1 = reprojError2Weighted(K, R_new, t_new, n3D2D, Ws, Ms, ms);
		err2 = _epiError2Weighted(invK, n2D2D, uWs, Rpre, tpre, umspre, R_new, t_new, ums);

		double err = err1 + err2;

		if (opt->verboseLM > 0)
			printf("[%d]err:%lf (%lf,%lf), lambda:%lf\n", i, err, err1, err2, opt->lambda);

		if (fabs(err - opt->err) < opt->epsErrorChangeLM) {
			opt->retTypeLM = 0;
			break;
		}
		if (err < opt->err) {
			//accept the new parameter
			doubleArrCopy(R, 0, R_new, 9);
			doubleArrCopy(t, 0, t_new, 3);
			doubleArrCopy(R_opt, 0, R_new, 9);
			doubleArrCopy(t_opt, 0, t_new, 3);
			opt->err = err;
			opt->lambda /= 10;
		} else {
			opt->lambda *= 100;
			if (opt->lambda > 1e+4) {
				//cannot find the optimum 
				opt->retTypeLM = -1;
				break;
			}
		}
	}
	//	if (opt->retTypeLM == -1) {
	//		doubleArrCopy(R_new, 0, R_opt, 9);
	//		doubleArrCopy(t_new, 0, t_opt, 3);
	//	}
	opt->nIterLM = i;
	return opt->retTypeLM >= 0;
}

bool intraCamEstimateEpi(const double K[9],
		const double R0[9],
		const double t0[3],
		int n3D2Ds,
		const double preRepErrs[],
		const double Ms[],
		const double ms[],

		int n2D2Ds,
		const double preEpiErrs[],
		const double Rpre[],
		const double tpre[],
		const double umspre[],
		const double ums[],

		const double tau,
		double R_opt[9],
		double t_opt[3],
		IntraCamPoseOption* opt) {

	double invK[9];
	mat33Inv(K, invK);

	if (opt->verboseRW)
		printf("=============start of intra-camera pose estimation ===========\n");

	double* Ws = new double[n3D2Ds];
	for (int i = 0; i < n3D2Ds; i++) {
		if (preRepErrs == 0)
			Ws[i] = 1.0;
		else {
			double e = fabs(preRepErrs[i]);
			if (e >= tau) {
				Ws[i] = 0;
			} else {
				e /= tau;
				e = 1 - e * e;
				Ws[i] = e * e;
			}
		}
	}
	double* uWs = new double[n2D2Ds];
	for (int j = 0; j < n2D2Ds; j++) {
		if (preEpiErrs == 0) {
			uWs[j] = 1.0;
		} else {
			double e = fabs(preEpiErrs[j]);
			if (e >= tau) {
				uWs[j] = 0;
			} else {
				e /= tau;
				e = 1 - e * e;
				uWs[j] = e * e;
			}
		}
	}

	double R[9], t[3];
	doubleArrCopy(R, 0, R0, 9);
	doubleArrCopy(t, 0, t0, 3);

	bool ret = true;
	int k = 0;
	opt->errRW = -1;
	for (; k < opt->maxIterRW; k++) {
		intraCamLMEpiWeightedProc(K, invK, R, t, n3D2Ds, Ws, Ms, ms, n2D2Ds, Rpre, tpre, uWs, umspre, ums, R_opt,
				t_opt, opt);

		opt->lambda0 = opt->lambda;
		if (opt->errRW < 0)
			opt->errRW = opt->err;
		else {
			if (fabs(opt->err - opt->errRW) < opt->epsErrorChangeRW) {
				ret = true;
				goto returnPoint;
			} else
				opt->errRW = opt->err;
		}
		if (opt->verboseRW) {
			printf(">> rw:[%d]:err:%lf\n", k, opt->errRW);
		}

		doubleArrCopy(R, 0, R_opt, 9);
		doubleArrCopy(t, 0, t_opt, 3);

		//compute the new weights
		for (int i = 0; i < n3D2Ds; i++) {
			double rm[2];
			project(K, R, t, Ms + 3 * i, rm);
			double dx = rm[0] - ms[2 * i];
			double dy = rm[1] - ms[2 * i + 1];
			double e = sqrt(dx * dx + dy * dy);
			if (e >= tau)
				Ws[i] = 0;
			else {
				e /= tau;
				e = 1 - e * e;
				Ws[i] = e * e;
			}
		}
	}
	returnPoint: opt->nIterRW = k;
	delete[] Ws;

	if (opt->verboseRW) {
		printf("err:%lf-->%lf\n", opt->err0, opt->err);
		printf("-------------end of intra-camera pose estimation ---------\n");
	}
	return ret;
}
