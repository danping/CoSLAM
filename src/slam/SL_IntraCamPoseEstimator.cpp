/*
 * SL_CamPoseEstimator.cpp
 *
 *  Created on: 2011-1-22
 *      Author: Danping Zou
 */

#include "SL_error.h"
#include "SL_IntraCamPoseEstimator.h"
#include "matching/SL_StereoMatcherHelper.h"
#include "math/SL_LinAlg.h"

#include "geometry/SL_Geometry.h"
#include "geometry/SL_Quaternion.h"
#include "geometry/SL_Triangulate.h"
#include "SL_FeaturePoint.h"
#include "SL_MapPoint.h"
#include <cassert>

IntraCamPoseEstimator::IntraCamPoseEstimator() {
	globs.intrcalib = 0;
	globs.rot0params = 0;
}

IntraCamPoseEstimator::~IntraCamPoseEstimator() {
	if (globs.intrcalib)
		delete[] globs.intrcalib;
	if (globs.rot0params)
		delete[] globs.rot0params;
}
void IntraCamPoseEstimator::computeExtraWeights(int nPts,
		const double* nVisCams,
		const double* nNeighNums,
		const double* nStaticFrms,
		Mat_d& extraW) {

	double maxStaticFrm = 60;
	extraW.resize(nPts, 1);
	for (int i = 0; i < nPts; i++) {
		double w1 = nVisCams[i] / nNeighNums[i];
		double w2 = (nStaticFrms[i] > maxStaticFrm ? maxStaticFrm : nStaticFrms[i]) / 30;
		extraW.data[i] = w1 + w2;
	}
}
void IntraCamPoseEstimator::setCorresponds(const Mat_d& Ms,
		const Mat_d& ms,
		const double* errs,
		const double* nVisCams,
		const double* nStaticFrms) {

	numPts = Ms.rows;
	pMs = Ms.data;
	pms = ms.data;

	//compute the initial weights based on the previous reprojection errors
	wt_vec.resize(numPts, 1);

	Mat_d neighNums;
	computeNeighborNum(ms, neighNums, 80);
	computeExtraWeights(numPts, nVisCams, neighNums.data, nStaticFrms, m_extraW);

	for (int i = 0; i < numPts; i++) {
		double e = errs[i];
		if (e < 0)
			repErr("The reprojection error is invalid :%lf\n", e);
		if (e >= sigma) {
			wt_vec.data[i] = 0;
		} else {
			e /= sigma;
			double tmp = 1 - e * e;
			double rw = tmp * tmp;
			wt_vec.data[i] = m_extraW.data[i] * rw;
		}
	}
}
void IntraCamPoseEstimator::apply() {
	assert(numPts > 0);
	//set the parameters
	const int cnp = 6;
	const int pnp = 3;
	const int mnp = 2;

	globs.cnp = cnp;
	globs.pnp = pnp;
	globs.mnp = mnp;

	globs.camparams = 0;

	if (!globs.intrcalib)
		globs.intrcalib = new double[5];

	globs.intrcalib[0] = pK[0];
	globs.intrcalib[1] = pK[2];
	globs.intrcalib[2] = pK[5];
	globs.intrcalib[3] = pK[4] / pK[0];
	globs.intrcalib[4] = pK[1];
	globs.nccalib = 5;

	if (!globs.rot0params)
		globs.rot0params = new double[FULLQUATSZ];

	vmask.resize(numPts, 1);
	memset(vmask.data, 1, numPts);

	param_vec.resize(cnp + numPts * pnp, 1);
	meas_vec.resize(numPts, mnp);

	if (wt_vec.empty()) {
		wt_vec.resize(numPts, 1);
		wt_vec.fill(1.0);
	}

	globs.ptparams = param_vec.data + 6;

	//fill the 3D points
	memcpy(param_vec.data + cnp, pMs, sizeof(double) * numPts * pnp);
	//fill the measurement vector
	memcpy(meas_vec.data, pms, sizeof(double) * numPts * mnp);

	R.cloneFrom(pR0, 3, 3);
	T.cloneFrom(pT0, 3, 1);

	Mat_d err(numPts, 1);
	Mat_d reproj_ms(numPts, 2);

	int i;
	for (i = 0; i < maxIter; i++) {
		if (oneStep())
			break;
		if (i == maxIter - 1)
			break;

		//compute the reprojection error
		project(pK, R.data, T.data, numPts, pMs, reproj_ms.data);

		//re-weight
		for (int j = 0; j < numPts; j++) {
			double e = dist2(reproj_ms.data + 2 * j, pms + 2 * j);
			if (e >= sigma) {
				wt_vec.data[j] = 0;
			} else {
				e /= sigma;
				double tmp = 1 - e * e;
				double rw = tmp * tmp;
				wt_vec.data[j] = m_extraW.data[i] * rw;
			}
		}
	}
}
bool IntraCamPoseEstimator::oneStep() {
	const int cnp = 6;
	const int mnp = 2;

	//update the rotation
	mat2quat(R.data, globs.rot0params);

	param_vec.data[0] = 0;
	param_vec.data[1] = 0;
	param_vec.data[2] = 0;
	param_vec.data[3] = T.data[0];
	param_vec.data[4] = T.data[1];
	param_vec.data[5] = T.data[2];

	sba_mot_levmar_wx(numPts, 1, 0, vmask.data, param_vec.data, cnp, meas_vec.data, wt_vec.data, mnp, img_projsRT_x,
			img_projsRT_jac_x, &globs, maxIter, 0, opts, info_data);

	//output the result
	double dq[4], q[4];
	_MK_QUAT_FRM_VEC(dq, param_vec.data);
	quatMultFast(dq, globs.rot0params, q);
	quat2mat(q, R.data);

	T.data[0] = param_vec.data[3];
	T.data[1] = param_vec.data[4];
	T.data[2] = param_vec.data[5];

	double initialErr = sqrt(info_data[0] / numPts);
	double finalErr = sqrt(info_data[1] / numPts);
	//test
//	logInfo("intra-camera pose update : initial error:%lf, final error:%lf #iterations:%lf stop reason:%lf\n",
//			initialErr, finalErr, info_data[5], info_data[6]);

	if (fabs(initialErr - finalErr) <= epsilon)
		return true;
	return false;
}

void computeNeighborNum(const Mat_d& ms, Mat_d& neighNums, double hw) {
	int num = ms.rows;
	Mat_d dMat;
	getDistMat(ms, dMat);

	neighNums.resize(num, 1);
	for (int i = 0; i < num; i++) {
		int n = 0;
		for (int j = 0; j < num; j++) {
			if (dMat.data[i * num + j] <= hw)
				n++;
		}
		neighNums.data[i] = n;
	}
}
void computeDensityWeights(const Mat_d& ms, Mat_d& ws, double hw) {
	int num = ms.rows;
	const double* pm1 = ms.data;
	const double* pm2 = ms.data;

	Mat_d dMat(num, num);
	for (int i = 0; i < num; i++) {
		dMat.data[i * num + i] = 0;
		pm2 = pm1 + 2;
		for (int j = i + 1; j < num; j++) {
			double d = dist2(pm1, pm2);
			dMat.data[i * num + j] = d;
			dMat.data[j * num + i] = d;
			pm2 += 2;
		}
		pm1 += 2;
	}
	ws.resize(num, 1);
	for (int i = 0; i < num; i++) {
		int adjNum = 0;
		for (int j = 0; j < num; j++) {
			if (dMat.data[i * num + j] <= hw)
				adjNum++;
		}
		ws.data[i] = 1.0 / (adjNum * adjNum);
	}

}
void computeDensityWeights(const std::vector<FeaturePoint*>& featPts, Mat_d& ws, double hw) {
	int num = featPts.size();

	Mat_d dMat(num, num);
	dMat.fill(hw + hw);

	for (int i = 0; i < num; i++) {
		if (!featPts[i]->mpt->isLocalStatic())
			continue;
		dMat.data[i * num + i] = 0;
		for (int j = i + 1; j < num; j++) {
			assert(featPts[i] != featPts[j]);
			if (featPts[j]->mpt->isLocalStatic()) {
				double d = dist2(featPts[i]->m, featPts[j]->m);
				dMat.data[i * num + j] = d;
				dMat.data[j * num + i] = d;
			}
		}
	}
	ws.resize(num, 1);
	for (int i = 0; i < num; i++) {
		if (featPts[i]->mpt->isLocalStatic()) {
			int adjNum = 0;
			for (int j = 0; j < num; j++) {
				if (dMat.data[i * num + j] < hw)
					adjNum++;
			}
			ws.data[i] = 1.0 / adjNum;
		} else
			ws.data[i] = -1.0;
	}
}
