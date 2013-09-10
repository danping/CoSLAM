/*
 * SL_SlovePnP.h
 *
 *  Created on: 2011-1-22
 *      Author: Danping Zou
 */

#include "SL_SolvePnP.h"
#include "geometry/SL_Quaternion.h"
#include "geometry/SL_Distortion.h"
#include "geometry/SL_BundleHelper.h"
#include "SL_error.h"
#include <cstring>
#include <cassert>

void solvePnPIter(
		int npts ,
		const double* ms ,
		const double* cov ,
		const double* Ms ,
		const double* R0 ,
		const double* t0 ,
		double* R ,
		double* t ,
		int maxIter ,
		double* err) {

	assert(npts > 0);
	//set the parameters
	const int cnp = 6;
	const int pnp = 3;
	const int mnp = 2;

	sbaGlobs globs;
	globs.cnp = cnp;
	globs.pnp = pnp;
	globs.mnp = mnp;

	globs.rot0params = new double[FULLQUATSZ];

	mat2quat(R0, globs.rot0params);

	globs.camparams = 0;

	globs.intrcalib = new double[5];
	globs.intrcalib[0] = 1;
	globs.intrcalib[1] = 0;
	globs.intrcalib[2] = 0;
	globs.intrcalib[3] = 1;
	globs.intrcalib[4] = 0;
	globs.nccalib = 5;

	double opts[SBA_OPTSSZ];

	opts[0] = SBA_INIT_MU * 1e-3;
	opts[1] = SBA_STOP_THRESH * 1e-3;
	opts[2] = SBA_STOP_THRESH * 1e-3;
	opts[3] = SBA_STOP_THRESH * 1e-3;
	//opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
	opts[4] = 0.0;

	char* vmask = new char[npts * 2];
	memset(vmask, 1, sizeof(char) * npts * 2);

	//fill camera parameters
	double* param_vec = new double[cnp + npts * pnp];
	param_vec[0] = 0;
	param_vec[1] = 0;
	param_vec[2] = 0;
	param_vec[3] = t0[0];
	param_vec[4] = t0[1];
	param_vec[5] = t0[2];

	//fill point parameters
	int i;
	double* ppar = param_vec + 6;
	const double* pM = Ms;
	for (i = 0; i < npts; ++i) {
		*ppar = *pM;
		*(ppar + 1) = *(pM + 1);
		*(ppar + 2) = *(pM + 2);
		pM += 3;
		ppar += 3;
	}

	globs.ptparams = param_vec + 6;

	//fill the measurement vector
	double* meas_vec = new double[npts * mnp];
	double* pmeas = meas_vec;
	const double* pm0 = ms;
	for (i = 0; i < npts; ++i) {
		*pmeas = *pm0;
		*(pmeas + 1) = *(pm0 + 1);
		pmeas += 2;
		pm0 += 2;
	}

	double info_data[SBA_INFOSZ];

	if (cov) {
		double* covx_vec = new double[npts * mnp * mnp];
		double* pcovx_vec = covx_vec;
		const double* pcov = cov;
		for (i = 0; i < npts; i++) {
			pcovx_vec[0] = *pcov;
			pcovx_vec[1] = 0;
			pcovx_vec[2] = 0;
			pcovx_vec[3] = *pcov;
			pcovx_vec += 4;
			pcov++;
		}
		sba_mot_levmar_x(
				npts,
				1,
				0,
				vmask,
				param_vec,
				cnp,
				meas_vec,
				covx_vec,
				mnp,
				img_projsRT_x,
				img_projsRT_jac_x,
				&globs,
				maxIter,
				0,
				opts,
				info_data);
		delete[] covx_vec;
	} else
		sba_mot_levmar_x(
				npts,
				1,
				0,
				vmask,
				param_vec,
				cnp,
				meas_vec,
				0,
				mnp,
				img_projsRT_x,
				img_projsRT_jac_x,
				&globs,
				maxIter,
				0,
				opts,
				info_data);

	if (err) {
		err[0] = sqrt(info_data[0] / npts);
		err[1] = sqrt(info_data[1] / npts);
	}
	//test
	logInfo("initial error:%lf, final error:%lf #iterations:%lf stop reason:%lf\n", sqrt(info_data[0] / npts), sqrt(
			info_data[1] / npts), info_data[5], info_data[6]);
	//copy back the results	
	double dq[4], q[4];
	_MK_QUAT_FRM_VEC(dq, param_vec);
	quatMultFast(dq, globs.rot0params, q);
	quat2mat(q, R);

	t[0] = param_vec[3];
	t[1] = param_vec[4];
	t[2] = param_vec[5];

	delete[] meas_vec;
	delete[] param_vec;
	delete[] vmask;
	delete[] globs.rot0params;
	delete[] globs.intrcalib;
}

void solvePnPIter(
		const double* K ,
		int npts ,
		const double* ms ,
		const double* cov ,
		const double* Ms ,
		const double* R0 ,
		const double* t0 ,
		double* R ,
		double* t ,
		int maxIter ,
		double* err) {

	assert(npts > 0);
	//set the parameters
	const int cnp = 6;
	const int pnp = 3;
	const int mnp = 2;

	sbaGlobs globs;
	globs.cnp = cnp;
	globs.pnp = pnp;
	globs.mnp = mnp;

	globs.rot0params = new double[FULLQUATSZ];

	mat2quat(R0, globs.rot0params);

	globs.camparams = 0;

	globs.intrcalib = new double[5];
	globs.intrcalib[0] = K[0];
	globs.intrcalib[1] = K[2];
	globs.intrcalib[2] = K[5];
	globs.intrcalib[3] = K[4] / K[0];
	globs.intrcalib[4] = K[1];
	globs.nccalib = 5;

	double opts[SBA_OPTSSZ];

	opts[0] = SBA_INIT_MU * 1e-3;
	opts[1] = SBA_STOP_THRESH * 1e-3;
	opts[2] = SBA_STOP_THRESH * 1e-3;
	opts[3] = SBA_STOP_THRESH * 1e-3;
	//opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
	opts[4] = 0.0;

	char* vmask = new char[npts * 2];
	memset(vmask, 1, sizeof(char) * npts * 2);

	//fill camera parameters
	double* param_vec = new double[cnp + npts * pnp];
	param_vec[0] = 0;
	param_vec[1] = 0;
	param_vec[2] = 0;
	param_vec[3] = t0[0];
	param_vec[4] = t0[1];
	param_vec[5] = t0[2];

	//fill point parameters
	int i;
	double* ppar = param_vec + 6;
	const double* pM = Ms;
	for (i = 0; i < npts; ++i) {
		*ppar = *pM;
		*(ppar + 1) = *(pM + 1);
		*(ppar + 2) = *(pM + 2);
		pM += 3;
		ppar += 3;
	}

	globs.ptparams = param_vec + 6;

	//fill the measurement vector
	double* meas_vec = new double[npts * mnp];
	double* pmeas = meas_vec;
	const double* pm0 = ms;
	for (i = 0; i < npts; ++i) {
		*pmeas = *pm0;
		*(pmeas + 1) = *(pm0 + 1);
		pmeas += 2;
		pm0 += 2;
	}

	double info_data[SBA_INFOSZ];

	if (cov) {
		double* covx_vec = new double[npts * mnp * mnp];
		double* pcovx_vec = covx_vec;
		const double* pcov = cov;
		for (i = 0; i < npts; i++) {
			pcovx_vec[0] = *pcov;
			pcovx_vec[1] = 0;
			pcovx_vec[2] = 0;
			pcovx_vec[3] = *pcov;
			pcovx_vec += 4;
			pcov++;
		}
		sba_mot_levmar_x(
				npts,
				1,
				0,
				vmask,
				param_vec,
				cnp,
				meas_vec,
				covx_vec,
				mnp,
				img_projsRT_x,
				img_projsRT_jac_x,
				&globs,
				maxIter,
				0,
				opts,
				info_data);
		delete[] covx_vec;
	} else
		sba_mot_levmar_x(
				npts,
				1,
				0,
				vmask,
				param_vec,
				cnp,
				meas_vec,
				0,
				mnp,
				img_projsRT_x,
				img_projsRT_jac_x,
				&globs,
				maxIter,
				0,
				opts,
				info_data);

	if (err) {
		err[0] = sqrt(info_data[0] / npts);
		err[1] = sqrt(info_data[1] / npts);
	}
	//test
	logInfo("initial error:%lf, final error:%lf #iterations:%lf stop reason:%lf\n", sqrt(info_data[0] / npts), sqrt(
			info_data[1] / npts), info_data[5], info_data[6]);
	//copy back the results	
	double dq[4], q[4];
	_MK_QUAT_FRM_VEC(dq, param_vec);
	quatMultFast(dq, globs.rot0params, q);
	quat2mat(q, R);

	t[0] = param_vec[3];
	t[1] = param_vec[4];
	t[2] = param_vec[5];

	delete[] meas_vec;
	delete[] param_vec;
	delete[] vmask;
	delete[] globs.rot0params;
	delete[] globs.intrcalib;
}

void solvePnPIter(const double* K , //intrinsic parameters
		const double* kc , //distortion parameters
		int npts ,
		const double* ms ,
		const double* cov ,
		const double* Ms ,
		const double* R0 ,
		const double* t0 ,
		double* R ,
		double* t ,
		int maxIter ,
		double* err) {

	assert(npts > 0);
	//set the parameters
	const int cnp = 16;
	const int pnp = 3;
	const int mnp = 2;

	sbaGlobs globs;
	globs.cnp = cnp;
	globs.pnp = pnp;
	globs.mnp = mnp;

	globs.rot0params = new double[FULLQUATSZ];

	mat2quat(R0, globs.rot0params);

	globs.camparams = 0;

	globs.intrcalib = new double[5];
	//	globs.intrcalib[0] = K[0];
	//	globs.intrcalib[1] = K[2];
	//	globs.intrcalib[2] = K[5];
	//	globs.intrcalib[3] = K[4] / K[0];
	//	globs.intrcalib[4] = K[1];
	globs.intrcalib = 0;
	globs.nccalib = 5;
	globs.ncdist = 5;

	double opts[SBA_OPTSSZ];

	opts[0] = SBA_INIT_MU * 1e-3;
	opts[1] = SBA_STOP_THRESH * 1e-3;
	opts[2] = SBA_STOP_THRESH * 1e-3;
	opts[3] = SBA_STOP_THRESH * 1e-3;
	//opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
	opts[4] = 0.0;

	char* vmask = new char[npts * 2];
	memset(vmask, 1, sizeof(char) * npts * 2);

	//fill camera parameters
	double* param_vec = new double[cnp + npts * pnp];
	param_vec[0] = K[0];
	param_vec[1] = K[2];
	param_vec[2] = K[5];
	param_vec[3] = K[4] / K[0];
	param_vec[4] = K[1];
	param_vec[5] = kc[0];
	param_vec[6] = kc[1];
	param_vec[7] = kc[2];
	param_vec[8] = kc[3];
	param_vec[9] = kc[4];
	param_vec[10] = 0;
	param_vec[11] = 0;
	param_vec[12] = 0;
	param_vec[13] = t0[0];
	param_vec[14] = t0[1];
	param_vec[15] = t0[2];

	//fill point parameters
	int i;
	double* ppar = param_vec + 6;
	const double* pM = Ms;
	for (i = 0; i < npts; ++i) {
		*ppar = *pM;
		*(ppar + 1) = *(pM + 1);
		*(ppar + 2) = *(pM + 2);
		pM += 3;
		ppar += 3;
	}

	globs.ptparams = param_vec + 6;

	//fill the measurement vector
	double* meas_vec = new double[npts * mnp];
	double* pmeas = meas_vec;
	const double* pm0 = ms;
	for (i = 0; i < npts; ++i) {
		*pmeas = *pm0;
		*(pmeas + 1) = *(pm0 + 1);
		pmeas += 2;
		pm0 += 2;
	}

	double info_data[SBA_INFOSZ];

	if (cov) {
		double* covx_vec = new double[npts * mnp * mnp];
		double* pcovx_vec = covx_vec;
		const double* pcov = cov;
		for (i = 0; i < npts; i++) {
			pcovx_vec[0] = *pcov;
			pcovx_vec[1] = 0;
			pcovx_vec[2] = 0;
			pcovx_vec[3] = *pcov;
			pcovx_vec += 4;
			pcov++;
		}
		sba_mot_levmar_x(
				npts,
				1,
				0,
				vmask,
				param_vec,
				cnp,
				meas_vec,
				covx_vec,
				mnp,
				img_projsRT_x,
				img_projsRT_jac_x,
				&globs,
				maxIter,
				0,
				opts,
				info_data);
		delete[] covx_vec;
	} else
		sba_mot_levmar_x(
				npts,
				1,
				0,
				vmask,
				param_vec,
				cnp,
				meas_vec,
				0,
				mnp,
				img_projsRT_x,
				img_projsRT_jac_x,
				&globs,
				maxIter,
				0,
				opts,
				info_data);

	if (err) {
		err[0] = sqrt(info_data[0] / npts);
		err[1] = sqrt(info_data[1] / npts);
	}
	//copy back the results	
	double dq[4], q[4];
	_MK_QUAT_FRM_VEC(dq, param_vec);
	quatMultFast(dq, globs.rot0params, q);
	quat2mat(q, R);

	t[0] = param_vec[3];
	t[1] = param_vec[4];
	t[2] = param_vec[5];

	//test
	logInfo("initial error:%lf, final error:%lf #iterations:%lf stop reason:%lf\n", sqrt(info_data[0] / npts), sqrt(
			info_data[1] / npts), info_data[5], info_data[6]);

	delete[] meas_vec;
	delete[] param_vec;
	delete[] vmask;
	delete[] globs.rot0params;
	delete[] globs.intrcalib;
}
