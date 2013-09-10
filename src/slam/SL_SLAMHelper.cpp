/*
 * SL_Pnp.cpp
 *
 *  Created on: 2011-1-17
 *      Author: Danping Zou
 */

#include "SL_SLAMHelper.h"
#include "SL_IntraCamPose.h"
#include "math/SL_LinAlg.h"
#include "geometry/SL_5point.h"
#include "geometry/SL_Triangulate.h"

#include <cfloat>
static int evaluatePose(const double* K, const double* R, const double* t,
		int nPts, const double* pts3d, const double* pts2d, double errThes,
		double* errReturn, unsigned char* inlierFlag) {

	Mat_d tmpPts2D(nPts, 2);
	project(K, R, t, nPts, pts3d, tmpPts2D);

	int inlierNum = 0;
	*errReturn = 0;
	for (int i = 0; i < nPts; i++) {
		double d = dist2(tmpPts2D + 2 * i, pts2d + 2 * i);
		if (d < errThes) {
			//inlier
			inlierNum++;
			*errReturn += d;
			if (inlierFlag)
				inlierFlag[i] = true;
		} else if (inlierFlag) {
			inlierFlag[i] = false;
		}
	}
	return inlierNum;
}
int solvePnPRansac(int numPts, const double* pts3d, const double* pts2d,
		const double* K, double* R, double* t, int iterMax, double errThres,
		double* errReturn, unsigned char* inlierFlag) {

	int indices[5];
	double sampPts3D[15];
	double sampPts2D[10];
	double tR[9], tT[3];

	double errMin = DBL_MAX;
	int nMaxInlier = 0;

	for (int iter = 0; iter < iterMax; iter++) {
		randChoose(numPts, indices, 5);
		for (int i = 0; i < 5; i++) {
			sampPts3D[3 * i] = pts3d[3 * indices[i]];
			sampPts3D[3 * i + 1] = pts3d[3 * indices[i] + 1];
			sampPts3D[3 * i + 2] = pts3d[3 * indices[i] + 2];

			sampPts2D[2 * i] = pts2d[2 * indices[i]];
			sampPts2D[2 * i + 1] = pts2d[2 * indices[i] + 1];
		}
		solvePnP(5, sampPts3D, sampPts2D, K, tR, tT);
		double err;
		int inlierNum = evaluatePose(K, tR, tT, numPts, pts3d, pts2d, errThres,
				&err, 0);
		if (inlierNum > nMaxInlier
				|| (inlierNum == nMaxInlier && err < errMin)) {
			nMaxInlier = inlierNum;
			memcpy(R, tR, 9 * sizeof(double));
			memcpy(t, tT, 3 * sizeof(double));
		}
		if (numPts * 0.8 < inlierNum)
			break;
	}

	if (inlierFlag)
		nMaxInlier = evaluatePose(K, tR, tT, numPts, pts3d, pts2d, errThres,
				&errMin, inlierFlag);

	if (errReturn)
		*errReturn = errMin;

	return nMaxInlier;
}
#ifdef USE_OPENCV_PNP
#include "opencv2/calib3d/calib3d.hpp"
#include "cvHelper.h"
#include "SL_Debug.h"
void solvePnP(int numPoints , const double* pts3d , const double* pts2d , const double* K , double* R , double* t) {
	//	printMat(numPts, 3, pts3d);
	//	printMat(numPts, 2, pts2d);

	cv::Mat cvPts3D(numPoints, 3, CV_64FC1, (void*) pts3d);
	cv::Mat cvPts2D(numPoints, 2, CV_64FC1, (void*) pts2d);
	cv::Mat cvK(3, 3, CV_64FC1, (void*) K);
	cv::Mat cvRVec(3, 1, CV_64FC1);
	cv::Mat cvTVec(3, 1, CV_64FC1, t);

	//	//test
	//	printMat(3, 3, K);
	CvMat rPts3D = cvPts3D;
	CvMat rPts2D = cvPts2D;
	CvMat rK = cvK;
	CvMat rRVec = cvRVec;
	CvMat rTVec = cvTVec;

	cvFindExtrinsicCameraParams2(&rPts3D, &rPts2D, &rK, 0, &rRVec, &rTVec, false);

	cv::Mat cvRM(3, 3, CV_64FC1, R);
	cv::Rodrigues(cvRVec, cvRM);
}
#else
#include "geometry/epnp.h"
void solvePnP(int numPts, const double* pts3d, const double* pts2d,
		const double* K, double* R, double* t) {
	epnp pnp;
	pnp.set_internal_parameters(K[2], K[5], K[0], K[4]);
	pnp.set_maximum_number_of_correspondences(numPts * 2);
	pnp.reset_correspondences();
	for (int i = 0; i < numPts; i++) {
		pnp.add_correspondence(pts3d[3 * i], pts3d[3 * i + 1], pts3d[3 * i + 2],
				pts2d[2 * i], pts2d[2 * i + 1]);
	}

	double R_est[3][3], t_est[3];
	pnp.compute_pose(R_est, t_est);
	memcpy(R, &R_est[0][0], 9 * sizeof(double));
	memcpy(t, t_est, 3 * sizeof(double));
}
#endif

void solvePnP(const Mat_d& pts3d, const Mat_d& pts2d, const double* K, Mat_d& R,
		Mat_d& t) {
	if (pts3d.rows != pts2d.rows)
		repErr(
				"solvePnP - the number of corresponding points should be the same.");
	int numPts = pts3d.rows;

	R.resize(3, 3);
	t.resize(3, 1);

	solvePnP(numPts, pts3d, pts2d, K, R, t);
}

void solvePnPRobust(int numPts, const double* pts3d, const double* pts2d,
		const double* K, double* R, double* t) {
	double cR[9], ct[3];
	solvePnP(numPts, pts3d, pts2d, K, cR, ct);

	IntraCamPoseOption opt;
	intraCamEstimate(K, cR, ct, numPts, 0, pts3d, pts2d, 20.0, R, t, &opt);
}

void solvePnPRobust(const Mat_d& pts3d, const Mat_d& pts2d, const double* K,
		Mat_d& R, Mat_d& t) {
	assert(pts3d.m > 0 && pts3d.m == pts2d.m);
	double cR[9], ct[3];
	solvePnP(pts3d.rows, pts3d, pts2d, K, cR, ct);

	R.resize(3, 3);
	t.resize(3, 1);
	IntraCamPoseOption opt;
	intraCamEstimate(K, cR, ct, pts3d.rows, 0, pts3d.data, pts2d.data, 6.0,
			R.data, t.data, &opt);
}
void solvePnPRobust(const Mat_d& pts3d, const Mat_d& covs, const Mat_d& pts2d,
		const double* K, Mat_d& R, Mat_d& t) {
	assert(pts3d.m > 0 && pts3d.m == pts2d.m);
	assert(pts3d.m > 0 && pts3d.m == pts2d.m);
	double cR[9], ct[3];
	solvePnP(pts3d.rows, pts3d, pts2d, K, cR, ct);

	R.resize(3, 3);
	t.resize(3, 1);
	IntraCamPoseOption opt;
	intraCamCovEstimate(K, cR, ct, pts3d.rows, 0, covs.data, pts3d.data,
			pts2d.data, 6.0, R.data, t.data, &opt);
}
void getCamCoords(const CamPoseItem* cam, double* p0, double* xp, double* yp,
		double* zp) {
	mat33TransProdVec(cam->R, cam->t, p0);
	p0[0] = -p0[0];
	p0[1] = -p0[1];
	p0[2] = -p0[2];

	xp[0] = cam->R[0];
	xp[1] = cam->R[1];
	xp[2] = cam->R[2];

	yp[0] = cam->R[3];
	yp[1] = cam->R[4];
	yp[2] = cam->R[5];

	zp[0] = cam->R[6];
	zp[1] = cam->R[7];
	zp[2] = cam->R[8];
}

void getCamCenter(const CamPoseItem* cam, double org[]) {
	getCameraCenter(cam->R, cam->t, org);
}

double getCamDist(const CamPoseItem* cam0, const CamPoseItem* cam1) {
	double org[3], org1[3];
	getCamCenter(cam0, org);
	getCamCenter(cam1, org1);
	return dist3(org, org1);
}

double getViewAngleChange(const double center[3], const CamPoseItem* cam1,
		const CamPoseItem* cam2) {
	const double PI = 3.14;
	double org1[3], org2[3];
	getCamCenter(cam1, org1);
	getCamCenter(cam2, org2);

	double r = getAbsRadiansBetween(center, org1, org2);
	return r / PI * 180.0;
}
