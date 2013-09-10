/*
 * SL_CoSLAMHelper.cpp
 *
 *  Created on: 2011-2-20
 *      Author: Danping Zou
 */

#include "SL_CoSLAMHelper.h"
#include "SL_MapPoint.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Geometry.h"
#include "geometry/SL_Distortion.h"
#include "geometry/SL_ConvexHull2D.h"
#include "geometry/SL_Triangulate.h"
#include <algorithm>
static bool keyFrmCompare(KeyPose* kf1, KeyPose* kf2) {
	if (kf1->cam->f < kf2->cam->f)
		return true;
	else if (kf1->cam->f == kf2->cam->f) {
		if (kf1->cam->camId < kf2->cam->camId)
			return true;
	}
	return false;
}
void sortKeyFrms(std::vector<KeyPose*>& keyFrms) {
	std::sort(keyFrms.begin(), keyFrms.end(), keyFrmCompare);
}

double diffAvgGrayLevel(const ImgG& img1, const ImgG& img2, double x1,
		double y1, double x2, double y2, int hw) {
	int ix1 = (int) x1;
	int iy1 = (int) y1;
	int ix2 = (int) x2;
	int iy2 = (int) y2;
	double s1 = 0;
	int k = 0;
	for (int i = -hw; i <= hw; i++) {
		int iyy = iy1 + i;
		if (iyy < 0 || iyy >= img1.rows)
			continue;
		for (int j = -hw; j <= hw; j++) {
			int ixx = ix1 + j;
			if (ixx >= 0 && ixx < img1.cols) {
				s1 += img1.data[iyy * img1.cols + ixx];
				k++;
			}
		}
	}
	s1 /= k;
	double s2 = 0;
	k = 0;
	for (int i = -hw; i <= hw; i++) {
		int iyy = iy2 + i;
		if (iyy < 0 || iyy >= img2.rows)
			continue;
		for (int j = -hw; j <= hw; j++) {
			int ixx = ix2 + j;
			if (ixx >= 0 && ixx < img2.cols) {
				s2 += img2.data[iyy * img2.cols + ixx];
				k++;
			}
		}
	}
	s2 /= k;
	return fabs(s1 - s2);
}
int isStaticRemovable(int numCams, MapPoint* p, double pixelVar, double M[3],
		double cov[9], int numFrame) {
	//find the view with the maximum reprojection error
	int maxI = -1;
	double maxErr = 1.0;
	int nVis = 0;
	for (int i = 0; i < numCams; i++) {
		const FeaturePoint* fp = p->pFeatures[i];
		if (fp) {
			double rm[2], var[4], ivar[4];

			project(fp->K, fp->cam->R, fp->cam->t, p->M, rm);
			getProjectionCovMat(fp->K, fp->cam->R, fp->cam->t, p->M, p->cov,
					var, pixelVar);
			mat22Inv(var, ivar);

			double err = mahaDist2(rm, fp->m, ivar);
			if (err > maxErr) {
				maxErr = err;
				maxI = i;
			}
			nVis++;
		}
	}
	//delete the view with maximum reprojection error 
	//and check whether this point become a static point
	assert(nVis > 0);

	if (maxI < 0)
		return -1;
//	FeaturePoint* fp = p->pFeatures[maxI];
//
//	fp->mpt = 0;
//	p->pFeatures[maxI] = 0;
//	nVis--;
//	p->numVisCam = nVis;
//
//	if (nVis > 1 && isStaticPoint(numCams, p, pixelVar, M, cov)) {
//		return maxI;
//	}

	if (nVis > 2
			&& isStaticPointExclude(numCams, p, pixelVar, M, cov, maxI,
					numFrame)) {
		return maxI;
	}
	return -1;
}

bool isStaticPoint(int numCams, const MapPoint*p, double pixelVar, double M[3],
		double cov[9], int numFrame) {
	double Ks[SLAM_MAX_NUM * 18], Rs[SLAM_MAX_NUM * 18], ts[SLAM_MAX_NUM * 6],
			ms[SLAM_MAX_NUM * 4], nms[SLAM_MAX_NUM * 4];
	int numView = 0;
	int firstFrame = p->lastFrame - numFrame;
	for (int i = 0; i < numCams; i++) {
		const FeaturePoint* fp = p->pFeatures[i];
		if (fp && fp->f >= firstFrame) {
			double iK[9];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, fp->K, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);

			getInvK(fp->K, iK);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);

			const FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp && fp->f >= firstFrame) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, maxFp->K, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);

				getInvK(maxFp->K, iK);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}
	triangulateMultiView(numView, Rs, ts, nms, M);
	getTriangulateCovMat(numView, Ks, Rs, ts, M, cov, pixelVar);
	//check the reprojection error for each view
	for (int i = 0; i < numView; i++) {
		double rm[2], var[4], ivar[4];
		project(Ks + 9 * i, Rs + 9 * i, ts + 3 * i, M, rm);
		getProjectionCovMat(Ks + 9 * i, Rs + 9 * i, ts + 3 * i, M, cov, var,
				pixelVar);
		mat22Inv(var, ivar);
		if (mahaDist2(rm, ms + 2 * i, ivar) > 1.0)
			return false;
	}
	return true;
}

bool isStaticPointExclude(int numCams, const MapPoint* p, double pixelVar,
		double M[3], double cov[9], int viewId, int numFrame) {

	double Ks[SLAM_MAX_NUM * 18], Rs[SLAM_MAX_NUM * 18], ts[SLAM_MAX_NUM * 6],
			ms[SLAM_MAX_NUM * 4], nms[SLAM_MAX_NUM * 4];
	int numView = 0;
	int firstFrame = p->lastFrame - numFrame;
	assert(viewId >= 0 && viewId < numCams);
	for (int i = 0; i < numCams; i++) {
		if (viewId == i)
			continue;
		const FeaturePoint* fp = p->pFeatures[i];
		if (fp && fp->f >= firstFrame) {
			double iK[9];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, fp->K, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);

			getInvK(fp->K, iK);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);

			const FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp && fp->f >= firstFrame) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, maxFp->K, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);

				getInvK(maxFp->K, iK);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}
	triangulateMultiView(numView, Rs, ts, nms, M);
	getTriangulateCovMat(numView, Ks, Rs, ts, M, cov, pixelVar);
	//check the reprojection error for each view
	for (int i = 0; i < numView; i++) {
		double rm[2], var[4], ivar[4];
		project(Ks + 9 * i, Rs + 9 * i, ts + 3 * i, M, rm);
		getProjectionCovMat(Ks + 9 * i, Rs + 9 * i, ts + 3 * i, M, cov, var,
				pixelVar);
		mat22Inv(var, ivar);
		if (mahaDist2(rm, ms + 2 * i, ivar) > 1.0)
			return false;
	}
	return true;

}
bool isDynamicPoint(int numCams, const MapPoint* p, double pixelVar,
		double M[3], double cov[9]) {
	double Ks[SLAM_MAX_NUM * 9], Rs[SLAM_MAX_NUM * 9], Ts[SLAM_MAX_NUM * 3],
			ms[SLAM_MAX_NUM * 2], nms[SLAM_MAX_NUM * 2];

	int numView = 0;
	int nDynamic = 0;
	for (int i = 0; i < numCams; i++) {
		FeaturePoint* fp = p->pFeatures[i];
		if (fp && fp->f == p->lastFrame) {
			double iK[9];
			doubleArrCopy(Ks, numView, fp->K, 9);
			doubleArrCopy(Rs, numView, fp->cam->R, 9);
			doubleArrCopy(Ts, numView, fp->cam->t, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			getInvK(fp->K, iK);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;
			if (fp->type == TYPE_FEATPOINT_DYNAMIC)
				nDynamic++;
		}
	}

	if (numView < 2) {
		return false;
	}

	double org[3];
	getCameraCenter(Rs, Ts, org);

	//retriangulate the 3D coordinates
	triangulateMultiView(numView, Rs, Ts, nms, M);
	if (isAtCameraBack(Rs, Ts, M)) {
		return false;
	}
	getTriangulateCovMat(numView, Ks, Rs, Ts, M, cov, pixelVar);

	//remove dynamic points with large covariance;
	double s = fabs(cov[0]) + fabs(cov[4]) + fabs(cov[8]);
	if (dist3(M, org) * 0.2 < sqrt(s)) {
		return false;
	}

	//check the reprojection error for each view
	for (int i = 0; i < numView; i++) {
		//double err = reprojErrorSingle(Ks + 9 * i, Rs + 9 * i, Ts + 3 * i, M, ms + 2 * i);

		double rm[2], var[4], ivar[4];

		project(Ks + 9 * i, Rs + 9 * i, Ts + 3 * i, M, rm);
		getProjectionCovMat(Ks + 9 * i, Rs + 9 * i, Ts + 3 * i, M, cov, var,
				pixelVar);
		mat22Inv(var, ivar);
		double err = mahaDist2(rm, ms + 2 * i, ivar);

		if (err > 1.0) {
			return false;
		}
	}

	return true;
}

bool isLittleMove(int numCams, const MapPoint* p, double pixelVar,
		const double M[3], const double cov[9]) {
	for (int i = 0; i < numCams; i++) {
		FeaturePoint* fp = p->pFeatures[i];
		if (fp) { //check the reprojection error
			double rm[2], var[4], ivar[4];
			project(fp->K, fp->cam->R, fp->cam->t, M, rm);
			getProjectionCovMat(fp->K, fp->cam->R, fp->cam->t, M, cov, var,
					pixelVar);
			mat22Inv(var, ivar);

			if (mahaDist2(rm, fp->m, ivar) >= 1) {
				return false;
			}
		}
	}
	return true;
}

/**
 * update position for static points
 */
void updateStaticPointPosition(int numCams, MapPoint* p, double pixelVar,
		bool updateCov) {
	double Ks[SLAM_MAX_NUM * 36], Rs[SLAM_MAX_NUM * 36], ts[SLAM_MAX_NUM * 12],
			ms[SLAM_MAX_NUM * 8], nms[SLAM_MAX_NUM * 8];
	int numView = 0;
	for (int i = 0; i < numCams; i++) {
		const FeaturePoint* fp = p->pFeatures[i];
		if (fp && fp->K) {
			double iK[9];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, fp->K, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);

			getInvK(fp->K, iK);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);
			const FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp && fp->cam) {
				if (fp->K) {
					double C[3];
					getCameraCenter(fp->cam->R, fp->cam->t, C);
					double angle = getAbsRadiansBetween(p->M, C0, C);
					if (angle > maxAngle) {
						maxAngle = angle;
						maxFp = fp;
					}
				}
				fp = fp->preFrame;
			}
			if (maxFp) {
				doubleArrCopy(Ks, numView, maxFp->K, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);

				getInvK(maxFp->K, iK);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}
	if (numView >= 2) {
		triangulateMultiView(numView, Rs, ts, nms, p->M);
		if (updateCov)
			getTriangulateCovMat(numView, Ks, Rs, ts, p->M, p->cov,
					pixelVar);
	}
}
void updateStaticPointPositionAtKeyFrms(int numCams, MapPoint* p,
		double pixelVar, bool updateCov) {
	double Ks[SLAM_MAX_NUM * 36], Rs[SLAM_MAX_NUM * 36], ts[SLAM_MAX_NUM * 12],
			ms[SLAM_MAX_NUM * 8], nms[SLAM_MAX_NUM * 8];
	int numView = 0;
	for (int i = 0; i < numCams; i++) {
		const FeaturePoint* fp = p->pFeatures[i];
		if (fp && fp->bKeyFrm) {
			double iK[9];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, fp->K, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);

			getInvK(fp->K, iK);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);

			const FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp) {
				if (fp->bKeyFrm) {
					double C[3];
					getCameraCenter(fp->cam->R, fp->cam->t, C);
					double angle = getAbsRadiansBetween(p->M, C0, C);
					if (angle > maxAngle) {
						maxAngle = angle;
						maxFp = fp;
					}
				}
				fp = fp->preFrame;
			}
			if (maxFp) {
				doubleArrCopy(Ks, numView, maxFp->K, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);

				getInvK(maxFp->K, iK);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}
	if (numView < 2)
		return;
	triangulateMultiView(numView, Rs, ts, nms, p->M);
	if (updateCov)
		getTriangulateCovMat(numView, Ks, Rs, ts, p->M, p->cov, pixelVar);
}
/**
 * update position for dynamic points
 */
void updateDynamicPointPosition(int numCams, MapPoint* p, double pixelVar,
		bool updateCov) {
	double Ks[SLAM_MAX_NUM * 9], Rs[SLAM_MAX_NUM * 9], Ts[SLAM_MAX_NUM * 3],
			ms[SLAM_MAX_NUM * 2], nms[SLAM_MAX_NUM * 2];
	int numView = 0;
	int nDynamic = 0;
	for (int i = 0; i < numCams; i++) {
		FeaturePoint* fp = p->pFeatures[i];
		if (fp) {
			double iK[9];
			doubleArrCopy(Ks, numView, fp->K, 9);
			doubleArrCopy(Rs, numView, fp->cam->R, 9);
			doubleArrCopy(Ts, numView, fp->cam->t, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			getInvK(fp->K, iK);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;
			if (fp->type == TYPE_FEATPOINT_DYNAMIC)
				nDynamic++;
		}
	}

	if (nDynamic < 1 || numView < 2)
		return;

	//retriangulate the 3D coordinates
	triangulateMultiView(numView, Rs, Ts, nms, p->M);
	if (updateCov)
		getTriangulateCovMat(numView, Ks, Rs, Ts, p->M, p->cov, pixelVar);
}
