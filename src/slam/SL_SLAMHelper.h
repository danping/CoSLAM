/*
 * SL_Pnp.h
 *
 *  Created on: 2011-1-17
 *      Author: Danping Zou
 */

#ifndef SL_SLAMHELPER_H
#define SL_SLAMHELPER_H
#include "math/SL_Matrix.h"
#include "slam/SL_Camera.h"
#include "tracking/SL_Track2D.h"

/* solve the PnP problem*/
int solvePnPRansac(int numPts, const double* pts3d, const double* pts2d,
		const double* K, double* R, double* t, int iterMax = 20,
		double errThres = 1.5, double* errReturn = 0,
		unsigned char* inlierFlag = 0);

void solvePnP(int numPts, const double* pts3d, const double* pts2d,
		const double* K, double* R, double* t);
void solvePnP(const Mat_d& pts3d, const Mat_d& pts2d, const double* K, Mat_d& R,
		Mat_d& t);
void solvePnPRobust(const Mat_d& pts3d, const Mat_d& pts2d, const double* K,
		Mat_d& R, Mat_d& t);

void solvePnPRobust(int numPts, const double* pts3d, const double* pts2d,
		const double* K, double* R, double* t);

void solvePnPRobust(const Mat_d& pts3d, const Mat_d& covs, const Mat_d& pts2d,
		const double* K, Mat_d& R, Mat_d& t);
void getCamCoords(const CamPoseItem* cam, double* p0, double* xp, double* yp,
		double* zp);
void getCamCenter(const CamPoseItem* cam, double org[]);
double getCamDist(const CamPoseItem* cam1, const CamPoseItem* cam2);
double getViewAngleChange(const double center[3], const CamPoseItem* cam1,
		const CamPoseItem* cam2);
#endif
