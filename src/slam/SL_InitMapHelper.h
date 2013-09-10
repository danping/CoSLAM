/*
 * SL_InitMapHelper.h
 *
 *  Created on: 2011-1-9
 *      Author: Danping Zou
 */

#ifndef SL_INITMAPSELECTIMAGEORDER_H_
#define SL_INITMAPSELECTIMAGEORDER_H_
#include "math/SL_Matrix.h"
#include "tracking/SL_Track2D.h"
#include "matching/SL_Matching.h"

#include "SL_SLAMHelper.h"
#include <list>
#include <queue>
/* generate a camera sequence that has the maximum match number between adjacent views*/
void selectCameraOrder(int camNum, const int* distMat, Mat_i& order);

/* select tracks with length larger than lMin*/
int selectTracks(int numTracks, const Track2D featureTracks[], int lMin, std::vector<const Track2D*>& tracks);

/* select tracks whose duration is no less than f1~f2*/
int selectTracksDuring(std::vector<const Track2D*>& vecTracks, int f1, int f2, std::vector<const Track2D*>& tracks);
/* get correspondence between f and f+1 from tracks*/
int getMatchingFromTracks(std::vector<const Track2D*>& vecPTracks, int f1, int f2, Mat_d& pts1, Mat_d& pts2);
int getMatchingFromTracks(int nTracks, const Track2D* vecPTracks, int f1, int f2, Mat_d& pts1, Mat_d& pts2);
/* get points at frame f*/
void getPointsFromTracks(std::vector<const Track2D*>& vecPTracks, int f, Mat_d& pts);
void getPointsFromTracks(std::vector<const Track2D*>& vecPTracks, int f, const Mat_uc& inlierFlag, Mat_d& pts);
int evaluatePose(const double* K, const double* R, const double* t, int nPts, const double* pts3d, const double pts2d, double errThes, double* errReturn, unsigned char* inlerFlag = 0);

/* for bundle adjustment*/
void TracksToMeasurments(const double invK[][9], std::vector<const Track2D*> vecPTracks, int nviews, Mat_d& measurements, Mat_c& vmask);

void getMatchIndices(Matching& matches, int maxInd, Mat_i& ind, bool inverse = false);
#endif /* SL_INITMAPSELECTIMAGEORDER_H_ */
