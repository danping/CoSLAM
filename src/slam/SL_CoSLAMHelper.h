/*
 * SL_CoSLAMHelper.h
 *
 *  Created on: 2011-2-20
 *      Author: Danping Zou
 */

#ifndef SL_COSLAMHELPER_H_
#define SL_COSLAMHELPER_H_
#include "SL_KeyPoseList.h"
#include "SL_SLAMHelper.h"

#include "tools/SL_WriteRead.h"
#include "imgproc/SL_ImageOp.h"

void sortKeyFrms(std::vector<KeyPose*>& keyFrms);
double diffAvgGrayLevel(const ImgG& img1, const ImgG& img2, double x1,
		double y1, double x2, double y2, int hw = 5);

/**
 * check whether the large 
 */
int isStaticRemovable(int numCams, MapPoint* p, double pixelVar, double M[3],
		double cov[9], int numFrame);
/**
 * when the reprojection errors at all views and all frames are all small ones, this point is considered to be a static point
 * return :
 * M : refined position of this point 
 * cov : covariance matrix of the estimated position
 */
bool isStaticPoint(int numCams, const MapPoint*p, double pixelVar, double M[3],
		double cov[9], int numFrame);

/**
 * check whether a map point is a static point when the feature point in camera #c is excluded
 */
bool isStaticPointExclude(int numCams, const MapPoint* p, double pixelVar,
		double M[3], double cov[9], int viewId, int numFrame);
/**
 * when the reprojection errors at the current frame are all small ones, this point is considered to be a dynamic ponit
 * @return
 * M : new position at the current frame
 * cov : covariance matrix at the current frame
 */
bool isDynamicPoint(int numCams, const MapPoint* p, double pixelVar,
		double M[3], double cov[9]);
/*
 * check whether the map point moves by projecting the new 3D position to old frames  
 */
bool isLittleMove(int numCams, const MapPoint* p, double pixelVar,
		const double M[3], const double cov[9]);

/*
 * get the convex hull of a set of 2D points
 */
void get2DConvexHull(const std::vector<double>& pts, std::vector<double>& cxh);

/**
 * update position for static points
 */
void updateStaticPointPosition(int numCams, MapPoint* p, double pixelVar,
		bool updateCov = true);
void updateStaticPointPositionAtKeyFrms(int numCams, MapPoint* p,
		double pixelVar, bool updateCov = true);

/**
 * update position for dynamic points
 */
void updateDynamicPointPosition(int numCams, MapPoint* p, double pixelVar,
		bool updateCov = true);

/**
 * get the area of polygon
 */
double getPolyArea(const std::vector<double>& poly);
#endif /* SL_COSLAMHELPER_H_ */
