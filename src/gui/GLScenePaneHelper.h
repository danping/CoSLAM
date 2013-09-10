/*
 * GLModelPaneHelper.h
 *
 *  Created on: 2011-6-12
 *      Author: tsou
 */

#ifndef GLMODELPANEHELPER_H_
#define GLMODELPANEHELPER_H_
#include "GLHelper.h"
#include "app/SL_CoSLAM.h"
void drawAllCam(const CamPoseList& camlst, double camSize = 1.0);

void drawCamera(int IW,
		int IH,
		const double* K,
		const CamPoseItem* cam,
		double camSize,
		const float* color,
		const double maxDist);

void drawCameraPose(const CamPoseItem* cam, double camSize, const float* color, float linewidth = 1.0f);
void drawCamCenter(const CamPoseItem* cam, const float* color, int ptSz = 1);
void drawCurMapPoint(const MapPoint* p, double range, double pointSize, bool drawCov);

void getCenterAndRange(const CamPoseList& camlst, double& xc, double& yc, double& zc, double& scale);
void getCenterAndRange(const MapPointList& pts, double& xc, double& yc, double& zc, double &range);
void
getCenterAndRange(const CamPoseList& camlst, const MapPointList& pts, double &xc, double& yc, double& zc, double &range);

#endif /* GLMODELPANEHELPER_H_ */
