/*
 * GPUKLT.h
 *
 *  Created on: Mar 24, 2011
 *      Author: Danping Zou
 */

#ifndef GPUKLT_H_
#define GPUKLT_H_

#include "CGKLT/v3d_gpupyramid.h"
#include "CGKLT/v3d_gpuklt.h"

#include "slam/SL_FeaturePoints.h"
#include "math/SL_Matrix.h"

#include "tracking/SL_Track2D.h"

class GPUKLT {
public:
	int m_camId;
	int m_frame;
	int m_W, m_H;
	//intrinsic parameters
	Mat_d m_K, m_invK, m_kud;
	Track2D* m_tks;
	int m_nMaxCorners;
protected:
	V3D_GPU::KLT_SequenceTracker* _tracker;
	V3D_GPU::KLT_TrackedFeature* _features;
protected:
	void addToFeaturePoints(int nDetectedFeatures,
			V3D_GPU::KLT_TrackedFeature* features, FeaturePoints& ips);
	void addToFeaturePoints(int nDetectedFeatures,
			V3D_GPU::KLT_TrackedFeature* features,
			std::vector<FeaturePoint*>& pExistFeat, FeaturePoints& ips);
public:
	GPUKLT();
	virtual ~GPUKLT();
	int currentFrame() {
		return m_frame;
	}
	void setIntrinsicParam(const double* K, const double* invK,
			const double* k_ud) {
		m_K.cloneFrom(K, 3, 3);
		m_invK.cloneFrom(invK, 3, 3);
		m_kud.cloneFrom(k_ud, 7, 1);
	}
	void clear();
public:
	//routines for KLT tracking
	void init(int camId, int W, int H,
			V3D_GPU::KLT_SequenceTrackerConfig* pCfg);
	//detect corners at the first frame to start KLT tracking
	//	f : current frame
	//	imageData : 8-bit gray image
	//	ips[output] : detected feature points 
	int first(int f, const unsigned char* imgData, FeaturePoints& ips);
	// pExistFeat : feature points fed to track
	int first(int f, const unsigned char* imgData,
			std::vector<FeaturePoint*>& pExistFeat, FeaturePoints& ips);

	int next(const unsigned char* imgData, FeaturePoints& ips);
//	// pExistFeat : feature points fed to track
//	int next(const unsigned char* imaData, std::vector<FeaturePoint*>& pExistFeat, FeaturePoints& ips);
	int feedExternFeatPoints(std::vector<FeaturePoint*>& externFeatPts);
public:
	//routines for only corner detection
	void detectCorners(int W, int H, const unsigned char* imgData,
			Mat_f& corners, float minCornerness = 1500.0f, int minDistance = 5);
public:
	//for debug
	void getCurrentCorners(Mat_d& corners);

};
#endif /* GPUKLT_H_ */
