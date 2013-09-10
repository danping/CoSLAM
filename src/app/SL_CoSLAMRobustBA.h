/*
 * SL_RobustBA.h
 *
 *  Created on: 2011-7-24
 *      Author: zou
 */

#ifndef SL_ROBUSTBA_H_
#define SL_ROBUSTBA_H_

/*
 * SL_Bundle.h
 *
 *  Created on: 2011-1-20
 *      Author: Danping Zou
 */

#include "math/SL_Matrix.h"
#include "slam/SL_Camera.h"
#include "slam/SL_MapPoint.h"
#include "slam/SL_FeaturePoint.h"
#include "slam/SL_KeyPoseList.h"
#include "slam/SL_GlobalPoseEstimation.h"
#include "geometry/SL_BundleAdjust.h"
#include <vector>
/*
 * use bundle adjustment to refine the camera poses and 3D points simultaneously
 */
class CoSLAM;
class RobustBundleRTSParameter {
public:
	//parameters
	int m_nPtsCon, m_nCamsCon, m_nMaxIter, m_nInnerMaxIter;
	double m_maxErr;
public:
	RobustBundleRTSParameter() :
			m_nPtsCon(0), m_nCamsCon(0), m_nMaxIter(5), m_nInnerMaxIter(10), m_maxErr(6.0) {

	}
	void setParameters(int nPtsCon, int nCamsCon, int nMaxIter, int nInnerMaxIter) {
		m_nPtsCon = nPtsCon;
		m_nCamsCon = nCamsCon;
		m_nMaxIter = nMaxIter;
		m_nInnerMaxIter = nInnerMaxIter;
	}
};
class RobustBundleRTS: public RobustBundleRTSParameter {
public:
	bool processed;
	CoSLAM* pCoSLAM;
	KeyFrame* firstKeyFrame;
	KeyFrame* lastKeyFrame;

	std::vector<CamPoseItem*> keyCamPoses;
	std::vector<MapPoint*> mapPoints;

	std::map<MapPoint*, int> mapPt2ind;
	std::map<int, MapPoint*> int2MapPt;
	std::map<std::pair<int, int>, int> cameraInd;
	std::map<MapPoint*, std::vector<const FeaturePoint*> > mapAndVecFeatPts;

	/* for correcting the camera poses of non-key frames*/
	GlobalPoseGraph camGraphs[SLAM_MAX_NUM];
	std::map<CamPoseItem*, int> nonKeyCamNodeId[SLAM_MAX_NUM];
	std::vector<CamPoseItem*> nonKeyCamPoses[SLAM_MAX_NUM];

	/* temporary variables*/
	std::vector<Mat_d> Ks, Rs, Ts;
	std::vector<Point3d> pt3Ds;
	std::vector<std::vector<Meas2D> > meas2Ds;
public:
	RobustBundleRTS();
	virtual ~RobustBundleRTS();
protected:
	void _addCorrespondingPoint(std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts,
	MapPoint* pt3d, const FeaturePoint* featPt);
	bool checkReprojError(std::map<MapPoint*, std::vector<const FeaturePoint*> >::iterator& iter, double errMax);
	void parseInputs(std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts);
public:
	//////////////////////////////////////////////////////////////////////////////////////////////
	/* the following is for bundle adjustment*/
	void setCoSLAM(CoSLAM* coSLAM);
	void setFirstKeyFrame(KeyFrame* keyFrame);
	void setLastKeyFrame(KeyFrame* keyFrame);

	void addKeyFrames();
	void addPoints();

	void addKeyCamera(const double* KMat, CamPoseItem* camPos);

	void addCorrespondingPoint(MapPoint* pt3d, const FeaturePoint* featPt);

	void apply(int nPtsCon, int nCamsCon, int maxIter = 5, int nInnerMaxIter = 10);
	void run(int nPtsCon, int nCamsCon, int maxIter = 5, int nInnerMaxIter = 10);

	//////////////////////////////////////////////////////////////////////////////////////////////
	/* the following is for updating the camera poses of non-key frames*/
	void constructCameraGraphs();
	void updateNonKeyCameraPoses();
public:
	void apply();
	void run();
	void output();
	void updateNewPosesPoints();
};
#endif /* SL_ROBUSTBA_H_ */
