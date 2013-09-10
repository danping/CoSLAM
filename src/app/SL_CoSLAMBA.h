/*
 * SL_Bundle.h
 *
 *  Created on: 2011-1-20
 *      Author: Danping Zou
 */

#ifndef SL_BUNDLE_H_
#define SL_BUNDLE_H_
#include "math/SL_Matrix.h"
#include "slam/SL_Camera.h"
#include "slam/SL_MapPoint.h"
#include "slam/SL_FeaturePoint.h"
#include "slam/SL_KeyPoseList.h"
#include "slam/SL_GlobalPoseEstimation.h"
#include "geometry/SL_BundleHelper.h"
#include <vector>
/*
 * use bundle adjustment to refine the camera poses and 3D points simultaneously
 */
class CoSLAM;
class BundleRTS {
public:
	bool processed;
	CoSLAM* pCoSLAM;
	KeyFrame* firstKeyFrame;
	KeyFrame* lastKeyFrame;

	int maxNumPts;
	int maxNumCams;

	int numPts;
	int numCams;
	int numMeas;
	int numPtsCon;
	int numCamsCon;

	//temporary variables
	double* m_paramVec;
	sbaGlobs m_globs;

	Mat_d Ks; //intrinsic matrices
	Mat_d ms; //2D points

	Mat_d Rs; //camera pose
	Mat_d Ts;
	Mat_d Ms; //3D points 
	Mat_c vmask;

	std::vector<CamPoseItem*> keyCamPoses;
	std::vector<MapPoint*> mapPoints;
	std::vector<FeaturePoint*> featPoints;

	std::map<MapPoint*, int> mapPointInd;
	std::map<std::pair<int, int>, int> cameraInd;
	std::map<MapPoint*, std::vector<const FeaturePoint*> > _mapPts;

	/* for correcting the camera poses of non-key frames*/
	GlobalPoseGraph camGraphs[SLAM_MAX_NUM];
	std::map<CamPoseItem*, int> nonKeyCamNodeId[SLAM_MAX_NUM];
	std::vector<CamPoseItem*> nonKeyCamPoses[SLAM_MAX_NUM];
public:
	BundleRTS();
	virtual ~BundleRTS();
protected:
	void compressMeasurements();
	void addCorrespond(std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts,
	MapPoint* pt3d,
	const FeaturePoint* featPt);
	bool checkReprojError(std::map<MapPoint*, std::vector<const FeaturePoint*> >::iterator& iter, double errMax);
	/* remove the feature points with large reprojection errors*/
	void refineInput(std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts);
public:
	//////////////////////////////////////////////////////////////////////////////////////////////
	/* the following is for bundle adjustment*/
	void setCoSLAM(CoSLAM* coSLAM);
	void init(int nMaxPoints, int nMaxCameras);
	void setFirstKeyFrame(KeyFrame* keyFrame);
	void setLastKeyFrame(KeyFrame* keyFrame);
	void addKeyFrames();
	void addPoints();
	void addKeyCamera(const double* KMat, CamPoseItem* camPos);
	int add3DPoint(MapPoint* pt3d);
	void _add2DPoint(MapPoint* pt3d, const FeaturePoint* featPt);
	void add2DPoint(MapPoint* pt3d, const FeaturePoint* featPt);

	void apply(int nPtsCon, int nCamsCon, int maxIter = 10);
	void run(int nPtsCon, int nCamsCon, int maxIter = 10);

	//////////////////////////////////////////////////////////////////////////////////////////////
	/* the following is for updating the camera poses of non-key frames*/
	void constructCameraGraphs();
	void updateNonKeyCameraPoses();
public:
	//parameters
	int m_nPtsCon, m_nCamsCon, m_nMaxIter;
	void setParameters(int nPtsCon, int nCamsCon, int nMaxIter) {
		m_nPtsCon = nPtsCon;
		m_nCamsCon = nCamsCon;
		m_nMaxIter = nMaxIter;
	}
	void apply();
	void run();
	void output();

	void updateNewPosesPoints();
};
#endif /* SL_BUNDLE_H_ */
