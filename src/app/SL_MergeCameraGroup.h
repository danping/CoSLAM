/*
 * SL_MergeCameraGroup.h
 *
 *  Created on: 2011-6-8
 *      Author: tsou
 */

#ifndef SL_MERGECAMERAGROUP_H_
#define SL_MERGECAMERAGROUP_H_
#include "slam/SL_FeaturePoint.h"
#include "slam/SL_GlobalPoseEstimation.h"
#include "slam/SL_KeyPoseList.h"
#include "math/SL_LinAlg.h"

#include "tools/cvHelper.h"

#include "geometry/SL_Point.h"
#include "geometry/SL_RigidTransform.h"
#include "geometry/SL_BundleAdjust.h"

#include <vector>
class MergeInfo {
public:
	int frame1, camId1, groupId1;
	int frame2, camId2, groupId2;
	bool valid; //if this merge information is valid, all valid MergeInfo objects are treated as hard constraints 
	double E[9], R[9], t[3];
	vector<pair<FeaturePoint*, FeaturePoint*> > matchedFeatPts;
public:
	MergeInfo() :
			frame1(-1), camId1(-1), groupId1(-1), frame2(-1), camId2(-1), groupId2(
					-1), valid(false) {
		memset(E, 0, sizeof(double) * 9);
		memset(R, 0, sizeof(double) * 9);
		memset(t, 0, sizeof(double) * 3);
	}
	MergeInfo(const MergeInfo& other) :
			frame1(other.frame1), camId1(other.camId1), groupId1(
					other.groupId1), frame2(other.frame2), camId2(other.camId2), groupId2(
					other.groupId2), valid(other.valid), matchedFeatPts(
					other.matchedFeatPts) {
		memcpy(E, other.E, sizeof(double) * 9);
		memcpy(R, other.R, sizeof(double) * 9);
		memcpy(t, other.t, sizeof(double) * 3);
	}
	void set(int f1, int cId1, int gId1, int f2, int cId2, int gId2) {
		frame1 = f1;
		camId1 = cId1;
		groupId1 = gId1;
		frame2 = f2;
		camId2 = cId2;
		groupId2 = gId2;
	}
	void set(const double tE[9], const double tR[9], const double tt[3]) {
		memcpy(E, tE, sizeof(double) * 9);
		memcpy(R, tR, sizeof(double) * 9);
		memcpy(t, tt, sizeof(double) * 3);
	}
	void set(int f1, int cId1, int gId1, int f2, int cId2, int gId2,
			const double tE[9], const double tR[9], const double tt[3]) {
		set(f1, cId1, gId1, f2, cId2, gId2);
		set(tE, tR, tt);
	}
	void reverse() {
		int tId = groupId2;
		groupId2 = groupId1;
		groupId1 = tId;

		tId = camId2;
		camId2 = camId1;
		camId1 = tId;

		double tE[9];
		mat33Trans(E, tE);
		doubleArrCopy(E, 0, tE, 9);

		double iR[9], it[3];
		invRigidTransFromTo(R, t, iR, it);
		doubleArrCopy(R, 0, iR, 9);
		doubleArrCopy(t, 0, it, 3);
	}
};

class CoSLAM;
class MergeCameraGroup {
public:
	MergeCameraGroup();
	virtual ~MergeCameraGroup();
public:
	KeyFrame* m_pCurKeyFrm;
	KeyFrame* m_pFirstConstrainFrm;
	KeyFrame* m_pFixedKeyFrm;

	//two camera groups need to be merged
	int m_gid1, m_gid2;
	int m_camid1, m_camid2;

	int imgW[SLAM_MAX_NUM];
	int imgH[SLAM_MAX_NUM];

	MergeInfo m_mergeInfo[SLAM_MAX_NUM * SLAM_MAX_NUM];
	int m_nMergeInfo;
	bool m_b3D3D;

	map<KeyPose*, int> pose_to_order;
	map<pair<int, int> , int> ind_to_order;
	map<pair<int, int> , KeyPose*> ind_to_pose;
	map<int, KeyPose*> order_to_pose;

	vector<int> m_camIds; //cameras for merge
	vector<int> m_keyFrmIds;

	map<CamPoseItem*, int> m_keyPoseNodeId;
	vector<CamPoseItem*> m_pKeyPoses;
	//for correcting the camera poses for key frames 
	GlobalPoseGraph keyGraph;
	//for correcting the camera poses for all video frames in each view
	GlobalPoseGraph camGraphs[SLAM_MAX_NUM];
	map<CamPoseItem*, int> m_camNodeId[SLAM_MAX_NUM];
	vector<CamPoseItem*> m_cams[SLAM_MAX_NUM];
	vector<FeaturePoint*> m_featPoints[SLAM_MAX_NUM];
public:
	int getLastFrame() {
		return m_pCurKeyFrm->f;
	}
	int getFirstFrame() {
		return m_pFixedKeyFrm->f;
	}
	void setCurrentFrame(KeyFrame* kf);
	void setImageSize(int iCam, int w, int h);
	/* check if there are groups that could be possibly merged. return the number of possible merges*/
	int checkPossibleMergable(int minNum, double minAreaRatio,
			double maxCamDist);
	/* check whether the two group can be merged*/
	int checkPossibleMergableBetween(int groupId1, int groupId2,
			const CameraGroup& group1, const CameraGroup& group2, int minInNum,
			double minAreaRatio, double maxCamDist);
	/* check view overlap between two cameras*/
	bool checkViewOverlapFromTo(int iCam, int jCam, int minInNum,
			double minInAreaRatio);
	bool checkViewOverlap(int iCam, int jCam, int minInNum,
			double minInAreaRatio);

	bool checkCamDist(int iCam, int jCam, double maxCamDist);

	void storeFeaturePoints();

	void addMergeInfo(bool valid, int f1, int camId1, int gId1, int f2,
			int camId2, int gId2, double E[9], double R[9], double t[9]) {
		m_mergeInfo[m_nMergeInfo].set(f1, camId1, gId1, f2, camId2, gId2, E, R,
				t);
		m_mergeInfo[m_nMergeInfo++].valid = valid;
	}
public:
	/*match feature points between iCam-th camera and jCam-th camera*/
	int detectSURFPoints(int iCam, Mat_d& surfPts, vector<float>& surfDesc,
			int& dimDesc);
	int matchSURFPoints(int iCam, int jCam, const Mat_d& pts1,
			const Mat_d& pts2, int dimDesc, vector<float>&desc1,
			vector<float>& desc2, Matching& matches);

	int computeEMat(int iCam, int jCam, const Mat_d& surfPts1,
			const Mat_d& surfPts2, const Matching& matches,
			Matching& newMatches, double E[9], int nRansac, double maxEpiErr,
			int minInlierNum);

	void getCameraPose(const double E[9], double R[9], double t[3]);

	int matchFeaturePointsNCC(int iCam, int jCam,
			vector<FeaturePoint*>& featPts1, vector<FeaturePoint*>& featPts2,
			const Mat_d& surfPts1, const Mat_d& surfpts2, Matching& surfMatches,
			const double E[9], Matching& featMatches, double maxEpiErr,
			double minNcc, double maxDisp);

	/**
	 * match the feature points on the possibly mergable cameras and get their relative poses
	 */
	int matchMergableCameras();

	bool checkMergeMapPoints(vector<vector<FeaturePoint*> >& featureTracks,
			vector<MapPoint*>& mapPoints, double thres = 500);

	void mergeMapPoints(vector<vector<FeaturePoint*> >& featureTracks,
			vector<MapPoint*>& mapPoints);

	void getCamIdsInBothGroups(const CameraGroup* group1,
			const CameraGroup* group2, vector<int>& camIds);

	int buildIdMapForSelfKeyPoses(const vector<int>& camIds,
			int nSelfMotion = 1);

	bool genMergeInfoVer2(int gId1, int gId2, int maxiCam, int maxjCam,
			const Matching& featMatches);

	int getCorresFeatPtsInPrevSelfKeyPoses(
			vector<vector<FeaturePoint*> >& featpts);
	/**
	 * get corresponding feature points
	 */
	int getCorresFeatPtsAndMapPts(int gId1, int gId2,
			vector<FeaturePoint*>& featPts1, vector<FeaturePoint*>& featPts2,
			const Matching& featMatches,
			vector<vector<FeaturePoint*> >& corres_feat_pts,
			vector<MapPoint*>& mapPts);

	int getUnMergedFeatureTracks(int gId1, int gId2,
			vector<vector<FeaturePoint*> >& featTracks,
			vector<MapPoint*>& mapPts, int maxNum);
	/**
	 * search the first frame that the matched cameras are previously in the same group
	 */
	void searchFirstKeyFrameForMerge(int nMaxKeyFrame = 100);
	/**
	 * construct graph from the first key frame
	 */
	void _constructGraphForKeyFrms(KeyFrame* frame, GlobalPoseGraph& graph);
	void constructGraphForKeyFrms() {
		_constructGraphForKeyFrms(m_pFixedKeyFrm, keyGraph);
	}
	/**
	 * construct graph form all video frames
	 */
	void _constructGraphForAllFrms(KeyFrame* frame, GlobalPoseGraph graphs[]);
	void constructGraphForAllFrms() {
		_constructGraphForAllFrms(m_pFixedKeyFrm, camGraphs);
	}
	/**
	 * recompute the new key poses according to the relative poses
	 */
	void recomputeKeyCamPoses();
	/**
	 * recompute the 3D coordinates of map points
	 */
	void recomputeMapPoints(vector<MapPoint*>& mapPoints, double pixel_var);
	/**
	 * update the new camera poses 
	 */
	void recomputeAllCameraPoses();

	/**
	 * 
	 */
	int mergeMatchedGroups(CameraGroup groups[SLAM_MAX_NUM], int& nGroup);
	void mergeMatchedFeaturePoints();
public:
	//for debug
	void printMergeInfo();
};

void getRigidTransFromToWithEMat(const double* R0, const double* T0,
		const double* R1, const double* T1, double dR[3], double dT[3],
		double E[9]);
double getMaxRepErr(const vector<Mat_d> & Ks, const vector<Mat_d>& Rs,
		const vector<Mat_d>& ts, const vector<vector<Meas2D> >& meas2d,
		const vector<Point3d>& pts3d);
#endif /* SL_MERGECAMERAGROUP_H_ */
