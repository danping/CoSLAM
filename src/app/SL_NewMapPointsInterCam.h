/*
 * SL_NewMapPointsInterCam.h
 *
 *  Created on: 2011-1-23
 *      Author: Danping Zou
 */

#ifndef SL_NEWMAPPOINTSINTERCAM_H_
#define SL_NEWMAPPOINTSINTERCAM_H_

#include "SL_SingleSLAM.h"

#include "matching/SL_Matching.h"
#include "math/SL_LinAlg.h"
#include "slam/SL_Define.h"
#include "slam/SL_MapPointList.h"
#include "slam/SL_CameraGroup.h"

#include <vector>
#include <map>

class CoSLAM;
class NewMapPts {
public:
	ImgG m_img[SLAM_MAX_NUM];
	double m_K[SLAM_MAX_NUM][9];
	double m_invK[SLAM_MAX_NUM][9];
	double m_R[SLAM_MAX_NUM][9];
	double m_t[SLAM_MAX_NUM][3];

	int numCams;
	CameraGroup m_camGroup;
	CoSLAM* pCoSLAM;
	int m_curFrame;

	/* feature points in each view*/
	std::vector<FeaturePoint*> pFeatPts[SLAM_MAX_NUM];

	/* output*/
	std::vector<MapPoint*> newMapPts;
	

	map<int, int> camid_to_ind;
	

	void decidePointType();
	/** match SURF feature points between two views*/
	virtual int matchBetween(int iCam, int jCam, Matching& matches) = 0;
	virtual void setInputs(const CameraGroup& camGroup, CoSLAM* coSLAM,
			int curFrame) = 0;
	virtual int run() = 0;
	virtual void output() = 0;
public:
	NewMapPts() :
			numCams(0), pCoSLAM(0), m_curFrame(-1) {
	}
	virtual ~NewMapPts() {
	}
};
/**
 * use NCC matching to generate new map points
 */
class NewMapPtsNCCParam {
public:
	double maxEpiErr;
	double minNcc;
	double maxDisp;
	double blockScale;
public:
	NewMapPtsNCCParam() {
		maxEpiErr = 50;
		minNcc = 0.80;
		maxDisp = 80;
		blockScale = 0.3;
	}
};
class NewMapPtsNCC: public NewMapPts, NewMapPtsNCCParam {
protected:
	Mat_d m_seeds1[SLAM_MAX_NUM * SLAM_MAX_NUM];
	Mat_d m_seeds2[SLAM_MAX_NUM * SLAM_MAX_NUM];
	Mat_d matFeatPts[SLAM_MAX_NUM];

public:
	NewMapPtsNCC();
	virtual ~NewMapPtsNCC();
public:
	void reset() {
		newMapPts.clear();
		pCoSLAM = 0;
		m_curFrame = -1;
		for (int i = 0; i < numCams; i++) {
			m_img[i].clear();
			matFeatPts[i].clear();
			pFeatPts[i].clear();
		}
		for (int i = 0; i < numCams * numCams; i++) {
			m_seeds1[i].clear();
			m_seeds2[i].clear();
		}
		numCams = 0;
	}
	void addSlam(SingleSLAM& sl, int curFrame) {
		doubleArrCopy(m_K[numCams], 0, sl.K.data, 9);
		mat33Inv(m_K[numCams], m_invK[numCams]);

		doubleArrCopy(m_R[numCams], 0, sl.m_camPos.current()->R, 9);
		doubleArrCopy(m_t[numCams], 0, sl.m_camPos.current()->t, 3);

		cloneImg(sl.m_img, m_img[numCams]);

		int maxNumFeatPts = sl.m_featPts.frameNum[curFrame];
		matFeatPts[numCams].resize(maxNumFeatPts, 2);
		pFeatPts[numCams].reserve(maxNumFeatPts);
		pFeatPts[numCams].clear();

		//get tracked feature points at current frame
		std::vector<FeaturePoint*> vecFeatPts;
		int fNum = sl.getTrackedFeatPts(vecFeatPts, 0, 3);
		int nFpts = 0;
		for (int n = 0; n < fNum; n++) {
			//store feature points that are unmapped or mapped to a false point 
			if (vecFeatPts[n]->mpt == 0 || vecFeatPts[n]->mpt->isFalse()) {
				pFeatPts[numCams].push_back(vecFeatPts[n]);
				matFeatPts[numCams].data[2 * nFpts] = vecFeatPts[n]->x;
				matFeatPts[numCams].data[2 * nFpts + 1] = vecFeatPts[n]->y;
				nFpts++;
			}
		}
		matFeatPts[numCams].rows = nFpts;
		numCams++;
	}
	/* get seed points between iCam and jCam views*/
	int getSeedsBetween(MapPointList& mapPts, int iCam, int jCam, Mat_d& seed1,
			Mat_d& seed2);
	/* match feature points between iCam and jCam views*/
	virtual int matchBetween(int iCam, int jCam, Matching& matches);

	virtual int reconstructTracks(Track2D tks[], //feature tracks
			int ntks, int curFrame, std::vector<MapPoint*>& mapPts, //new map points that are generated
			int minLen, double maxRpErr);

	virtual void setInputs(const CameraGroup& camGroup, CoSLAM* coSLAM,
			int curFrame);
	virtual int run();
	virtual void output();
};
//#define USE_GPUSURF
#ifdef USE_GPUSURF
/**
 * using surf matching to generate new map points (including detecting new SURF feature points
 */
class NewMapPtsSURFParam {
public:
	double hessianThreshold;
public:
	NewMapPtsSURFParam() :
	hessianThreshold(0.5) {
	}
};

class NewMapPtsSURF: public NewMapPts, NewMapPtsSURFParam {
public:
	std::vector<cv::KeyPoint> surfPoints[SLAM_MAX_NUM];
	std::vector<float> surfDescs[SLAM_MAX_NUM];
	int m_descDim;
public:
	NewMapPtsSURF() :
	NewMapPtsSURFParam() {
	}
	;
	virtual ~NewMapPtsSURF();
	void addSlam(SingleSLAM& sl, int curFrame) {
		doubleArrCopy(m_K[numCams], 0, sl.K.data, 9);
		mat33Inv(m_K[numCams], m_invK[numCams]);

		doubleArrCopy(m_R[numCams], 0, sl.m_camPos.current()->R, 9);
		doubleArrCopy(m_t[numCams], 0, sl.m_camPos.current()->t, 3);

		cloneImg(sl.m_img, m_img[numCams]);
		numCams++;
	}

	/**detect SURF feature points in each view*/
	int detectSURFPoints();
	/** match SURF feature points between two views*/
	virtual int matchBetween(int iCam, int jCam, Matching& matches);

	virtual void setInputs(const CameraGroup& camGroup, CoSLAM* coSLAM, int curFrame);
	virtual int reconstructTracks(Track2D tks[], //feature tracks
			int ntks,
			int curFrame,
			std::vector<MapPoint*>& mapPts,//new map points that are generated
			int minLen,
			double maxRpErr);
	virtual int run();
	virtual void output();
};
#endif
void getValidRowsCols(const Mat_d& distMat, double invalidVal, Mat_c& rowFlag,
		Mat_c& colFlag);
/* convert matches to indices*/
void match2ind(int maxNum, const Matching& matches, Mat_i& indices);
/* generate 2D feature tracks from sequential matching results*/
int featTracksFromMatches(int numCams, std::vector<FeaturePoint*> pFeatPts[],
		Matching matches[], Track2D tks[]);

#endif 
