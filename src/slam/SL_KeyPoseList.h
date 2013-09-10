/*
 * SL_KeyFrmLst.h
 *
 *  Created on: 2010-11-25
 *      Author: Danping Zou
 */

#ifndef SL_KEYFRMLST_H_
#define SL_KEYFRMLST_H_
#include "SL_Define.h"

#include "imgproc/SL_Image.h"
#include "imgproc/SL_ImageOp.h"

#include "slam/SL_Camera.h"
#include "slam/SL_CameraGroup.h"
#include "slam/SL_FeaturePoint.h"
#include "slam/SL_MapPointList.h"

class KeyPose {
public:
	bool bSelfMotion;	//produced by self motion
	int frame; 			//current frame
	CamPoseItem* cam; 	//camera pose
	double K[9]; 		//camera intrinsic parameters

	//pointers to the feature points at the current frame
	FeaturePoint* pHead;
	FeaturePoint* pTail;

	int nMappedPts;

	KeyPose* pre;
	KeyPose* next;

	ImgG img; //current image
	ImgG imgSmall; //small image
	double imgScale;
public:
	KeyPose() :
			bSelfMotion(false),frame(0), cam(0), pHead(0), pTail(0), nMappedPts(0), pre(0), next(0) {
	}
	KeyPose(int f, CamPoseItem* cam_) :
			bSelfMotion(false),frame(f), cam(cam_), pHead(0), pTail(0), nMappedPts(0), pre(0), next(
					0) {
	}
	void setNumMappedPoints(int num) {
		nMappedPts = num;
	}
	void setAllImages(const ImgG& imgGray, const ImgG& imgGraySmall,
			double scale) {
		cloneImg(imgGray, img);
		cloneImg(imgGraySmall, imgSmall);
		imgScale = scale;
	}
	void setImage(const ImgG& imgGray) {
		cloneImg(imgGray, img);
	}
	void setSmallImage(const ImgG& imgGraySmall, double scale) {
		cloneImg(imgGraySmall, imgSmall);
		imgScale = scale;
	}
	void setCameraIntrinsic(const double* intrin) {
		memcpy(K, intrin, sizeof(double) * 9);
	}
	void setFeatPoints(FeaturePoint* head, FeaturePoint* tail) {
		pHead = head;
		pTail = tail;
	}

public:
	//debug
	void print() {
		logInfo("----------++++++++++\n");
		logInfo("frame:%d\n", frame);
		logInfo("nMappedPts:%d\n", nMappedPts);
		logInfo("++++++++++----------\n");
	}
};
class KeyPoseList {
public:
	int num;
	KeyPose head;
	KeyPose* tail;
public:
	KeyPoseList();
	~KeyPoseList();
public:
	void clear();
	KeyPose* add(int f, CamPoseItem* cam_);
	const int size() const {
		return num;
	}
	KeyPose* current() const {
		return tail;
	}
	KeyPose* first() const {
		return head.next;
	}
};
//A key frame is defined whenever a key camera pose is inserted at this frame. 
//It is aimed to fast access those key camera poses.
class KeyFrame {
public:
	int f;
	KeyPose* pPose[SLAM_MAX_NUM];
	CameraGroup camGroups[SLAM_MAX_NUM];
	int nGroup;
	int nCam;
	int nMapPts;
	//to store dynamic points
	MapPointList dynMapPoints;
public:
	KeyFrame* prev;
	KeyFrame* next;
public:
	KeyFrame() :
			f(-1), nGroup(0), nCam(0), nMapPts(0), prev(0), next(0) {
		memset(pPose, 0, sizeof(KeyPose*) * SLAM_MAX_NUM);
	}
	KeyFrame(int frame) :
			f(frame), nGroup(0), nCam(0), nMapPts(0), prev(0), next(0) {
		memset(pPose, 0, sizeof(KeyPose*) * SLAM_MAX_NUM);
	}
	void setKeyPose(int iCam, KeyPose* pose) {
		pPose[iCam] = pose;
	}
	void setCamNum(int camNum) {
		nCam = camNum;
	}
	void setMapPtsNum(int ptsNum) {
		nMapPts = ptsNum;
	}
	void setCamGroups(CameraGroup groups[], int groupNum) {
		for (int i = 0; i < groupNum; i++) {
			camGroups[i].copy(groups[i]);
		}
		nGroup = groupNum;
	}
	void storeDynamicMapPoints(const MapPointList& curMapPts) {
		for (const MapPoint* mpt = curMapPts.getHead();
				mpt != curMapPts.getTail(); mpt = mpt->next) {
			if (mpt->isLocalDynamic()) {
				MapPoint* newMpt = new MapPoint(*mpt);
				dynMapPoints.add(newMpt);
			}
		}
	}
};
class KeyFrameList {
public:
	int num;
	KeyFrame head;
	KeyFrame* tail;
public:
	KeyFrameList();
	~KeyFrameList();
public:
	void clear();
	KeyFrame* add(int frame);
	const int size() const {
		return num;
	}
	KeyFrame* current() const {
		return tail;
	}
	KeyFrame* first() const {
		return head.next;
	}
};
#endif /* SL_KEYFRMLST_H_ */
