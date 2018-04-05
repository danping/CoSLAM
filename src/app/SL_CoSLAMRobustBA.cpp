/*
 * SL_RobustBA.cpp
 *
 *  Created on: 2011-7-24
 *      Author: zou
 */

#include "SL_error.h"
#include "SL_CoSLAM.h"
#include "SL_CoSLAMRobustBA.h"
#include "SL_GlobParam.h"

#include "slam/SL_CoSLAMHelper.h"
#include "slam/SL_SLAMHelper.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Quaternion.h"
#include "geometry/SL_RigidTransform.h"
#include "geometry/SL_Triangulate.h"
#include <cassert>

RobustBundleRTS::RobustBundleRTS() :
		processed(false), pCoSLAM(0), firstKeyFrame(0), lastKeyFrame(0) {

}
RobustBundleRTS::~RobustBundleRTS() {
}
void RobustBundleRTS::setCoSLAM(CoSLAM* coSLAM) {
	pCoSLAM = coSLAM;
}
void RobustBundleRTS::setFirstKeyFrame(KeyFrame* keyFrame) {
	firstKeyFrame = keyFrame;
}
void RobustBundleRTS::setLastKeyFrame(KeyFrame* keyFrame) {
	lastKeyFrame = keyFrame;
}
void RobustBundleRTS::addKeyFrames() {
	assert(firstKeyFrame && lastKeyFrame);
	//add key poses for bundle adjustment
	for (KeyFrame* kf = firstKeyFrame; kf != lastKeyFrame->next;
			kf = kf->next) {
		for (int i = 0; i < kf->nCam; i++) {
			addKeyCamera(kf->pPose[i]->K, kf->pPose[i]->cam);
		}
	}
}
void RobustBundleRTS::addPoints() {
	//add feature points
	assert(firstKeyFrame && lastKeyFrame);
	for (KeyFrame* kf = firstKeyFrame; kf != lastKeyFrame->next;
			kf = kf->next) {
		for (int c = 0; c < kf->nCam; c++) {
			FeaturePoint* pHead = kf->pPose[c]->pHead;
			FeaturePoint* pTail = kf->pPose[c]->pTail;

			if (!pHead)
				continue;

			for (FeaturePoint* p = pHead; p != pTail->next; p = p->next) {
				//only static map points are used for bundle adjustment
				if (p->mpt && p->mpt->isLocalStatic()) {
					addCorrespondingPoint(p->mpt, p);
				}
			}
		}
	}
	//add dynamic points
	for (KeyFrame* kf = firstKeyFrame; kf != lastKeyFrame->next;
			kf = kf->next) {
		for (MapPoint* mpt = kf->dynMapPoints.getHead(); mpt; mpt = mpt->next) {
			for (int c = 0; c < kf->nCam; c++) {
				if (mpt->pFeatures[c] && mpt->pFeatures[c]->f == kf->f) {
					addCorrespondingPoint(mpt, mpt->pFeatures[c]);
				}
			}
		}
	}
}
void RobustBundleRTS::_addCorrespondingPoint(
		std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts,
		MapPoint* mpt,
		const FeaturePoint* featPt) {
	mapPts[mpt].push_back(featPt);
}
void RobustBundleRTS::addKeyCamera(const double* KMat, CamPoseItem* camPos) {
	assert(KMat && camPos);
	int frame = camPos->f;
	int camId = camPos->camId;
	cameraInd[std::make_pair(frame, camId)] = keyCamPoses.size();
	Ks.push_back(Mat_d(3, 3, KMat));
	Rs.push_back(Mat_d(3, 3, camPos->R));
	Ts.push_back(Mat_d(3, 1, camPos->t));
	keyCamPoses.push_back(camPos);
}
void RobustBundleRTS::addCorrespondingPoint(MapPoint* pt3d,
		const FeaturePoint* featPt) {
	_addCorrespondingPoint(mapAndVecFeatPts, pt3d, featPt);
}

void RobustBundleRTS::apply(int nPtsCon, int nCamsCon, int maxIter,
		int nInnerMaxIter) {
	run(nPtsCon, nCamsCon, maxIter, nInnerMaxIter);
	output();
}
void RobustBundleRTS::apply() {
	run();
	output();
}
void RobustBundleRTS::parseInputs(
		std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts) {
	mapPoints.clear();
	pt3Ds.clear();
	meas2Ds.clear();

	size_t validNum = 0;
	size_t featNum = 0;

	std::map<MapPoint*, std::vector<const FeaturePoint*> >::iterator iter;
	for (iter = mapPts.begin(); iter != mapPts.end(); iter++) {
		size_t nfpts = iter->second.size();
		if (nfpts > 1) {
			//add map point
			MapPoint* mpt = iter->first;
			mapPt2ind[mpt] = validNum;
			int2MapPt[validNum] = mpt;

			mapPoints.push_back(mpt);
			pt3Ds.push_back(Point3d(mpt->x, mpt->y, mpt->z));

			validNum++;
			featNum += nfpts;

			meas2Ds.push_back(std::vector<Meas2D>());

			//add corresponding feature points
			std::vector<const FeaturePoint*> vecFeatPts;
			vecFeatPts.resize(keyCamPoses.size(), 0);
			for (size_t i = 0; i < nfpts; i++) {
				const FeaturePoint* fpt = iter->second[i];
				std::pair<int, int> camFrameId = std::make_pair(fpt->f,
						fpt->camId);
				if (cameraInd.count(camFrameId) == 0)
					repErr(
							"RobustBundleRTS::parseInputs - cannot find the corresponding camera (frame:%d, camId:%d)",
							fpt->f, fpt->camId);
				else {
					int camId = cameraInd[camFrameId];
					vecFeatPts[camId] = fpt;
				}
			}
			for (size_t i = 0; i < keyCamPoses.size(); i++) {
				if (vecFeatPts[i]) {
					meas2Ds.back().push_back(
							Meas2D(i, vecFeatPts[i]->x, vecFeatPts[i]->y));
				}
			}
			//test
			//logInfo("%d : %d\n", validNum - 1, nfpts);
		}
	}
	//test
//	logInfo("total camera poses:%d\n", keyCamPoses.size());
//	logInfo("total map points:%d\n", validNum);
//	logInfo("total feature points:%d\n", featNum);
}

void RobustBundleRTS::run() {
	run(m_nPtsCon, m_nCamsCon, m_nMaxIter);
}
void RobustBundleRTS::run(int nPtsCon, int nCamsCon, int maxIter,
		int nInnerMaxIter) {
	parseInputs(mapAndVecFeatPts);
    try{
    bundleAdjustRobust(nCamsCon, Ks, Rs, Ts, nPtsCon, pt3Ds, meas2Ds, m_maxErr,
            maxIter, nInnerMaxIter);
    //}catch(std::exception& e){
    }catch(...){
        std::cerr << "bundle adjustment fail" << std::endl;
    }
}

void RobustBundleRTS::constructCameraGraphs() {
	assert(pCoSLAM && firstKeyFrame && lastKeyFrame);
	int numCams = pCoSLAM->numCams;
	for (int c = 0; c < numCams; c++) {
		camGraphs[c].clear();
		nonKeyCamNodeId[c].clear();
		nonKeyCamPoses[c].clear();
		int nTotalNode = 0;
		for (CamPoseItem* cam = firstKeyFrame->pPose[c]->cam; cam;
				cam = cam->next)
			nTotalNode++;

		camGraphs[c].reserve(nTotalNode, nTotalNode);

		//add nodes
		for (CamPoseItem* cam = firstKeyFrame->pPose[c]->cam; cam;
				cam = cam->next) {
			CamPoseNode* node = camGraphs[c].newNode();
			node->set(cam->f, cam->camId, cam->R, cam->t);

			//record the pointers
			nonKeyCamNodeId[c][cam] = node->id;
			nonKeyCamPoses[c].push_back(cam);
		}

		//set fixed nodes on key frames
		for (KeyPose* kp = firstKeyFrame->pPose[c];
				kp && kp->frame <= lastKeyFrame->f; kp = kp->next) {
			CamPoseItem* cam = kp->cam;
			int id = nonKeyCamNodeId[c][cam];
			CamPoseNode* fixedNode = &camGraphs[c].poseNodes[id];
			fixedNode->fixed = true;
		}

		//add edges
		CamPoseItem* cam0 = firstKeyFrame->pPose[c]->cam->next;
		assert(cam0);
		for (CamPoseItem* cam = cam0; cam; cam = cam->next) {
			int id1 = nonKeyCamNodeId[c][cam->pre];
			int id2 = nonKeyCamNodeId[c][cam];
			CamPoseEdge* edge = camGraphs[c].addEdge();

			double R[9], t[3];
			getRigidTransFromTo(cam->pre->R, cam->pre->t, cam->R, cam->t, R, t);
			edge->set(id1, id2, R, t);
		}
	}
}
void RobustBundleRTS::updateNonKeyCameraPoses() {
	assert(pCoSLAM);
	int nCam = pCoSLAM->numCams;
	for (int c = 0; c < nCam; c++) {
		camGraphs[c].computeNewCameraRotations();
		camGraphs[c].computeNewCameraTranslations();
		//update the camera poses;
		for (int i = 0; i < camGraphs[c].nNodes; i++) {
			CamPoseItem* cam = nonKeyCamPoses[c][i];
			if (!camGraphs[c].poseNodes[i].fixed) {
				memcpy(cam->R, camGraphs[c].poseNodes[i].newR,
						sizeof(double) * 9);
				memcpy(cam->t, camGraphs[c].poseNodes[i].newt,
						sizeof(double) * 3);
			}
		}
	}
}
void RobustBundleRTS::updateNewPosesPoints() {
	//update the points
	for (MapPoint* mpt = pCoSLAM->curMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->lastFrame <= firstKeyFrame->f)
			continue;

		if (mpt->isLocalStatic())
			updateStaticPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, true);
		else if (mpt->isLocalDynamic())
			updateDynamicPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, true);
	}
	for (MapPoint* mpt = pCoSLAM->actMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->lastFrame <= firstKeyFrame->f)
			continue;
		if (mpt->isLocalStatic())
			updateStaticPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, true);
		else if (mpt->isLocalStatic())
			updateDynamicPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, true);
	}
}

void RobustBundleRTS::output() {
	bool updateNonKeyCameras = firstKeyFrame && pCoSLAM;
	//for updating the camera poses of non-key frames
	if (updateNonKeyCameras) {
		constructCameraGraphs();
	}
	//update the camera positions
	assert(Rs.size() == Ts.size());
	for (size_t c = 0; c < Rs.size(); c++) {

		memcpy(keyCamPoses[c]->R, Rs[c].data, sizeof(double) * 9);
		memcpy(keyCamPoses[c]->t, Ts[c].data, sizeof(double) * 3);

		//update the camera poses of fixed nodes in camera graphs ( for updating the non-key frames)
		if (updateNonKeyCameras) {
			int camId = keyCamPoses[c]->camId;
			int nodeId = nonKeyCamNodeId[camId][keyCamPoses[c]];
			memcpy(camGraphs[camId].poseNodes[nodeId].R, keyCamPoses[c]->R,
					sizeof(double) * 9);
			memcpy(camGraphs[camId].poseNodes[nodeId].t, keyCamPoses[c]->t,
					sizeof(double) * 3);
		}
	}

	//update the map points
	for (size_t i = 0; i < pt3Ds.size(); i++) {
		memcpy(mapPoints[i]->M, pt3Ds[i].M, sizeof(double) * 3);
		bool outlier = false;
		for (size_t j = 0; j < meas2Ds[i].size(); j++) {
			if (meas2Ds[i][j].outlier > 0) {
				outlier = true;
				break;
			}
		}
		if (outlier)
			mapPoints[i]->setFalse();
	}

	//update the camera poses of non-key frames
	if (updateNonKeyCameras) {
		updateNonKeyCameraPoses();
		updateNewPosesPoints();
	}
}
