/*
 * SL_Bundle.cpp
 *
 *  Created on: 2011-1-20
 *      Author: Danping Zou
 */

#include "SL_error.h"
#include "SL_CoSLAM.h"
#include "SL_CoSLAMBA.h"
#include "SL_GlobParam.h"

#include "slam/SL_SLAMHelper.h"
#include "slam/SL_CoSLAMHelper.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Geometry.h"
#include "geometry/SL_Quaternion.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_RigidTransform.h"
#include "extern/sba-1.6/sba.h"
#include "tools/SL_Tictoc.h"
#include <cassert>

BundleRTS::BundleRTS() :
		processed(false), pCoSLAM(0), firstKeyFrame(0), lastKeyFrame(0), numPts(
				0), numCams(0), numMeas(0), numPtsCon(0), numCamsCon(0), m_paramVec(
				0) {
	m_globs.rot0params = 0;
	m_nPtsCon = 0;
	m_nCamsCon = 0;
	m_nMaxIter = 10;
}
BundleRTS::~BundleRTS() {
	if (m_paramVec) {
		delete[] m_paramVec;
		m_paramVec = 0;
	}
	if (m_globs.rot0params) {
		delete[] m_globs.rot0params;
		m_globs.rot0params = 0;
	}
}
void BundleRTS::setCoSLAM(CoSLAM* coSLAM) {
	pCoSLAM = coSLAM;
}
void BundleRTS::setFirstKeyFrame(KeyFrame* keyFrame) {
	firstKeyFrame = keyFrame;
}
void BundleRTS::setLastKeyFrame(KeyFrame* keyFrame) {
	lastKeyFrame = keyFrame;
}
void BundleRTS::addKeyFrames() {
	assert(firstKeyFrame && lastKeyFrame);
	//add key poses for bundle adjustment
	for (KeyFrame* kf = firstKeyFrame; kf != lastKeyFrame->next;
			kf = kf->next) {
		for (int i = 0; i < kf->nCam; i++) {
			addKeyCamera(kf->pPose[i]->K, kf->pPose[i]->cam);
		}
	}
}
void BundleRTS::addPoints() {
	//add feature points
	assert(firstKeyFrame && lastKeyFrame);
	for (KeyFrame* kf = firstKeyFrame; kf != lastKeyFrame->next;
			kf = kf->next) {
		for (int c = 0; c < kf->nCam; c++) {
			int iCam = kf->pPose[c]->cam->camId;
			int frm = kf->f;

			FeaturePoint* pHead = kf->pPose[c]->pHead;
			FeaturePoint* pTail = kf->pPose[c]->pTail;

			if (!pHead)
				continue;

			int nAdded = 0;
			for (FeaturePoint* p = pHead; p != pTail->next; p = p->next) {
				//only static map points are used for bundle adjustment
				if (p->mpt && p->mpt->isLocalStatic()) {
					add2DPoint(p->mpt, p);
					//test
					nAdded++;
				}
			}
			logInfo("camId : %d, frame : %d -- added %d\n", iCam, frm, nAdded);
		}
	}
	//add dynamic points
	for (KeyFrame* kf = firstKeyFrame; kf != lastKeyFrame->next;
			kf = kf->next) {
		for (MapPoint* mpt = kf->dynMapPoints.getHead(); mpt; mpt = mpt->next) {
			for (int c = 0; c < kf->nCam; c++) {
				if (mpt->pFeatures[c]) {
					add2DPoint(mpt, mpt->pFeatures[c]);
				}
			}
		}
	}
}
void BundleRTS::init(int nMaxPoints, int nMaxCameras) {
	numPts = 0;
	numCams = 0;
	numMeas = 0;

	numPtsCon = 0;
	numCamsCon = 0;

	if (m_paramVec) {
		delete[] m_paramVec;
		m_paramVec = 0;
	}
	if (m_globs.rot0params) {
		delete[] m_globs.rot0params;
		m_globs.rot0params = 0;
	}

	keyCamPoses.clear();
	mapPoints.clear();
	featPoints.clear();

	mapPointInd.clear();
	cameraInd.clear();
	_mapPts.clear();

	maxNumPts = nMaxPoints;
	maxNumCams = nMaxCameras;

	Ks.resize(maxNumCams, 9);
	ms.resize(maxNumPts * maxNumCams, 2);
	Rs.resize(maxNumCams, 9);
	Ts.resize(maxNumCams, 3);
	Ms.resize(maxNumPts, 3);
	vmask.resize(maxNumPts, maxNumCams);
	memset(vmask.data, 0, maxNumPts * maxNumCams);

	keyCamPoses.reserve(maxNumCams);
	keyCamPoses.reserve(maxNumPts);
}

//void BundleRTS::setRT(int i , const double* KMat , const double* R , const double* T) {
//	if (i >= numPts || i < 0)
//		error("BundleRTS::setRT - invalid camera %d!", i);
//	double pKs = Ks.data + 9 * i;
//	double pRs = Rs.data + 9 * i;
//	double pTs = Ts.data + 3 * i;
//
//	memcpy(pKs, KMat, sizeof(double) * 9);
//	memcpy(pRs, R, sizeof(double) * 9);
//	memcpy(pTs, T, sizeof(double) * 3);
//}

void BundleRTS::compressMeasurements() {
	int N = numPts * numCams;
	int offset = 0;
	double * pCur = ms.data;
	for (int i = 0; i < N; i++) {
		if (vmask.data[i] == 0)
			offset++;
		else {
			double* pPrev = pCur - 2 * offset;
			pPrev[0] = pCur[0];
			pPrev[1] = pCur[1];
		}
		pCur += 2;
	}
}
void BundleRTS::addCorrespond(
		std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts,
		MapPoint* pt3d,
		const FeaturePoint* featPt) {
	mapPts[pt3d].push_back(featPt);
}
bool BundleRTS::checkReprojError(
		std::map<MapPoint*, std::vector<const FeaturePoint*> >::iterator& iter
		, double errMax) {
	MapPoint* p = iter->first;
	std::vector<const FeaturePoint*>& vecfpt = iter->second;
	size_t nfpt = vecfpt.size();
	for (size_t i = 0; i < nfpt; i++) {
		const FeaturePoint* fp = vecfpt[i];
		double rm[2];
		int f = fp->f;
		int camId = fp->camId;

		std::pair<int, int> camFrameId = std::make_pair(f, camId);
		if (cameraInd.count(camFrameId) == 0)
			return false;

		int camInd = cameraInd[camFrameId];
		CamPoseItem* cam = keyCamPoses[camInd];
		project(Ks.data + 9 * camInd, cam->R, cam->t, p->M, rm);

		double d = dist2(rm, fp->m);
		if (d > errMax)
			return false;
	}
	return true;
}
void BundleRTS::refineInput(
		std::map<MapPoint*, std::vector<const FeaturePoint*> >& mapPts) {
	std::map<MapPoint*, std::vector<const FeaturePoint*> >::iterator iter;
	//test
	logInfo("BundleRTS::refineInput\n");
	int nOutlier = 0;
	for (iter = mapPts.begin(); iter != mapPts.end(); iter++) {
		size_t nfpts = iter->second.size();
		if (nfpts > 1 && checkReprojError(iter, 3.5)) {
			for (size_t i = 0; i < nfpts; i++)
				_add2DPoint(iter->first, iter->second[i]);
		} else
			nOutlier++;
	}
	logInfo("Removed %d outliers\n", nOutlier);
}
void BundleRTS::addKeyCamera(const double* KMat, CamPoseItem* camPos) {
	if (numCams > maxNumCams)
		repErr("BundleRTS::addCamera - error! numCams (%d) > maxNumCams (%d)",
				numCams, maxNumCams);

	int frame = camPos->f;
	int camId = camPos->camId;

	cameraInd[std::make_pair(frame, camId)] = numCams;

	double* pKs = Ks.data + 9 * numCams;
	double* pRs = Rs.data + 9 * numCams;
	double* pTs = Ts.data + 3 * numCams;

	memcpy(pKs, KMat, sizeof(double) * 9);
	memcpy(pRs, camPos->R, sizeof(double) * 9);
	memcpy(pTs, camPos->t, sizeof(double) * 3);

	numCams++;

	keyCamPoses.push_back(camPos);
}
int BundleRTS::add3DPoint(MapPoint* pt3d) {
	if (numPts >= maxNumPts)
		repErr("BundleRTS::add3DPoint - error ! numPts (%d) >= maxNumCams (%d)",
				numPts, maxNumPts);
	double* pMs = Ms.data + 3 * numPts;
	memcpy(pMs, pt3d->M, sizeof(double) * 3);

	mapPoints.push_back(pt3d);
	mapPointInd[pt3d] = numPts;
	int nRet = numPts;
	numPts++;
	return nRet;
}
void BundleRTS::_add2DPoint(MapPoint* pt3d, const FeaturePoint* featPt) {
	int ptInd;
	if (mapPointInd.count(pt3d) == 0)
		ptInd = add3DPoint(pt3d);
	else
		ptInd = mapPointInd[pt3d];

	std::pair<int, int> camFrameId = std::make_pair(featPt->f, featPt->camId);
	if (cameraInd.count(camFrameId) == 0)
		repErr(
				"BundleRTS::add2DPoint - cannot find the corresponding camera (frame:%d, camId:%d)",
				featPt->f, featPt->camId);
	else {
		//add this feature point
		int camInd = cameraInd[camFrameId];
		int ind = ptInd * numCams + camInd;
		ms.data[2 * ind] = featPt->x;
		ms.data[2 * ind + 1] = featPt->y;
		vmask.data[ind] = 1;
	}
	numMeas++;
}
void BundleRTS::add2DPoint(MapPoint* pt3d, const FeaturePoint* featPt) {
	addCorrespond(_mapPts, pt3d, featPt);
}

void BundleRTS::apply(int nPtsCon, int nCamsCon, int maxIter) {
	run(nPtsCon, nCamsCon, maxIter);
	output();
}
void BundleRTS::apply() {
	run();
	output();
}

void BundleRTS::run() {
	run(m_nPtsCon, m_nCamsCon, m_nMaxIter);
}
void BundleRTS::run(int nPtsCon, int nCamsCon, int maxIter) {
	refineInput(_mapPts);

	numPtsCon = nPtsCon;
	numCamsCon = nCamsCon;

	compressMeasurements();

	const int cnp = 11; //5:intrinsic parameters 6:extrinsic parameters
	const int pnp = 3;
	const int mnp = 2;

	m_globs.cnp = cnp;
	m_globs.pnp = pnp;
	m_globs.mnp = mnp;

	if (m_globs.rot0params) {
		delete[] m_globs.rot0params;
	}
	m_globs.rot0params = new double[FULLQUATSZ * numCams];

	//set initial camera parameters
	for (int i = 0; i < numCams; ++i) {
		mat2quat(Rs + 9 * i, m_globs.rot0params + 4 * i);
	}

	m_globs.intrcalib = 0;
	m_globs.nccalib = 5;

	m_globs.camparams = 0;
	m_globs.ptparams = 0;

	/* call sparse LM routine */
	double opts[SBA_OPTSSZ];
	opts[0] = SBA_INIT_MU * 1E-4;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = 0; //0.05 * numMeas; // uncomment to force termination if the average reprojection error drops below 0.05
	opts[4] = 1E-16; // uncomment to force termination if the relative reduction in the RMS reprojection error drops below 1E-05

	if (m_paramVec)
		delete[] m_paramVec;
	m_paramVec = new double[numCams * cnp + numPts * pnp];

	double * pParamVec = m_paramVec;
	double* pKs = Ks.data;
	double* pTs = Ts.data;
	for (int i = 0; i < numCams; ++i) {
		pParamVec[0] = pKs[0];
		pParamVec[1] = pKs[2];
		pParamVec[2] = pKs[5];
		pParamVec[3] = pKs[4] / pKs[0];
		pParamVec[4] = pKs[1];

		pParamVec[5] = 0;
		pParamVec[6] = 0;
		pParamVec[7] = 0;
		pParamVec[8] = pTs[0];
		pParamVec[9] = pTs[1];
		pParamVec[10] = pTs[2];

		pParamVec += cnp;
		pKs += 9;
		pTs += 3;
	}
	double* pParamPoints = m_paramVec + numCams * cnp;
	memcpy(pParamPoints, Ms.data, numPts * 3 * sizeof(double));

	double sbaInfo[SBA_INFOSZ];
	if (sba_motstr_levmar_x(numPts, numPtsCon, numCams, numCamsCon, vmask,
			m_paramVec, cnp, pnp, ms.data, 0, mnp, img_projsKRTS_x,
			img_projsKRTS_jac_x, (void *) (&m_globs), maxIter, 0, opts,
			sbaInfo) == SBA_ERROR) {
		//for debug
		//save the bundle data for debug
		
		
		repErr("bundle adjustment failed!\n");
	}
	//test
	logInfo(
			"initial error:%lf, final error:%lf #iterations:%lf stop reason:%lf\n",
			sqrt(sbaInfo[0] / numMeas), sqrt(sbaInfo[1] / numMeas), sbaInfo[5],
			sbaInfo[6]);
}

void BundleRTS::constructCameraGraphs() {
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
void BundleRTS::updateNonKeyCameraPoses() {
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
void BundleRTS::updateNewPosesPoints() {
	//update the points
	for (MapPoint* mpt = pCoSLAM->curMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->lastFrame <= firstKeyFrame->f)
			continue;
//		if (mpt->bLocalDyn == TYPE_MAPPOINT_STATIC || mpt->bLocalDyn == TYPE_MAPPOINT_UNCERTAIN
//		)
		if (mpt->isLocalDynamic())
			updateStaticPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, false);
		else if (mpt->isLocalDynamic())
			updateDynamicPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, false);
	}
	for (MapPoint* mpt = pCoSLAM->actMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->lastFrame <= firstKeyFrame->f)
			continue;
//		if (mpt->bLocalDyn == TYPE_MAPPOINT_STATIC || mpt->bLocalDyn == TYPE_MAPPOINT_UNCERTAIN
//		)
		if (mpt->isLocalStatic())
			updateStaticPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, false);
		else if (mpt->isLocalDynamic())
			updateDynamicPointPosition(pCoSLAM->numCams, mpt,
					Const::PIXEL_ERR_VAR, false);
	}
}
void BundleRTS::output() {
	bool updateNonKeyCameras = firstKeyFrame && pCoSLAM;
	//for updating the camera poses of non-key frames
	if (updateNonKeyCameras)
		constructCameraGraphs();

	const int cnp = 11; //5:intrinsic parameters 6:extrinsic parameters
	const int pnp = 3;

	double* pParamVec = m_paramVec;
	//update the camera positions
	double dq[4], q[4];
	for (int c = 0; c < numCams; c++) {
		//rotation
		_MK_QUAT_FRM_VEC(dq, pParamVec+5);
		quatMultFast(dq, m_globs.rot0params + c * FULLQUATSZ, q);
		//update the camera poses
		quat2mat(q, keyCamPoses[c]->R);
		memcpy(keyCamPoses[c]->t, pParamVec + 8, sizeof(double) * 3);

		//update the camera poses of fixed nodes in camera graphs
		if (updateNonKeyCameras) {
			int camId = keyCamPoses[c]->camId;
			int nodeId = nonKeyCamNodeId[camId][keyCamPoses[c]];
			memcpy(camGraphs[camId].poseNodes[nodeId].R, keyCamPoses[c]->R,
					sizeof(double) * 9);
			memcpy(camGraphs[camId].poseNodes[nodeId].t, keyCamPoses[c]->t,
					sizeof(double) * 3);
		}
		pParamVec += cnp;
	}
	//update the map points
	double* pParamPoints = m_paramVec + numCams * cnp;
	for (int i = 0; i < numPts; i++) {
		memcpy(mapPoints[i]->M, pParamPoints, sizeof(double) * pnp);
		pParamPoints += pnp;
	}
	//update the camera poses of non-key frames
	if (updateNonKeyCameras) {
		updateNonKeyCameraPoses();
		updateNewPosesPoints();
	}
}
