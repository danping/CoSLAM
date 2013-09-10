/*
 * SL_SingleSLAM.cpp
 *
 *  Created on: 2011-1-12
 *      Author: Danping Zou
 */
#include "SL_GlobParam.h"
#include "SL_SingleSLAM.h"
#include "SL_GlobParam.h"

#include "slam/SL_MapPoint.h"
#include "slam/SL_IntraCamPose.h"
#include "slam/SL_SLAMHelper.h"
#include "slam/SL_SolvePnP.h"

#include "imgproc/SL_ImageOp.h"

#include "math/SL_LinAlg.h"
#include "matching/SL_GuidedSSDMatcher.h"

#include "geometry/SL_Distortion.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_5point.h"
#include <cfloat>

//#define DEBUG_MODE 1
SingleSLAM::SingleSLAM() :
		SingleSLAMParam(), W(0), H(0), blkW(0), blkH(0), m_nMappedStaticPts(0), startFrameInVideo(
				0) {
	m_smallScale = 0.3;
}
SingleSLAM::~SingleSLAM() {
}
void SingleSLAM::propagateFeatureStates() {
	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.empty())
			continue;
		Track2DNode* node = tk.tail;
		if (node->pre) {
			//propagate the type of feature points
			node->pt->type = node->pre->pt->type;
			if (node->pre->pt->mpt) {
				MapPoint* pMapPt = node->pre->pt->mpt;
				if (pMapPt->state != STATE_MAPPOINT_CURRENT)
					continue;
				//map correspondence propagation
				if (!pMapPt->isFalse()) {
					pMapPt->pFeatures[camId] = node->pt;
					node->pt->mpt = pMapPt;
					pMapPt->lastFrame = currentFrame();
					if (pMapPt->isCertainStatic())
						pMapPt->staticFrameNum++;
					node->pt->reprojErr = node->pre->pt->reprojErr;
				}
			}
		}
	}
}
int SingleSLAM::getStaticMappedTrackNodes(std::vector<Track2DNode*>& nodes) {
	int k = 0;
	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.empty())
			continue;
		Track2DNode* node = tk.tail;
		if (node->pt->mpt) {
			if (node->pt->mpt->isCertainStatic()) {
				nodes.push_back(node);
				k++;
			}
		}
	}
	return k;
}
int SingleSLAM::getUnMappedTrackNodes(std::vector<Track2DNode*>& nodes,
		int minLen) {
	int k = 0;
	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.empty() || tk.length() < minLen)
			continue;
		Track2DNode* node = tk.tail;
		if (!node->pt->mpt) {
			nodes.push_back(node);
			k++;
		}
	}
	return k;
}
int SingleSLAM::getUnMappedAndDynamicTrackNodes(
		std::vector<Track2DNode*>& nodes, int minLen) {
	int k = 0;
	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.empty() || tk.length() < minLen)
			continue;
		Track2DNode* node = tk.tail;
		if (!node->pt->mpt || node->pt->mpt->isCertainDynamic()) {
			nodes.push_back(node);
			k++;
		}
	}
	return k;
}
int SingleSLAM::getNumMappedFeatPts() {
	int curFrame = currentFrame();
	FeaturePoint* pHead = m_featPts.getFrameHead(curFrame);
	assert(pHead);
	FeaturePoint* pEnd = m_featPts.getFrameTail(curFrame)->next;

	int num = 0;
	FeaturePoint* p = pHead;
	while (p != pEnd) {
		if (p->mpt)
			num++;
		p = p->next;
	}
	return num;
}
int SingleSLAM::getNumMappedStaticPts() {
	int curFrame = currentFrame();
	FeaturePoint* pHead = m_featPts.getFrameHead(curFrame);
	assert(pHead);
	FeaturePoint* pEnd = m_featPts.getFrameTail(curFrame)->next;

	int num = 0;
	FeaturePoint* p = pHead;
	while (p != pEnd) {
		if (p->mpt && p->mpt->isCertainStatic())
			num++;
		p = p->next;
	}
	m_nMappedStaticPts = num;
	return num;
}
int SingleSLAM::getNumMultiMappedFeatPts() {
	int curFrame = currentFrame();
	FeaturePoint* pHead = m_featPts.getFrameHead(curFrame);
	assert(pHead);
	FeaturePoint* pEnd = m_featPts.getFrameTail(curFrame)->next;

	int num = 0;
	FeaturePoint* p = pHead;
	while (p != pEnd) {
		if (p->mpt && p->mpt->numVisCam > 1)
			num++;
		p = p->next;
	}
	return num;
}
int SingleSLAM::getUnMappedAndTrackedFeatPts(
		std::vector<FeaturePoint*>& vecFeatPts, Mat_d* featPts, int trackLen) {
	int k = 0;
	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.empty())
			continue;
		if (tk.f2 - tk.f1 >= trackLen && !tk.tail->pt->mpt) {
			vecFeatPts.push_back(tk.tail->pt);
			k++;
		}
	}
	if (featPts) {
		featPts->resize(k, 2);
		for (int i = 0; i < k; i++) {
			featPts->data[2 * i] = vecFeatPts[i]->x;
			featPts->data[2 * i + 1] = vecFeatPts[i]->y;
		}
	}
	return k;
}
int SingleSLAM::getTrackedFeatPts(std::vector<FeaturePoint*>& vecFeatPts,
		Mat_d* featPts, int trackLen) {
	int k = 0;
	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.empty())
			continue;
		if (tk.f2 - tk.f1 >= trackLen) {
			vecFeatPts.push_back(tk.tail->pt);
			k++;
		}
	}
	if (featPts) {
		featPts->resize(k, 2);
		for (int i = 0; i < k; i++) {
			featPts->data[2 * i] = vecFeatPts[i]->x;
			featPts->data[2 * i + 1] = vecFeatPts[i]->y;
		}
	}
	return k;
}
int SingleSLAM::getUnMappedFeatPts(std::vector<FeaturePoint*>& vecFeatPts,
		Mat_d* featPts) {
	return getMappedFeatPts(0, 0, vecFeatPts, featPts);
}
int SingleSLAM::getMonoMappedFeatPts(std::vector<FeaturePoint*>& vecFeatPts,
		Mat_d* featPts) {
	return getMappedFeatPts(1, 1, vecFeatPts, featPts);
}
int SingleSLAM::getAllMappedFeatPts(std::vector<FeaturePoint*>& vecFeatPts,
		Mat_d* featPts) {
	return getMappedFeatPts(1, SLAM_MAX_NUM, vecFeatPts, featPts);
}
int SingleSLAM::getMappedFeatPts(int numVis1, int numVis2,
		std::vector<FeaturePoint*>& mappedFeatPts, Mat_d* featPts) {
	assert(numVis2 >= numVis1);
	int curFrame = currentFrame();
	FeaturePoint* pHead = m_featPts.getFrameHead(curFrame);
	if (!pHead)
		return 0;
	FeaturePoint* pEnd = m_featPts.getFrameTail(curFrame)->next;

	mappedFeatPts.clear();
	int maxNum = m_featPts.frameNum[curFrame];
	mappedFeatPts.reserve(maxNum);

	FeaturePoint* p = pHead;
	while (p != pEnd) {
		if (numVis1 <= 0) {
			if (numVis2 <= 0) {
				if (!p->mpt)
					mappedFeatPts.push_back(p);
			} else if (!p->mpt || (p->mpt && p->mpt->numVisCam <= numVis2))
				mappedFeatPts.push_back(p);

		} else {
			if (p->mpt && p->mpt->numVisCam <= numVis2)
				mappedFeatPts.push_back(p);
		}
		p = p->next;
	}

	if (featPts) {
		featPts->resize(mappedFeatPts.size(), 2);
		for (int i = 0; i < featPts->rows; i++) {
			featPts->data[2 * i] = mappedFeatPts[i]->x;
			featPts->data[2 * i + 1] = mappedFeatPts[i]->y;
		}
	}
	return (int) mappedFeatPts.size();
}
int SingleSLAM::getMappedDynPts(std::vector<FeaturePoint*>& mappedDynPts) {
	int curFrame = currentFrame();
	FeaturePoint* pHead = m_featPts.getFrameHead(curFrame);
	if (!pHead)
		return 0;
	FeaturePoint* pEnd = m_featPts.getFrameTail(curFrame)->next;

	mappedDynPts.clear();
	int maxNum = m_featPts.frameNum[curFrame];
	mappedDynPts.reserve(maxNum);

	int n = 0;
	for (FeaturePoint* fp = pHead; fp != pEnd; fp = fp->next) {
		if (fp->mpt && fp->mpt->isCertainDynamic()) {
			mappedDynPts.push_back(fp);
			n++;
		}
	}
	return n;

}
int SingleSLAM::initTracker(int f, vector<FeaturePoint*>& existFeatPoints) {

	W = videoReader->_w;
	H = videoReader->_h;

	blkW = W / nColBlk;
	blkH = H / nRowBlk;

	m_rgb.resize(W, H);
	m_img.resize(W, H);
	m_smallImg.resize((int) W * m_smallScale, (int) H * m_smallScale);

	//videoReader->getCurGrayImage(m_img.data);
	videoReader->readCurFrame(m_rgb.data, m_img.data);

//	//treat the existing feature points as seeds to track
//	std::vector<FeaturePoint*> pTmpPts;
//	if (m_featPts.getFrameHead(f)) {
//		FeaturePoint* pHead = m_featPts.getFrameHead(f);
//		FeaturePoint* pTail = m_featPts.getFrameTail(f)->next;
//
//		for (FeaturePoint* pPt = pHead; pPt != pTail; pPt = pPt->next) {
//			pTmpPts.push_back(pPt);
//		}
//	}

	V3D_GPU::KLT_SequenceTrackerConfig cfg;
    cfg.minDistance = Param::minDistance;
	cfg.minCornerness = Param::minCornerness;
	cfg.nLevels = Param::nLevels;
	cfg.windowWidth = Param::windowWidth;
	cfg.convergenceThreshold = Param::convergeThreshold;
	cfg.SSD_Threshold = Param::SSD_Threshold;
	cfg.trackWithGain = Param::trackWithGain;

	m_tracker.init(camId, W, H, &cfg);
	m_tracker.setIntrinsicParam(K, iK, k_ud);

	if (existFeatPoints.empty())
		return m_tracker.first(f, m_img.data, m_featPts);

	return m_tracker.first(f, m_img.data, existFeatPoints, m_featPts);
}
int SingleSLAM::initTracker(int f) {
	vector<FeaturePoint*> existPts;
	return initTracker(f, existPts);
}
void SingleSLAM::readFirstFrame() {
	//videoReader->getCurGrayImage(m_img.data);
	videoReader->readCurFrame(m_rgb.data, m_img.data);
	scaleDownAvg(m_img, m_smallImg, m_smallScale);
}
void SingleSLAM::grabReadFrame() {
	videoReader->grabFrame();
	//videoReader->getCurGrayImage(m_img.data);
	videoReader->readCurFrame(m_rgb.data, m_img.data);

	cv::Mat cvImg(m_img.rows, m_img.cols, CV_8UC1, m_img.data);
	cv::Mat cvSmallImg(m_smallImg.rows, m_smallImg.cols, CV_8UC1,
			m_smallImg.data);
	cv::resize(cvImg, cvSmallImg, cv::Size(m_smallImg.cols, m_smallImg.rows));
	//scaleDownAvg(m_img, m_smallImg, m_smallScale);
}

int SingleSLAM::trackFeaturePoints() {
	return m_tracker.next(m_img.data, m_featPts);
}

void SingleSLAM::updateCamParamForFeatPts(const double* intrin,
		CamPoseItem* camPos) {
	int frame = currentFrame();
	FeaturePoint* phead = m_featPts.getFrameHead(frame);
	if (phead) {
		FeaturePoint* pend = m_featPts.getFrameTail(frame)->next;
		for (FeaturePoint* p = phead; p != pend; p = p->next) {
			p->setIntrinsic(intrin);
			p->setCameraPose(camPos);
		}
	}
}
int SingleSLAM::chooseStaticFeatPts(std::vector<FeaturePoint*>& featPts) {
	int len = nRowBlk * nColBlk + 1;

	Track2D** tracks = new Track2D*[len];
	memset(tracks, 0, sizeof(Track2D*) * len);

	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D* tk = &m_tracker.m_tks[i];
		if (tk->empty())
			continue;

		FeaturePoint* fp = tk->tail->pt;
		if (fp->type == TYPE_FEATPOINT_STATIC
				|| (fp->mpt && fp->mpt->isCertainStatic())) {
			//compute in which block the feature point should lie
			int bx = static_cast<int>((fp->x) / blkW);
			int by = static_cast<int>((fp->y) / blkH);
			if (bx >= nColBlk || by >= nRowBlk)
				continue;

			int bi = by * nColBlk + bx;
			assert(bi < len);

			Track2D* tkOld = tracks[bi];
			if (tkOld == 0) {
				tracks[bi] = tk;
			} else {
				FeaturePoint* fpOld = tkOld->tail->pt;
				if (!fpOld->mpt) {
					if (fp->mpt) {
						tracks[bi] = tk;
					} else {
						if (tkOld->length() < tk->length()) {
							tracks[bi] = tk;
						}
					}
				}
			}
		}
	}

	featPts.reserve(nRowBlk * nColBlk * 2);

	int k = 0;
	for (int i = 0; i < len; i++) {
		if (tracks[i]) {
			featPts.push_back(tracks[i]->tail->pt);
			k++;
		}
	}
	delete[] tracks;
	return k;
}
int SingleSLAM::chooseDynamicFeatPts(std::vector<FeaturePoint*>& featPts) {
	int len = nRowBlk * nColBlk + 1;

	Track2D** tracks = new Track2D*[len];
	memset(tracks, 0, sizeof(Track2D*) * len);

	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D* tk = &m_tracker.m_tks[i];
		if (tk->empty())
			continue;

		FeaturePoint* fp = tk->tail->pt;
		if (!fp->mpt)
			continue;
		if (fp->mpt->numVisCam < 2)
			continue;
		if (fp->mpt->isCertainDynamic()
				|| (fp->mpt->isUncertain() && fp->mpt->bNewPt)) {
			//compute in which block the feature point should lie
			int bx = static_cast<int>((fp->x) / blkW);
			int by = static_cast<int>((fp->y) / blkH);
			if (bx >= nColBlk || by >= nRowBlk)
				continue;

			int bi = by * nColBlk + bx;
			assert(bi < len);

			Track2D* tkOld = tracks[bi];
			if (tkOld == 0) {
				tracks[bi] = tk;
			} else {
				FeaturePoint* fpOld = tkOld->tail->pt;
				if (fpOld->mpt->numVisCam < fp->mpt->numVisCam) {
					tracks[bi] = tk;
				}
			}

		}
	}
	featPts.reserve(nRowBlk * nColBlk * 2);
	int k = 0;
	for (int i = 0; i < len; i++) {
		if (tracks[i]) {
			featPts.push_back(tracks[i]->tail->pt);
			k++;
		}
	}
	delete[] tracks;
	return k;
}
int SingleSLAM::fastPoseUpdate3D() {
	propagateFeatureStates();
//get the feature points corresponding to the map points
	std::vector<Track2DNode*> nodes;
	int num = getStaticMappedTrackNodes(nodes);
	if (num < 5) {
		repErr(
				"[camera id:%d]intra-camera pose update failed! less than five static map points (%d)",
				camId, num);
		return -1;
	}

//choose the feature points for pose estimation
	std::vector<FeaturePoint*> featPts;
	int iChoose = chooseStaticFeatPts(featPts);
//test
	logInfo("number of chosen features :%d\n", iChoose);

	std::vector<FeaturePoint*> mappedFeatPts;
	std::vector<FeaturePoint*> unmappedFeatPts;

	mappedFeatPts.reserve(nRowBlk * nColBlk * 2);
	unmappedFeatPts.reserve(nRowBlk * nColBlk * 2);

	for (size_t i = 0; i < featPts.size(); i++) {
		if (featPts[i]->mpt)
			mappedFeatPts.push_back(featPts[i]);
		else if (featPts[i]->preFrame)
			unmappedFeatPts.push_back(featPts[i]);
	}

	//get the 2D-3D corresponding points
	int n3D2Ds = mappedFeatPts.size();
	Mat_d ms(n3D2Ds, 2), Ms(n3D2Ds, 3), repErrs(n3D2Ds, 1);
	for (int i = 0; i < n3D2Ds; i++) {
		FeaturePoint* fp = mappedFeatPts[i];
		ms.data[2 * i] = fp->x;
		ms.data[2 * i + 1] = fp->y;

		Ms.data[3 * i] = fp->mpt->x;
		Ms.data[3 * i + 1] = fp->mpt->y;
		Ms.data[3 * i + 2] = fp->mpt->z;

		repErrs.data[i] = fp->reprojErr;
	}

	//get the 2D-2D corresponding points
	int n2D2Ds = unmappedFeatPts.size();
	Mat_d ums(n2D2Ds, 2), umspre(n2D2Ds, 2), Rpre(n2D2Ds, 9), tpre(n2D2Ds, 3);
	for (int i = 0; i < n2D2Ds; i++) {
		FeaturePoint* fp = unmappedFeatPts[i];

		ums.data[2 * i] = fp->x;
		ums.data[2 * i + 1] = fp->y;

		//travel back to the frame of the first appearance
		//while (fp->preFrame) {
		fp = fp->preFrame;
		//}
		assert(fp);

		umspre.data[2 * i] = fp->x;
		umspre.data[2 * i + 1] = fp->y;

		doubleArrCopy(Rpre.data, i, fp->cam->R, 9);
		doubleArrCopy(tpre.data, i, fp->cam->t, 3);
	}

//estimate the camera pose using both 2D-2D and 3D-2D correspondences
	double R[9], t[3];
	double* cR = m_camPos.current()->R;
	double* cT = m_camPos.current()->t;

//	//test
//	logInfo("==============start of camId:%d=================\n", camId);
//	logInfo("n3D2D:%d, n2D2D:%d\n", n3D2Ds, n2D2Ds);
//
//	write(K, "/home/tsou/data/K.txt");
//	write(cR, 3, 3, "/home/tsou/data/%d_R0.txt", camId);
//	write(cT, 3, 1, "/home/tsou/data/%d_T0.txt", camId);
//	write(repErrs, "/home/tsou/data/%d_errs.txt", camId);
//	write(Ms, "/home/tsou/data/%d_Ms.txt", camId);
//	write(ms, "/home/tsou/data/%d_ms.txt", camId);
//	write(Rpre, "/home/tsou/data/%d_Rpre.txt", camId);
//	write(tpre, "/home/tsou/data/%d_tpre.txt", camId);
//	write(umspre, "/home/tsou/data/%d_umspre.txt", camId);
//	write(ums, "/home/tsou/data/%d_ums.txt", camId);
//
//	//test
//	printMat(3, 3, cR);
//	printMat(3, 1, cT);

	IntraCamPoseOption opt;
	double R_tmp[9], t_tmp[3];
	intraCamEstimate(K.data, cR, cT, n3D2Ds, repErrs.data, Ms.data, ms.data, 6,
			R_tmp, t_tmp, &opt);

	if (getCameraDistance(R_tmp, t_tmp, Rpre.data, tpre.data) > 1000) {
		opt.verboseLM = 1;
		intraCamEstimateEpi(K.data, R_tmp, t_tmp, n3D2Ds, repErrs.data, Ms.data,
				ms.data, n2D2Ds, 0, Rpre.data, tpre.data, umspre.data, ums.data,
				6, R, t, &opt);
	} else {
		doubleArrCopy(R, 0, R_tmp, 9);
		doubleArrCopy(t, 0, t_tmp, 3);
	}

//	printMat(3, 3, cR);
//	printMat(3, 1, cT);
//	printMat(3, 3, R);
//	printMat(3, 1, cT);
//	logInfo("==============end of camId:%d=================\n", camId);
//	intraCamEstimate(K.data,cR,cT,n3D2Ds, repErrs.data,Ms.data,ms.data,6.0,R,t,&opt);
//	find outliers

	int numOut = 0;
	double rm[2], var[4], ivar[4];
	for (int i = 0; i < num; i++) {
		double* pM = nodes[i]->pt->mpt->M;
		double* pCov = nodes[i]->pt->mpt->cov;
		project(K, R, t, pM, rm);
		getProjectionCovMat(K, R, t, pM, pCov, var, Const::PIXEL_ERR_VAR);
		mat22Inv(var, ivar);
		double err = mahaDist2(rm, nodes[i]->pt->m, ivar);
		if (err < 1) { //inlier
			nodes[i]->pt->reprojErr = err;
			seqTriangulate(K, R, t, nodes[i]->pt->m, pM, pCov,
					Const::PIXEL_ERR_VAR);
			project(K, R, t, pM, rm);
			getProjectionCovMat(K, R, t, pM, pCov, var, Const::PIXEL_ERR_VAR);
			mat22Inv(var, ivar);
			err = mahaDist2(rm, nodes[i]->pt->m, ivar);
			if (err >= 1) {
				nodes[i]->pt->mpt->setFalse();
			}
		} else {
			//outliers
			numOut++;
			double repErr = dist2(rm, nodes[i]->pt->m);
			nodes[i]->pt->reprojErr = repErr;
			nodes[i]->pt->mpt->setUncertain();
		}
	}
	CamPoseItem* camPos = m_camPos.add(currentFrame(), camId, R, t);
	updateCamParamForFeatPts(K, camPos);

	return num;
}
#include "gui/MyApp.h"
#include <sys/stat.h>
#include <sys/types.h>
//#define DEBUG_MODE
int SingleSLAM::poseUpdate3D(bool largeErr) {
	propagateFeatureStates();
//get the feature points corresponding to the map points
	std::vector<Track2DNode*> nodes;
	int num = getStaticMappedTrackNodes(nodes);
	if (num < 1) {
		warn(
				"[camera id:%d]intra-camera pose update failed! less than five static map points (%d)",
				camId, num);
		leaveBACriticalSection();
		CoSLAM::ptr->pause();
		enterBACriticalSection();
		return -1;
	}

//choose the feature points for pose estimation
	std::vector<FeaturePoint*> featPts;
	chooseStaticFeatPts(featPts);
	std::vector<FeaturePoint*> mappedFeatPts;

	mappedFeatPts.reserve(nRowBlk * nColBlk * 2);

	for (size_t i = 0; i < featPts.size(); i++) {
		if (featPts[i]->mpt)
			mappedFeatPts.push_back(featPts[i]);
	}

//get the 2D-3D corresponding points
	int n3D2Ds = mappedFeatPts.size();
	Mat_d ms(n3D2Ds, 2), Ms(n3D2Ds, 3), covs(n3D2Ds, 9);
	for (int i = 0; i < n3D2Ds; i++) {
		FeaturePoint* fp = mappedFeatPts[i];
		ms.data[2 * i] = fp->x;
		ms.data[2 * i + 1] = fp->y;

		Ms.data[3 * i] = fp->mpt->x;
		Ms.data[3 * i + 1] = fp->mpt->y;
		Ms.data[3 * i + 2] = fp->mpt->z;

		memcpy(covs.data, fp->mpt->cov, sizeof(double) * 9);
	}

	Mat_d R(3, 3), t(3, 1);
	double* cR = m_camPos.current()->R;
	double* cT = m_camPos.current()->t;

	IntraCamPoseOption opt;

	//test
	Mat_d old_errs(n3D2Ds, 1);
	//get reprojection error beform pose update
	for (int i = 0; i < n3D2Ds; i++) {
		FeaturePoint* fp = mappedFeatPts[i];
		double m[2];
		project(K.data, cR, cT, fp->mpt->M, m);
		old_errs[i] = dist2(m, fp->m);
	}
	//end of test

	intraCamEstimate(K.data, cR, cT, Ms.rows, 0, Ms.data, ms.data,
			Param::maxErr, R.data, t.data, &opt);

	Mat_d new_errs(n3D2Ds, 1);
	for (int i = 0; i < n3D2Ds; i++) {
		FeaturePoint* fp = mappedFeatPts[i];
		double m[2];
		project(K.data, R, t, fp->mpt->M, m);
		new_errs[i] = dist2(m, fp->m);
	}

//find outliers
	int numOut = 0;
	double errThres = largeErr ? 6.0 : 2.0;
	double rm[2], var[4], ivar[4];
	for (int i = 0; i < num; i++) {
		double* pM = nodes[i]->pt->mpt->M;
		double* pCov = nodes[i]->pt->mpt->cov;
		project(K, R, t, pM, rm);
		getProjectionCovMat(K, R, t, pM, pCov, var, Const::PIXEL_ERR_VAR);
		mat22Inv(var, ivar);
		double err = mahaDist2(rm, nodes[i]->pt->m, ivar);
		if (err < errThres) { //inlier
			nodes[i]->pt->reprojErr = err;
			seqTriangulate(K, R, t, nodes[i]->pt->m, pM, pCov,
					Const::PIXEL_ERR_VAR);
			project(K, R, t, pM, rm);
			getProjectionCovMat(K, R, t, pM, pCov, var, Const::PIXEL_ERR_VAR);
			mat22Inv(var, ivar);
			err = mahaDist2(rm, nodes[i]->pt->m, ivar);
			//			if (err >= 1) {
			//				nodes[i]->pt->mpt->setUncertain();
			//				//test
			//				printf("poseUpdate:1\n");
			//			}
		} else {
			//outliers
			numOut++;
			double repErr = dist2(rm, nodes[i]->pt->m);
			nodes[i]->pt->reprojErr = repErr;
			nodes[i]->pt->mpt->setUncertain();
		}
	}
	CamPoseItem* camPos = m_camPos.add(currentFrame(), camId, R.data, t.data);
	updateCamParamForFeatPts(K, camPos);
//	if (currentFrame() > 9)
//		MyApp::bStop = true;
#ifdef DEBUG_MODE
//if the number of outliers are too much (may due to some distant points)
	if (currentFrame() >= 76) {
		//test
		printf("f:%d,cam:%d : n3d2d:%d, num:%d, numOut:%d\n", currentFrame(),
				camId, n3D2Ds, num, numOut);
		char dirPath[1024];
		sprintf(dirPath, "/home/tsou/slam_posefailed/%s", MyApp::timeStr);
		mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

		savePGM(m_img, "/home/tsou/slam_posefailed/%s/%d_img_%d.pgm",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(Ms, "/home/tsou/slam_posefailed/%s/%d_pts3d_%d.txt",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(ms, "/home/tsou/slam_posefailed/%s/%d_pts2d_%d.txt",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(covs, "/home/tsou/slam_posefailed/%s/%d_cov3d_%d.txt",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(3, 3, K, "/home/tsou/slam_posefailed/%s/%d_K_%d.txt",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(old_errs, "/home/tsou/slam_posefailed/%s/%d_old_errs_%d.txt",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(new_errs, "/home/tsou/slam_posefailed/%s/%d_new_errs_%d.txt",
				MyApp::timeStr, currentFrame(), camId);

		writeMat(3, 3, cR, "/home/tsou/slam_posefailed/%s/%d_oldR_%d.txt",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(3, 1, cT, "/home/tsou/slam_posefailed/%s/%d_oldT_%d.txt",
				MyApp::timeStr, currentFrame(), camId);
		writeMat(R, "/home/tsou/slam_posefailed/%s/%d_R_%d.txt", MyApp::timeStr,
				currentFrame(), camId);
		writeMat(t, "/home/tsou/slam_posefailed/%s/%dT_%d.txt", MyApp::timeStr,
				currentFrame(), camId);

		//test
		logInfo("program paused for debug at camera %d!\n", currentFrame(), camId);
		leaveBACriticalSection();
		CoSLAM::ptr->pause();
		enterBACriticalSection();
	}
#endif
	return num;
}
int SingleSLAM::poseUpdate3D2D() {
	propagateFeatureStates();

//get the feature points corresponding to the map points
	std::vector<Track2DNode*> nodes;
	int num = getStaticMappedTrackNodes(nodes);
	if (num < 5) {
		repErr(
				"[camera id:%d]intra-camera pose update failed! less than five static map points (%d)",
				camId, num);
		return -1;
	}

//get 2D-3D corresponding points
	Mat_d ms(num, 2), Ms(num, 3), repErrs(num, 1);
	for (int i = 0; i < num; i++) {
		ms.data[2 * i] = nodes[i]->pt->x;
		ms.data[2 * i + 1] = nodes[i]->pt->y;

		Ms.data[3 * i] = nodes[i]->pt->mpt->x;
		Ms.data[3 * i + 1] = nodes[i]->pt->mpt->y;
		Ms.data[3 * i + 2] = nodes[i]->pt->mpt->z;

		repErrs.data[i] = nodes[i]->pt->reprojErr;
	}

//get 2D-2D corresponding points
//TODO
	return 0;
}
int SingleSLAM::detectDynamicFeaturePoints(int maxLen, int minLen,
		int minOutNum, double maxEpiErr) {
	std::vector<Track2DNode*> nodes;
	getUnMappedAndDynamicTrackNodes(nodes, minLen);

	int k = 0;
	for (size_t i = 0; i < nodes.size(); i++) {
		FeaturePoint* fp = nodes[i]->pt;
		const double* R0 = fp->cam->R;
		const double* t0 = fp->cam->t;
		const double* m0 = fp->m;
		assert(fp->cam);

		int nOut = 0;
		double E[9], F[9];
		int f = 0;
		for (; fp && nOut <= minOutNum && f < maxLen; fp = fp->preFrame) {
			if (!fp->cam)
				continue;
			const double* R1 = fp->cam->R;
			const double* t1 = fp->cam->t;
			const double* m1 = fp->m;

			formEMat(R1, t1, R0, t0, E);
			getFMat(iK.data, iK.data, E, F);

			if (epipolarError(F, m0, m1) >= maxEpiErr) {
				nOut++;
			}
		}

		if (nOut > minOutNum) {
			nodes[i]->pt->type = TYPE_FEATPOINT_DYNAMIC;
			k++;
		} else if (!nodes[i]->pt->mpt) {
			nodes[i]->pt->type = TYPE_FEATPOINT_STATIC;
		}
	}
	return k;
}

double SingleSLAM::getCameraTranslationSelf() {
	CamPoseItem* oldPos = m_selfKeyPose.back()->cam;
	CamPoseItem* curPos = m_camPos.current();
	return getCamDist(oldPos, curPos);
}
double SingleSLAM::getViewAngleChangeSelf(const double center[3]) {
	CamPoseItem* oldPos = m_selfKeyPose.back()->cam;
	CamPoseItem* curPos = m_camPos.current();
	return getViewAngleChange(center, oldPos, curPos);
}
KeyPose* SingleSLAM::addKeyPose(bool bSelfMotion) {
	KeyPose* pose = 0;
	CamPoseItem* cam = m_camPos.current();

	pose = m_keyPose.add(currentFrame(), cam);
	pose->setNumMappedPoints(m_nMappedStaticPts);

	//test (for debugging group merge)
	//pose->setImage(m_img);

	pose->setSmallImage(m_smallImg, m_smallScale);
	pose->setCameraIntrinsic(K.data);
	pose->setFeatPoints(m_featPts.getFrameHead(currentFrame()),
			m_featPts.getFrameTail(currentFrame()));

	if (bSelfMotion) {
		pose->bSelfMotion = true;
		m_selfKeyPose.push_back(pose);
		m_prevKeyPos = m_lastKeyPos;
		m_lastKeyPos = pose;
	}

	for (FeaturePoint* fp = pose->pHead; fp != pose->pTail->next;
			fp = fp->next) {
		fp->bKeyFrm = true;
	}
	return pose;
}

int SingleSLAM::getCurFeatPts(std::vector<FeaturePoint*>& pFeatPoints) {
	int num = m_featPts.frameNum[currentFrame()];
	pFeatPoints.reserve(num);
	pFeatPoints.clear();

	FeaturePoint* pHead = m_featPts.getFrameHead(currentFrame());
	if (!pHead)
		return 0;
	FeaturePoint* pEnd = m_featPts.getFrameTail(currentFrame())->next;
	FeaturePoint* p = pHead;
	num = 0;
	while (p != pEnd) {
		pFeatPoints.push_back(p);
		p = p->next;
		num++;
	}
	return num;
}
//test
#include "gui/MyApp.h"

int SingleSLAM::getUnMappedAndTrackedCorrespondence(int f1, int f2,
		std::vector<Track2DNode*>& nodes1, std::vector<Track2DNode*>& nodes2) {
	if (f1 >= f2)
		repErr("tracks_get_corresponds() : f1 < f2 is required.");

	int cn = 0;
	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.f1 <= f1 && tk.f2 >= f2 && !tk.tail->pt->mpt) {
			Track2DNode* p = tk.head.next;
			bool bStatic = true;
			while (p && bStatic) {
				if (p->pt->type == TYPE_FEATPOINT_DYNAMIC) {
					bStatic = false;
					break;
				}
				p = p->next;
			}
			if (bStatic && tk.head.next) {
				Track2DNode* head = 0;
				Track2DNode* tail = 0;
				for (p = tk.head.next; p; p = p->next) {
					if (p->f == f1)
						head = p;
					else if (p->f == f2)
						tail = p;
				}
				if (head && tail) {
					nodes1.push_back(head);
					nodes2.push_back(tail);
					cn++;
				}
			}
		}
	}
	return cn;
}
int SingleSLAM::newMapPoints(std::vector<MapPoint*>& mapPts, double maxEpiErr,
		double maxNcc) {
	std::vector<FeaturePoint*> vecFeatPts;
	getUnMappedAndTrackedFeatPts(vecFeatPts, 0, Param::nMinFeatTrkLen);

	mapPts.clear();
	mapPts.reserve(4096);

	double M[3], m1[2], m2[2];

	//reconstruct 3D map points
	int numRecons = 0;
	for (size_t k = 0; k < vecFeatPts.size(); k++) {
		FeaturePoint* cur_fp = vecFeatPts[k];
		FeaturePoint* pre_fp = cur_fp;
		while (pre_fp->preFrame && pre_fp->preFrame->cam) {
			if (pre_fp->type == TYPE_FEATPOINT_DYNAMIC) {
				break;
			}
			pre_fp = pre_fp->preFrame;
		}
		if (pre_fp->type == TYPE_FEATPOINT_DYNAMIC || !pre_fp->cam)
			continue;

		normPoint(iK.data, pre_fp->m, m1);
		normPoint(iK.data, cur_fp->m, m2);

		//triangulate the two feature points to get the 3D point
		binTriangulate(pre_fp->cam->R, pre_fp->cam->t, cur_fp->cam->R,
				cur_fp->cam->t, m1, m2, M);

		if (isAtCameraBack(cur_fp->cam->R, cur_fp->cam->t, M))
			continue;

		double cov[9], org[3];
		getBinTriangulateCovMat(K.data, pre_fp->cam->R, pre_fp->cam->t, K.data,
				cur_fp->cam->R, cur_fp->cam->t, M, cov, Const::PIXEL_ERR_VAR);
		getCameraCenter(cur_fp->cam->R, cur_fp->cam->t, org);
		double s = fabs(cov[0] + cov[4] + cov[8]);
		if (dist3(org, M) < sqrt(s))
			continue;

		//check the reprojection error
		double err1 = reprojErrorSingle(K.data, pre_fp->cam->R, pre_fp->cam->t,
				M, pre_fp->m);
		double err2 = reprojErrorSingle(K.data, cur_fp->cam->R, cur_fp->cam->t,
				M, cur_fp->m);

		if (err1 < maxEpiErr && err2 < maxEpiErr) {
			//a new map point is generated
			refineTriangulation(cur_fp, M, cov);
			err1 = reprojErrorSingle(K.data, pre_fp->cam->R, pre_fp->cam->t, M,
					pre_fp->m);
			err2 = reprojErrorSingle(K.data, cur_fp->cam->R, cur_fp->cam->t, M,
					cur_fp->m);
			if (isAtCameraBack(cur_fp->cam->R, cur_fp->cam->t, M)
					|| isAtCameraBack(pre_fp->cam->R, pre_fp->cam->t, M))
				continue;
			if (err1 < maxEpiErr && err2 < maxEpiErr) {
				MapPoint* pM = new MapPoint(M[0], M[1], M[2], pre_fp->f);
				doubleArrCopy(pM->cov, 0, cov, 9);
				mapPts.push_back(pM);
				pM->lastFrame = cur_fp->f;
				for (FeaturePoint* p = cur_fp; p; p = p->preFrame)
					p->mpt = pM;

				//add the feature point
				pM->addFeature(camId, cur_fp);
				pM->setLocalStatic();

				//compute the NCC block
				pM->nccBlks[camId].computeScaled(m_lastKeyPos->imgSmall,
						m_lastKeyPos->imgScale, cur_fp->x, cur_fp->y);

				int x = max(0, min((int) cur_fp->x, m_rgb.w - 1));
				int y = max(0, min((int) cur_fp->y, m_rgb.h - 1));
				pM->setColor(m_rgb(x, y));
				numRecons++;
			}
		}
	}
	return numRecons;
}
void SingleSLAM::refineTriangulation(const FeaturePoint* fp, double M[3],
		double cov[9]) {
	double Ks[18], Rs[18], ts[6], ms[4], nms[4];

	double iK[9];
	getInvK(K.data, iK);

	const double* R0 = fp->cam->R;
	const double* t0 = fp->cam->t;

	doubleArrCopy(Ks, 0, K.data, 9);
	doubleArrCopy(Rs, 0, R0, 9);
	doubleArrCopy(ts, 0, t0, 3);
	doubleArrCopy(ms, 0, fp->m, 2);

	normPoint(iK, fp->m, nms);

	double C0[3];
	getCameraCenter(R0, t0, C0);

	const FeaturePoint* maxFp = 0;
	double maxAngle = 0;

	//search a camera pose that has the largest difference from the view direction (R0,t0)
	fp = fp->preFrame;
	while (fp && fp->cam) {
		double C[3];
		getCameraCenter(fp->cam->R, fp->cam->t, C);
		double angle = getAbsRadiansBetween(M, C0, C);
		if (angle > maxAngle) {
			maxAngle = angle;
			maxFp = fp;
		}
		fp = fp->preFrame;
	}

	if (maxFp) {
		doubleArrCopy(Ks, 1, K.data, 9);
		doubleArrCopy(Rs, 1, maxFp->cam->R, 9);
		doubleArrCopy(ts, 1, maxFp->cam->t, 3);
		doubleArrCopy(ms, 1, maxFp->m, 2);
		normPoint(iK, maxFp->m, nms + 2);
		triangulateMultiView(2, Rs, ts, nms, M);
		getTriangulateCovMat(2, Ks, Rs, ts, M, cov, Const::PIXEL_ERR_VAR);
	}
}
int SingleSLAM::regMapPoints(std::vector<MapPoint*>& mapPts,
		PtrVec<NCCBlock>& nccblks, double maxEpiErr, double maxNcc,
		double scale) {
	CamPoseItem* cam = m_camPos.current();
	assert(cam);

	Mat_d featPts;
	std::vector<FeaturePoint*> featPtsPtr;
	getUnMappedFeatPts(featPtsPtr, &featPts);

	int numMapPoints = mapPts.size();
	int nRegistered = 0;

	ImgG img;
	scaleDownAvg(m_img, img, scale);

	for (int i = 0; i < numMapPoints; i++) {
		double ms0[2], var[4];
		project(K.data, cam->R, cam->t, mapPts[i]->M, ms0);
		if (ms0[0] < 0 || ms0[0] >= videoReader->_w || ms0[1] < 0
				|| ms0[1] >= videoReader->_h)
			continue;

		getProjectionCovMat(K.data, cam->R, cam->t, mapPts[i]->M,
				mapPts[i]->cov, var, Const::PIXEL_ERR_VAR);
		int iNearest = searchNearestPoint(featPts, ms0[0], ms0[1], var,
				maxEpiErr, 0);
		if (iNearest < 0)
			continue;

		//compare the image blocks
		double fx = featPts.data[2 * iNearest];
		double fy = featPts.data[2 * iNearest + 1];

		NCCBlock nb;
		getNCCBlock(img, fx * scale, fy * scale, nb);

		double ncc = matchNCCBlock(&nb, nccblks[i]);
		if (ncc >= maxNcc) {
			//find a projection
			mapPts[i]->addFeature(camId, featPtsPtr[iNearest]);
			featPtsPtr[iNearest]->mpt = mapPts[i];
			nRegistered++;
		}
	}
	return nRegistered;
}

void SingleSLAM::removeFeatPts(int frame) {
	m_featPts.removeBefore(frame);
}
void SingleSLAM::removeKeyPoseImgs(int frame) {
	if (frame < 0)
		return;

	for (KeyPose* kp = m_keyPose.first(); kp && kp->frame < frame;
			kp = kp->next) {
		kp->img.clear();
		kp->imgSmall.clear();
	}
}

void featPoint2Mat(std::vector<FeaturePoint*>& pFeatPoints, Mat_d& matPts) {
	size_t num = pFeatPoints.size();
	matPts.resize(num, 2);
	for (size_t i = 0; i < num; i++) {
		matPts.data[2 * i] = pFeatPoints[i]->x;
		matPts.data[2 * i + 1] = pFeatPoints[i]->y;
	}
}
FeaturePoint* searchNearestFeatPt(FeaturePoints& featPts, int f, double m[2],
		double maxDist) {
	FeaturePoint* pHead = featPts.getFrameHead(f);
	if (!pHead)
		return 0;
	FeaturePoint* pEnd = featPts.getFrameTail(f)->next;
	FeaturePoint* p = pHead;
	double dMin = DBL_MAX;
	FeaturePoint* pMin = 0;
	while (p != pEnd) {
		double d = dist2(m, p->m);
		if (d < maxDist && d < dMin) {
			dMin = d;
			pMin = p;
		}
		p = p->next;
	}
	return pMin;
}

FeaturePoint* searchMahaNearestFeatPt(FeaturePoints& featPts, int f,
		double m[2], double var[4], double maxDist) {
	FeaturePoint* pHead = featPts.getFrameHead(f);
	if (!pHead)
		return 0;

	double ivar[4];
	mat22Inv(var, ivar);
	matScale(2, 2, ivar, 1 / maxDist, ivar);

	if (!featPts.getFrameTail(f))
		return 0;
	FeaturePoint* pEnd = featPts.getFrameTail(f)->next;
	FeaturePoint* p = pHead;
	double dMin = DBL_MAX;
	FeaturePoint* pMin = 0;
	while (p != pEnd) {
		double d = mahaDist2(m, p->m, ivar);
		if (d < dMin) {
			dMin = d;
			pMin = p;
		}
		p = p->next;
	}
	return pMin;
}

void SingleSLAM::feedExtraFeatPtsToTracker(
		std::vector<FeaturePoint*>& featPts) {
	m_tracker.feedExternFeatPoints(featPts);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//for debug
void SingleSLAM::saveCamPoses(const char* filePath, int startFrame) {
	FILE* fp = fopen(filePath, "w");
	if (!fp)
		repErr("SingleSLAM::saveCamPoses - cannot open '%s'!", filePath);

	for (CamPoseItem* cam = m_camPos.current(); cam && cam->f >= startFrame;
			cam = cam->pre) {
		fprintf(fp, "%d\t", cam->f);
		for (int i = 0; i < 9; i++)
			fprintf(fp, "%g ", cam->R[i]);
		fprintf(fp, "%g %g %g\n", cam->t[0], cam->t[1], cam->t[2]);
	}
	fclose(fp);
}
void SingleSLAM::saveFeatureTracks(const char* filePath, int minLen,
		int startFrame) {
	FILE* fp = fopen(filePath, "w");
	if (!fp)
		repErr("SingleSLAM::saveCamPoses -- cannot open '%s'!", filePath);

	for (int i = 0; i < m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = m_tracker.m_tks[i];
		if (tk.empty() || tk.length() < minLen)
			continue;

		if (tk.tail->pt->type == TYPE_FEATPOINT_DYNAMIC)
			continue;

		Track2DNode* node = tk.tail;
		MapPoint* mpt = node->pt->mpt;
		if (mpt && mpt->isCertainStatic())
			fprintf(fp, "1 %g %g %g\n", mpt->M[0], mpt->M[1], mpt->M[2]);
		else
			fprintf(fp, "0 0 0 0\n");
		//output the feature points

		std::vector<FeaturePoint*> tmpFeatPts;
		for (FeaturePoint* featPt = node->pt; featPt && featPt->f >= startFrame;
				featPt = featPt->preFrame) {
			tmpFeatPts.push_back(featPt);
		}

		for (std::vector<FeaturePoint*>::reverse_iterator iter =
				tmpFeatPts.rbegin(); iter != tmpFeatPts.rend(); iter++) {
			FeaturePoint* featPt = *iter;
			fprintf(fp, "%d %g %g ", featPt->f, featPt->x, featPt->y);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}
