/*
 * SL_NewMapPointsInterCam.h
 *
 *  Created on: 2011-1-23
 *      Author: Danping Zou
 */

#include "SL_NewMapPointsInterCam.h"
#include "SL_CoSLAM.h"
#include "SL_GlobParam.h"
#include "slam/SL_FeatureMatching.h"
#include "matching/SL_BlockDescriptorExtractor.h"
#include "matching/SL_StereoMatcher.h"
#include "matching/SL_StereoMatcherHelper.h"
#include "matching/SL_GuidedNCCMatcher.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Geometry.h"
#include "geometry/SL_Distortion.h"
#include "geometry/SL_5point.h"
#include "geometry/SL_Triangulate.h"

#include <cassert>

void NewMapPts::decidePointType() {
	using namespace std;
	assert(pCoSLAM);

	const int hw = 20;

	//generate dynamic masks
	int numCams = pCoSLAM->numCams;
	Mat_uc mask[SLAM_MAX_NUM];
	for (int c = 0; c < numCams; c++) {
		int camId = m_camGroup.camIds[c];
		mask[camId].resize(pCoSLAM->slam[camId].H, pCoSLAM->slam[camId].W);
		mask[camId].fill(0);
	}

	for (int c = 0; c < numCams; c++) {
		int camId = m_camGroup.camIds[c];
		vector<FeaturePoint*> vecDynMapPts;
		pCoSLAM->slam[camId].getMappedDynPts(vecDynMapPts);

		int W = mask[camId].cols;
		int H = mask[camId].rows;
		for (size_t i = 0; i < vecDynMapPts.size(); i++) {
			FeaturePoint* fp = vecDynMapPts[i];
			int x = static_cast<int>(fp->x + 0.5);
			int y = static_cast<int>(fp->y + 0.5);

			for (int s = -hw; s <= hw; s++) {
				for (int t = -hw; t <= hw; t++) {
					int x1 = x + t;
					int y1 = y + s;
					if (x1 < 0 || x1 >= W || y1 < 0 || y1 >= H)
						continue;
					mask[camId].data[y1 * W + x1] = 1;
				}
			}
		}
	}

	for (size_t i = 0; i < newMapPts.size(); i++) {
		MapPoint* mpt = newMapPts[i];
		if (mpt->isUncertain()) {
			bool isStatic = true;
			for (int c = 0; c < numCams; c++) {
				int camId = m_camGroup.camIds[c];
				int W = mask[camId].cols;
				int H = mask[camId].rows;
				FeaturePoint* fp = mpt->pFeatures[camId];
				if (fp) {
					int x = static_cast<int>(fp->x + 0.5);
					int y = static_cast<int>(fp->y + 0.5);

					if (x >= 0 && x < W && y >= 0 && y < H) {
						if (mask[camId][y * W + x] > 0) {
							isStatic = false;
							break;
						}
					}
				}
			}
			if (isStatic) {
				mpt->setLocalStatic();
			}
		}
	}

}

NewMapPtsNCC::NewMapPtsNCC() {
}

NewMapPtsNCC::~NewMapPtsNCC() {
}
int NewMapPtsNCC::getSeedsBetween(MapPointList& mapPts, int iCam, int jCam,
		Mat_d& seed1, Mat_d& seed2) {
	MapPoint* pHead = mapPts.getHead();
	if (!pHead)
		repErr("SLAM failed - No map point can be found!\n");
	std::vector<FeaturePoint*> vecFeatPts1, vecFeatPts2;
	for (MapPoint* p = pHead; p; p = p->next) {
		//if ((p->iLocalType == TYPE_MAPPOINT_STATIC || p->iLocalType == TYPE_MAPPOINT_DYNAMIC) && p->numVisCam >= 2 && p->pFeatures[iCam] && p->pFeatures[iCam]->f == m_curFrame && p->pFeatures[jCam] && p->pFeatures[jCam]->f == m_curFrame) {
		if (!p->isFalse() && !p->isUncertain() && p->numVisCam >= 2
				&& p->pFeatures[iCam] && p->pFeatures[iCam]->f == m_curFrame
				&& p->pFeatures[jCam] && p->pFeatures[jCam]->f == m_curFrame) {
			vecFeatPts1.push_back(p->pFeatures[iCam]);
			vecFeatPts2.push_back(p->pFeatures[jCam]);
		}
	}
	if (vecFeatPts1.empty())
		return 0;

	seed1.resize(vecFeatPts1.size(), 2);
	seed2.resize(vecFeatPts2.size(), 2);

	for (size_t i = 0; i < vecFeatPts1.size(); i++) {
		seed1.data[2 * i] = vecFeatPts1[i]->x;
		seed1.data[2 * i + 1] = vecFeatPts1[i]->y;
		seed2.data[2 * i] = vecFeatPts2[i]->x;
		seed2.data[2 * i + 1] = vecFeatPts2[i]->y;
	}
	return seed1.rows;
}
void NewMapPtsNCC::setInputs(const CameraGroup& camGroup, CoSLAM* coSLAM,
		int curFrame) {
	m_curFrame = curFrame;
	m_camGroup.copy(camGroup);

	numCams = 0;
	for (int i = 0; i < m_camGroup.num; i++) {
		int iCam = m_camGroup.camIds[i];
		addSlam(coSLAM->slam[iCam], m_curFrame);
	}

	pCoSLAM = coSLAM;
	MapPointList& curMapPts = pCoSLAM->curMapPts;

	for (int i = 0; i < numCams - 1; i++) {
		int cam1 = i;
		int cam2 = i + 1;
		int iCam = m_camGroup.camIds[i];
		int jCam = m_camGroup.camIds[i + 1];
		getSeedsBetween(curMapPts, iCam, jCam, m_seeds1[cam1 * numCams + cam2],
				m_seeds2[cam1 * numCams + cam2]);
	}
}
int NewMapPtsNCC::run() {
	newMapPts.clear();

	Matching matches[SLAM_MAX_NUM];
	for (int i = 0; i < numCams - 1; i++) {
		matchBetween(i, i + 1, matches[i]);
	}
	Track2D tks[SLAM_MAX_TRACKNUM];
	int ntks = featTracksFromMatches(numCams, pFeatPts, matches, tks);
	int npts = reconstructTracks(tks, ntks, m_curFrame, newMapPts, 2, 3.0);

	return npts;
}

void NewMapPtsNCC::output() {
	decidePointType();
	for (size_t i = 0; i < newMapPts.size(); i++) {
		MapPoint* mpt = newMapPts[i];
		for (int c = 0; c < numCams; c++) {
			int camId = m_camGroup.camIds[c];
			if (mpt->pFeatures[camId]) {
				for (FeaturePoint* fp = mpt->pFeatures[camId]; fp->nextFrame;
						fp = fp->nextFrame) {
					fp->mpt = mpt;
					mpt->lastFrame =
							fp->f > mpt->lastFrame ? fp->f : mpt->lastFrame;

					mpt->nccBlks[camId].computeScaled(
							pCoSLAM->slam[camId].m_smallImg,
							pCoSLAM->slam[camId].m_smallScale, fp->x, fp->y);

					int x = max(0, min((int) fp->x, pCoSLAM->slam[camId].m_img.w));
					int y = max(0, min((int) fp->y, pCoSLAM->slam[camId].m_img.h));

					mpt->setColor(pCoSLAM->slam[camId].m_rgb(x,y));
				}
			}
		}
		if (newMapPts[i]->lastFrame < pCoSLAM->curFrame) {
			pCoSLAM->actMapPts.add(mpt);
		} else
			pCoSLAM->curMapPts.add(mpt);
	}
}
int NewMapPtsNCC::reconstructTracks(Track2D tracks[], int numTracks,
		int curFrame, std::vector<MapPoint*>& mapPts, int minLen,
		double maxRpErr) {
	int num = 0;
	for (int k = 0; k < numTracks; k++) {
		if (tracks[k].length() >= minLen) {
			Mat_d ms(numCams, 2);
			Mat_d nms(numCams, 2);
			Mat_d Ks(numCams, 9);
			Mat_d Rs(numCams, 9);
			Mat_d Ts(numCams, 3);

			int npts = 0;
			for (Track2DNode* pTkNode = tracks[k].head.next; pTkNode; pTkNode =
					pTkNode->next) {
				int camId = pTkNode->f;
				ms.data[2 * npts] = pTkNode->x;
				ms.data[2 * npts + 1] = pTkNode->y;

				//normalize the image coordinates of the feature points
				normPoint(m_invK[camId], ms.data + 2 * npts,
						nms.data + 2 * npts);

				memcpy(Ks.data + 9 * npts, m_K[camId], sizeof(double) * 9);
				memcpy(Rs.data + 9 * npts, m_R[camId], sizeof(double) * 9);
				memcpy(Ts.data + 3 * npts, m_t[camId], sizeof(double) * 3);
				npts++;
			}

			double M[4];
			triangulateMultiView(npts, Rs.data, Ts.data, nms.data, M);
			bool outlier = false;
			//check re-projection error
			for (int i = 0; i < npts; i++) {
				double rm[2];
				project(Ks.data + 9 * i, Rs.data + 9 * i, Ts.data + 3 * i, M,
						rm);
				double err = dist2(ms.data + 2 * i, rm);
				if (err > maxRpErr
						|| isAtCameraBack(Rs.data + 9 * i, Ts.data + 3 * i, M)) {
					outlier = true;
					break;
				}
			}
			//if it is a inlier, a new map point is generated
			if (!outlier) {
				MapPoint* pM = new MapPoint(M[0], M[1], M[2], curFrame);
				mapPts.push_back(pM);
				//get the triangulation covariance
				getTriangulateCovMat(npts, Ks.data, Rs.data, Ts.data, M,
						pM->cov, Const::PIXEL_ERR_VAR);
				int nDyn = 0;
				for (Track2DNode* pTkNode = tracks[k].head.next; pTkNode;
						pTkNode = pTkNode->next) {
					int iCam = m_camGroup.camIds[pTkNode->f];
					FeaturePoint* fp = pTkNode->pt;
					fp->reprojErr = reprojErrorSingle(fp->K, fp->cam->R,
							fp->cam->t, M, fp->m);
					pM->addFeature(iCam, fp);
					if (fp->type == TYPE_FEATPOINT_DYNAMIC)
						nDyn++;
				}
				if (nDyn > 1) {
					pM->setLocalDynamic();
				}
//				else if (nDyn == 1)
//					pM->type = TYPE_MAPPOINT_UNCERTAIN;
//				else{
//					pM->type = TYPE_MAPPOINT_UNCERTAIN;
//					pM->newPt = true;
//				}
				else
					pM->setUncertain();
				num++;
			}
		}
	}
	return num;
}
int NewMapPtsNCC::matchBetween(int cam1, int cam2, Matching& matches) {
	Mat_d& corners1 = matFeatPts[cam1];
	Mat_d& corners2 = matFeatPts[cam2];

	ImgG& img1 = m_img[cam1];
	ImgG& img2 = m_img[cam2];

	PtrVec<NCCBlock> nccblks1, nccblks2;
	getNCCBlocks(img1, corners1, nccblks1, blockScale);
	getNCCBlocks(img2, corners2, nccblks2, blockScale);

	const double* R1 = m_R[cam1];
	const double* t1 = m_t[cam1];

	const double* R2 = m_R[cam2];
	const double* t2 = m_t[cam2];

	double E[9], F[9];
	formEMat(R2, t2, R1, t1, E);
	getFMat(m_invK[cam1], m_invK[cam2], E, F);

	Mat_d epiMat, nccMat;
	getEpiNccMat(F, corners1, corners2, nccblks1, nccblks2, maxEpiErr, minNcc,
			epiMat, nccMat);

	//	Mat_d seeds1, seeds2;
	//	int nSeeds = getSeedsBetween(mapPts, iCam, jCam, seeds1, seeds2);
	Mat_d& seeds1 = m_seeds1[cam1 * numCams + cam2];
	Mat_d& seeds2 = m_seeds2[cam1 * numCams + cam2];

	assert(seeds1.rows == seeds2.rows);

	int nSeeds = seeds1.rows;
	if (nSeeds > 0) {
		//use the guided feature matching method
		//1.compute disparity matrix
		Mat_d dispMat;
		getDisparityMat(corners1, corners2, seeds1, seeds2, maxDisp, dispMat);
		//2.matching
		greedyGuidedNCCMatch(nccMat, dispMat, matches);

	} else
		greedyNCCMatch(nccMat, matches);

	return matches.num;
}
#ifdef USE_GPUSURF
#include <gpusurf/GpuSurfDetector.hpp>
NewMapPtsSURF::~NewMapPtsSURF() {
	for (int c = 0; c < numCams; c++) {
		for (size_t i = 0; i < pFeatPts[c].size(); i++) {
			FeaturePoint* fp = pFeatPts[c][i];
			if (!fp->mpt)
			delete fp;
		}
	}
}
/**detect SURF feature points in each view*/
int NewMapPtsSURF::detectSURFPoints() {
	using namespace std;
	//1. generate mask
	Mat_uc mask[SLAM_MAX_NUM];
	for (int c = 0; c < numCams; c++) {
		int camId = m_camGroup.camIds[c];
		mask[camId].resize(pCoSLAM->slam[camId].H, pCoSLAM->slam[camId].W);
		mask[camId].fill(0);
	}

//	const int hw = 5;
//
//	for (int c = 0; c < numCams; c++) {
//		int camId = m_camGroup.camIds[c];
//		vector<FeaturePoint*> mappedFeatPts;
//		pCoSLAM->slam[camId].getMappedFeatPts(numCams, numCams, mappedFeatPts, 0);
//
//		int W = mask[camId].cols;
//		int H = mask[camId].rows;
//		for (size_t i = 0; i < mappedFeatPts.size(); i++) {
//			FeaturePoint* fp = mappedFeatPts[i];
//			int x = static_cast<int>(fp->x + 0.5);int
//			y = static_cast<int>(fp->y + 0.5);
//
//			for(int s = -hw; s <= hw; s++) {
//				for( int t = -hw; t <= hw; t++) {
//					int x1 = x+t;
//					int y1 = y+s;
//					if( x1 < 0 || x1 >= W || y1 < 0 || y1 >= H)
//					continue;
//					mask[camId].data[y1*W+x1] = 1;
//				}
//			}
//		}
//
//	}

	//2. detect the SURF feature points

	int npts = 0;
	for (int c = 0; c < numCams; c++) {
		asrl::GpuSurfConfiguration configuration;
		configuration.threshold = hessianThreshold;
		asrl::GpuSurfDetector detector(configuration);
		m_descDim = detector.descriptorSize();

		cv::Mat img(m_img[c].rows, m_img[c].cols, CV_8UC1, m_img[c].data);

		detector.buildIntegralImage(img);
		detector.detectKeypoints();
		detector.findOrientationFast();
		detector.computeDescriptors();

		vector<cv::KeyPoint> tmpKeyPts;
		vector<float> tmpDescs;
		detector.getKeypoints(tmpKeyPts);
		detector.getDescriptors(tmpDescs);

		int camId = m_camGroup.camIds[c];

		//remove the masked key points

		int W = mask[camId].cols;
		int H = mask[camId].rows;
		for (size_t i = 0; i < tmpKeyPts.size(); i++) {
			cv::KeyPoint kp = tmpKeyPts[i];
			int x = static_cast<int>(kp.pt.x + 0.5);
			int y = static_cast<int>(kp.pt.y + 0.5);

			if (x >= 0 && x < W && y >= 0 && y < H) {
				if (mask[camId][y * W + x] == 0) {
					surfPoints[camId].push_back(kp);
					for (int j = 0; j < m_descDim; j++)
					surfDescs[camId].push_back(tmpDescs[i * m_descDim + j]);
				}
			}
		}
		pFeatPts[c].clear();
		for (size_t i = 0; i < surfPoints[camId].size(); i++) {
			FeaturePoint* fp = new FeaturePoint(pCoSLAM->curFrame, camId, surfPoints[camId][i].pt.x, surfPoints[camId][i].pt.y);
			fp->setIntrinsic(pCoSLAM->slam[camId].K);
			fp->setCameraPose(pCoSLAM->slam[camId].m_camPos.current());
			pFeatPts[c].push_back(fp);
		}
		//test
		logInfo("num surfPts[%d]:%d\n", camId, surfPoints[camId].size());
		logInfo("num pFeatPts[%d]:%d\n", c, pFeatPts[c].size());
	}
	return npts;
}
/** match SURF feature points between two views*/
int NewMapPtsSURF::matchBetween(int iCam, int jCam, Matching& matches) {

	std::vector<float>& desc1 = surfDescs[iCam];
	std::vector<float>& desc2 = surfDescs[jCam];

	int dim = m_descDim;

	std::vector<cv::DMatch> matches1, matches2;

	cv::Mat matDesc1(desc1.size() / dim, dim, CV_32F, &desc1[0]);
	cv::Mat matDesc2(desc2.size() / dim, dim, CV_32F, &desc2[0]);
	cv::BruteForceMatcher<cv::L2<float> > matcher;
	matcher.match(matDesc1, matDesc2, matches1);
	matcher.match(matDesc2, matDesc1, matches2);

	//test
	cv::Mat img1(m_img[iCam].rows, m_img[iCam].cols, CV_8UC1, m_img[iCam].data);
	cv::Mat img2(m_img[jCam].rows, m_img[jCam].cols, CV_8UC1, m_img[jCam].data);

	cv::Mat outImg;
	cv::drawMatches(img1, surfPoints[iCam], img2, surfPoints[jCam], matches1, outImg);
	cv::imwrite("/home/tsou/test.bmp", outImg);

	//cross validation
	std::vector<int> ind1;
	ind1.assign(surfPoints[iCam].size(), -1);

	for (size_t j = 0; j < matches1.size(); j++) {
		int idx1 = matches1[j].queryIdx;
		int idx2 = matches1[j].trainIdx;
		ind1[idx1] = idx2;
	}

	matches.clear();
	matches.reserve(matches2.size());
	for (size_t j = 0; j < matches2.size(); j++) {
		int idx2 = matches2[j].queryIdx;
		int idx1 = matches2[j].trainIdx;
		if (ind1[idx1] == idx2) {
			matches.add(idx1, idx2, matches2[j].distance);
		}
	}
	return matches.num;
}

void NewMapPtsSURF::setInputs(const CameraGroup& camGroup, CoSLAM* coSLAM, int curFrame) {
	m_curFrame = curFrame;
	m_camGroup.copy(camGroup);
	numCams = 0;
	for (int i = 0; i < m_camGroup.num; i++) {
		int iCam = m_camGroup.camIds[i];
		addSlam(coSLAM->slam[iCam], m_curFrame);
	}
	pCoSLAM = coSLAM;
}
int NewMapPtsSURF::reconstructTracks(Track2D tracks[], int numTracks, int curFrame, std::vector<MapPoint*>& mapPts, //new map points that are generated
		int minLen,
		double maxRpErr) {

	std::vector<FeaturePoint*> reconFeatPts[SLAM_MAX_NUM];
	int num = 0;
	for (int k = 0; k < numTracks; k++) {
		if (tracks[k].length() >= minLen) {
			Mat_d ms(numCams, 2);
			Mat_d nms(numCams, 2);
			Mat_d Ks(numCams, 9);
			Mat_d Rs(numCams, 9);
			Mat_d Ts(numCams, 3);

			int npts = 0;
			for (Track2DNode* pTkNode = tracks[k].head.next; pTkNode; pTkNode = pTkNode->next) {
				int camId = m_camGroup.camIds[pTkNode->f];
				ms.data[2 * npts] = pTkNode->x;
				ms.data[2 * npts + 1] = pTkNode->y;

				//normalize the image coordinates of the feature points
				normPoint(m_invK[camId], ms.data + 2 * npts, nms.data + 2 * npts);

				memcpy(Ks.data + 9 * npts, m_K[camId], sizeof(double) * 9);
				memcpy(Rs.data + 9 * npts, m_R[camId], sizeof(double) * 9);
				memcpy(Ts.data + 3 * npts, m_t[camId], sizeof(double) * 3);
				npts++;
			}

			double M[4];
			triangulateMultiView(npts, Rs.data, Ts.data, nms.data, M);
			bool outlier = false;
			//check re-projection error
			for (int i = 0; i < npts; i++) {
				double rm[2];
				project(Ks.data + 9 * i, Rs.data + 9 * i, Ts.data + 3 * i, M, rm);
				double err = dist2(ms.data + 2 * i, rm);
				if (err > maxRpErr || isAtCameraBack(Rs.data + 9 * i, Ts.data + 3 * i, M)) {
					outlier = true;
					break;
				}
			}
			//if it is a inlier, a new map point is generated
			if (!outlier) {
				MapPoint* pM = new MapPoint(M[0], M[1], M[2], curFrame);
				mapPts.push_back(pM);
				//get the triangulation covariance
				getTriangulateCovMat(npts, Ks.data, Rs.data, Ts.data, M, pM->cov, Const::PIXEL_ERR_VAR);
				for (Track2DNode* pTkNode = tracks[k].head.next; pTkNode; pTkNode = pTkNode->next) {
					int camId = m_camGroup.camIds[pTkNode->f];
					FeaturePoint* fp = pTkNode->pt;
					reconFeatPts[pTkNode->f].push_back(fp);
					fp->reprojErr = reprojErrorSingle(fp->K, fp->cam->R, fp->cam->t, M, fp->m);
					pM->addFeature(camId, fp);
				}
				pM->setUncertain();
				num++;
			}
		}
	}

	//test
	logInfo("%d new map points are generated!\n", num);

	for (int c = 0; c < numCams; c++) {
		int camId = m_camGroup.camIds[c];
		pCoSLAM->slam[camId].feedExtraFeatPtsToTracker(reconFeatPts[camId]);
	}
	return num;

}
int NewMapPtsSURF::run() {
	newMapPts.clear();
	detectSURFPoints();

	Matching matches[SLAM_MAX_NUM];
	for (int i = 0; i < numCams - 1; i++) {
		int camId1 = m_camGroup.camIds[i];
		int camId2 = m_camGroup.camIds[i + 1];
		matchBetween(camId1, camId2, matches[i]);
		//test
		printf("%d-%d : %d\n", i, i + 1, matches[i].num);
	}

	Track2D tks[SLAM_MAX_TRACKNUM];
	int ntks = featTracksFromMatches(numCams, pFeatPts, matches, tks);
	int npts = reconstructTracks(tks, ntks, m_curFrame, newMapPts, 3, 30.0);

	return npts;
}
void NewMapPtsSURF::output() {
	decidePointType();
	for (size_t i = 0; i < newMapPts.size(); i++) {
		MapPoint* mpt = newMapPts[i];
		for (int j = 0; j < numCams; j++) {
			int camId = m_camGroup.camIds[j];
			if (mpt->pFeatures[camId]) {
				for (FeaturePoint* fp = mpt->pFeatures[camId]; fp->nextFrame; fp = fp->nextFrame) {
					fp->mpt = mpt;
					mpt->lastFrame = fp->f > mpt->lastFrame ? fp->f : mpt->lastFrame;
				}
				pCoSLAM->slam[camId].m_featPts.add(mpt->pFeatures[camId]);
			}
		}
		if (newMapPts[i]->lastFrame < pCoSLAM->curFrame) {
			pCoSLAM->actMapPts.add(mpt);
		} else
		pCoSLAM->curMapPts.add(mpt);
	}
}
#endif
void getValidRowsCols(const Mat_d& distMat, double invalidVal, Mat_c& rowFlag,
		Mat_c& colFlag) {
	int m = distMat.rows;
	int n = distMat.cols;

	rowFlag.resize(m, 1);
	colFlag.resize(n, 1);

	for (int i = 0; i < m; i++) {
		bool allNone = true;
		for (int j = 0; j < n; j++) {
			if (distMat.data[i * n + j] != invalidVal) {
				allNone = false;
				break;
			}
		}
		rowFlag.data[i] = allNone ? 0 : 1;
	}

	for (int j = 0; j < n; j++) {
		bool allNone = true;
		for (int i = 0; i < m; i++) {
			if (distMat.data[i * n + j] != invalidVal) {
				allNone = false;
				break;
			}
		}
		colFlag.data[j] = allNone ? 0 : 1;
	}
}

void match2ind(int maxNum, const Matching& matches, Mat_i& indices) {
	assert(maxNum > 0);

	indices.resize(maxNum, 1);
	indices.fill(-1);
	for (int i = 0; i < matches.num; i++) {
		int idx1 = matches[i].idx1;
		int idx2 = matches[i].idx2;

		indices.data[idx1] = idx2;
	}
}
int featTracksFromMatches(int numCams, std::vector<FeaturePoint*> pFeatPts[],
		Matching matches[], Track2D tks[]) {
	Mat_i matchingIdx[SLAM_MAX_NUM];
	Mat_i flag[SLAM_MAX_NUM];

	for (int i = 1; i < numCams; i++) {
		match2ind((int) pFeatPts[i - 1].size(), matches[i - 1],
				matchingIdx[i - 1]);
	}

	//flag of a feature point records the corresponding track id 
	for (int i = 0; i < numCams; i++) {
		flag[i].resize(pFeatPts[i].size(), 1);
		flag[i].fill(-1);
	}

	//find tracks
	int nTracks = 0;
	for (int i = 0; i < numCams - 1; i++) {
		int iCam = i;
		int jCam = i + 1;

		size_t npts = pFeatPts[iCam].size();
		if (i == 0) {
			for (size_t n = 0; n < npts; n++) {
				//generate initial tracks
				int m = matchingIdx[i].data[n];
				if (m >= 0) {
					tks[nTracks].add(i, pFeatPts[iCam][n]);
					tks[nTracks].add(i + 1, pFeatPts[jCam][m]);

					flag[iCam].data[n] = nTracks;
					flag[jCam].data[m] = nTracks;
					nTracks++;
				}
			}
		} else {
			for (size_t n = 0; n < npts; n++) {
				int trackId = flag[iCam].data[n];
				if (trackId >= 0) { //a existing track
					int m = matchingIdx[i].data[n];
					if (m >= 0) {
						//extend the track
						tks[trackId].add(i + 1, pFeatPts[jCam][m]);
						flag[jCam].data[m] = trackId;
					}
				} else {
					//a new track
					int m = matchingIdx[i].data[n];
					if (m >= 0) {
						tks[nTracks].add(i, pFeatPts[iCam][n]);
						tks[nTracks].add(i + 1, pFeatPts[jCam][m]);

						flag[iCam].data[n] = nTracks;
						flag[jCam].data[m] = nTracks;
						nTracks++;
					}
				}
			}
		}
	}
	return nTracks;
}

