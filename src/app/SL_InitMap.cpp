/*
 * MapInitializer.cpp
 *
 *  Created on: 2011-1-4
 *      Author: Danping Zou
 */

#include "SL_InitMap.h"
#include "tools/SL_Debug.h"
#include "matching/SL_StereoMatcher.h"
#include "matching/SL_GuidedSSDMatcher.h"
#include "matching/SL_GuidedNCCMatcher.h"
#include "matching/SL_BlockDescriptorExtractor.h"
#include "matching/SL_StereoMatcherHelper.h"
#include "matching/SL_SurfMatching.h"

#include "geometry/SL_5point.h"
#include "geometry/SL_EMatWrapper.h"
#include "imgproc/SL_ImageOp.h"
#include "imgproc/SL_ImageIO.h"

#include "slam/SL_Define.h"
#include "slam/SL_InitMapHelper.h"
#include "slam/SL_FeatureMatching.h"
#include "SL_CoSLAMBA.h"
#include "SL_GlobParam.h"
#if CV_MAJOR_VERSION <= 2
#if CV_MINOR_VERSION > 3
#include <opencv2/nonfree/features2d.hpp>
#endif
#elif(CV_MAJOR_VERSION == 3)
#include <opencv2/xfeatures2d.hpp>
#endif

//#define DEBUG_MODE
InitMap::InitMap() :
		m_numSurfTracks(0), m_numCornerTracks(0), numCams(0) {
}

InitMap::~InitMap() {

}
void InitMap::detectSurfFeats(int iCam) {
	const ImgG* img = m_pImgGray[iCam];
	KpVec& keyPts = m_cvSurfPts[iCam];
	std::vector<float> desc;
    
    cv::Mat cvImg(img->rows, img->cols, CV_8UC1, img->data);
#if CV_MAJOR_VERSION <= 2
#if CV_MINOR_VERSION > 3
	cv::SurfFeatureDetector surf(400, 4, 2, false);
	surf(cvImg, cv::Mat(), keyPts, desc);
	int dimDesc = surf.descriptorSize();
#else
	cv::SURF surf(400, 4, 2, false);
	surf(cvImg, cv::Mat(), keyPts, desc);
	int dimDesc = surf.descriptorSize();
#endif
#elif CV_MAJOR_VERSION == 3
    using namespace cv::xfeatures2d;
    cv::Ptr<SURF> surf = SURF::create(400, 4, 2, false);
    surf->detectAndCompute(cvImg,cv::Mat(), keyPts, desc);
    int dimDesc = surf->descriptorSize();    
#endif
	m_surfDesc[iCam].resize(keyPts.size(), dimDesc);

	Mat_d& pts = m_surfPts[iCam];
	Mat_d& ptsNorm = m_normSurfPts[iCam];

	int num = keyPts.size();

	pts.resize(num, 2);
	ptsNorm.resize(num, 2);

	//sort the surf points
	std::vector<std::pair<double, int> > items;
	for (int i = 0; i < num; i++) {
		items.push_back(std::make_pair(keyPts[i].pt.x + keyPts[i].pt.y, i));
	}
	std::sort(items.begin(), items.end());

	for (int i = 0; i < num; i++) {
		int ind = items[i].second;
		pts.data[2 * i] = keyPts[ind].pt.x;
		pts.data[2 * i + 1] = keyPts[ind].pt.y;

		memcpy(m_surfDesc[iCam].data + dimDesc * i, &desc[0] + dimDesc * ind,
				dimDesc * sizeof(float));
	}

	//remove the distortion
	undistorNormPoints(invK[iCam], invkd[iCam], num, pts, ptsNorm);
	imagePoints(K[iCam], num, ptsNorm, pts);
}

void InitMap::detectCorners(int iCam) {
	const ImgG* img = m_pImgGray[iCam];
	//	KLT corner detector
	Mat_f corners;
	int IW = img->w;
	int IH = img->h;
	GPUKLT klt;
	klt.setIntrinsicParam(K[iCam], invK[iCam], invkd[iCam]);
	klt.detectCorners(IW, IH, img->data, corners, 500.0f, 5);

	//compute the normalized points
	Mat_d& pts = m_corners[iCam];
	Mat_d& ptsNorm = m_normCorners[iCam];
	int num = corners.rows;

	pts.resize(num, 2);
	ptsNorm.resize(num, 2);

	for (int i = 0; i < num; i++) {
		pts.data[2 * i] = corners.data[2 * i];
		pts.data[2 * i + 1] = corners.data[2 * i + 1];
	}

	undistorNormPoints(invK[iCam], invkd[iCam], num, pts, ptsNorm);
	imagePoints(K[iCam], num, ptsNorm, pts);

	m_cornerFlag[iCam].resize(num, 1);
	m_cornerFlag[iCam].fill(-1);
}
#include "calibration/SL_CalibTwoCam.h"
int InitMap::estimateFmat(const Mat_d& pts1, const Mat_d& pts2, Mat_d& F,
		Mat_uc& inlierFlag) {
	if (!findFMatrix(pts1, pts2, F, epiErrMax, FF_RANSAC))
		return -1;   

	int nInlier = getInlierFMatches(pts1, pts2, F, epiErrMax, inlierFlag);
	return nInlier;
}

void InitMap::addCam(const SingleSLAM& cam) {
	addCam(&cam.m_rgb, &cam.m_img, &cam.m_smallImg, cam.m_smallScale,
			cam.K.data, cam.k_c.data, cam.iK.data, cam.k_ud.data);
}
void InitMap::addCam(const ImgRGB* rgb, const ImgG* iG, const ImgG* iSG,
		const double imgScale, const double* cK, const double* ckd,
		const double* ciK, const double* cikd) {
	m_pImgRGB[numCams] = rgb;
	m_pImgGray[numCams] = iG;
	m_pImgGraySmall[numCams] = iSG;
	m_imgScale[numCams] = imgScale;
	memcpy(K[numCams], cK, sizeof(double) * 9);
	memcpy(kd[numCams], ckd, sizeof(double) * 5);
	memcpy(invK[numCams], ciK, sizeof(double) * 9);
	memcpy(invkd[numCams], cikd, sizeof(double) * 7);
	numCams++;
}

void InitMap::apply(int frame0, std::vector<FeaturePoints*>& pFeaturePts,
		MapPointList& mapPts) {
	if (numCams < 2)
		repErr("StereoMatch - at least two cameras are required!\n");
	//detect surf features at all cameras
	for (int i = 0; i < numCams; i++) {
		detectSurfFeats(i);
		detectCorners(i);
	}
	for (int i = 0; i < numCams; i++) {
		for (int j = i + 1; j < numCams; j++) {
			matchSurfBetween(i, j);
			matchCornerNCCBetween(i, j, 0.6, 7.0, 60);
		}
	}
    
	//select camera orders
	selectImageOrder();
	getSurfTracks();
	getCornerTracks();

	//compute extrinsic parameters for all cameras
	computeExtrinsic();
	//reconstruct the initial map points
	int nSurfPts = reconstructTracks(m_surfTracks, m_numSurfTracks, 2, frame0,
			pFeaturePts, mapPts, 30.0);

	logInfo("number of SURF feature points reconstructed : %d\n", nSurfPts);

	if (nSurfPts <= SLAM_FEATURE_WIDTH * SLAM_FEATURE_HEIGHT / 2) {
		int nCorners = reconstructTracks(m_cornerTracks, m_numCornerTracks, 2,
				frame0, pFeaturePts, mapPts, 30.0);
		logInfo("number of corners reconstructed : %d\n", nCorners);
	}
	logInfo("removed outliers:%d\n", removeOutliers(mapPts, 12.0));
}

void InitMap::drawResult(const ImgG* img1, const ImgG* img2,
		const KpVec& keyPts1, const KpVec& keyPts2, const DMatchVec& matches,
		const Mat_uc& flag) {
	cv::Mat tmpImg;
	cv::Mat matImg1(img1->rows, img1->cols, CV_8UC1, img1->data);
	cv::Mat matImg2(img2->rows, img2->cols, CV_8UC1, img2->data);
	//cv::drawMatchesScale(matImg1, keyPts1, matImg2, keyPts2, matches, tmpImg);
	drawMatching(matImg1, keyPts1, matImg2, keyPts2, matches, tmpImg, 1,
			flag.data);
	cv::imshow("matches", tmpImg);
	//	drawKeyPoints(matImg1, keyPts1, tmpImg, 1. / coarseScale);
	//	cv::imshow("img0", tmpImg);
	//	drawKeyPoints(matImg2, keyPts2, tmpImg, 1. / coarseScale);
	//	cv::imshow("img1", tmpImg);
	cv::waitKey(-1);
}
void InitMap::drawResult(const ImgG* img1, const ImgG* img2,
		const KpVec& keyPts1, const KpVec& keyPts2, const DMatchVec& matches) {

	cv::Mat tmpImg;
	cv::Mat matImg1(img1->rows, img1->cols, CV_8UC1, img1->data);
	cv::Mat matImg2(img2->rows, img2->cols, CV_8UC1, img2->data);
	//cv::drawMatchesScale(matImg1, keyPts1, matImg2, keyPts2, matches, tmpImg);
	drawMatching(matImg1, keyPts1, matImg2, keyPts2, matches, tmpImg, 1, 0);
	cv::imshow("matches", tmpImg);
	//	drawKeyPoints(matImg1, keyPts1, tmpImg, 1. / coarseScale);
	//	cv::imshow("img0", tmpImg);
	//	drawKeyPoints(matImg2, keyPts2, tmpImg, 1. / coarseScale);
	//	cv::imshow("img1", tmpImg);
	cv::waitKey(-1);
}

void InitMap::_refineSurfMatches(int iCam, int jCam,
		const Matching& surfMatchesOrg, Matching& surfMatchesRefined) {
	const ImgG& img1 = *m_pImgGray[iCam];
	const ImgG& img2 = *m_pImgGray[jCam];

	Mat_d& pts1 = m_surfPts[iCam];
	Mat_d& pts2 = m_surfPts[jCam];

	const double ssdThres = 30;
	const int HW = 5;
	const int len = (2 * HW + 1) * (2 * HW + 1);
	Mat_d block1(2 * HW + 1, 2 * HW + 1);
	Mat_d block2(2 * HW + 1, 2 * HW + 1);

	surfMatchesRefined.clear();
	surfMatchesRefined.reserve(surfMatchesOrg.num);
	for (int i = 0; i < surfMatchesOrg.num; i++) {
		int idx1 = surfMatchesOrg[i].idx1;
		int idx2 = surfMatchesOrg[i].idx2;

		int x1 = (int) (pts1.data[2 * idx1] + 0.5);
		int y1 = (int) (pts1.data[2 * idx1 + 1] + 0.5);

		int x2 = (int) (pts2.data[2 * idx2] + 0.5);
		int y2 = (int) (pts2.data[2 * idx2 + 1] + 0.5);

		getImgBlock(x1, y1, HW, img1, block1.data);
		getImgBlock(x2, y2, HW, img2, block2.data);

		double ssd = computeDescDist(len, block1.data, block2.data);
		if (ssd < ssdThres) {
			surfMatchesRefined.add(idx1, idx2, surfMatchesOrg[i].dist);
		}
	}
}
void InitMap::matchSurfBetween(int iCam, int jCam) {
	Matching surfMatches;
	matchSurf(m_surfDesc[iCam], m_surfDesc[jCam], surfMatches, 0.83);

	Mat_d pts1, pts2;
	getMatchedPts(surfMatches, m_surfPts[iCam], m_surfPts[jCam], pts1, pts2);

	Mat_d matF;
	Mat_uc inlierFlag;
	if (estimateFmat(pts1, pts2, matF, inlierFlag) < 0) {
		logInfo("no F matrix can be estimated between (%d -%d)\n", iCam, jCam);
		return;
	}

	F[iCam][jCam].cloneFrom(matF);
	//---------------------keep inlier matches -----------------------------------//
	int numMatch = surfMatches.num;
	assert(numMatch == inlierFlag.m);
	m_surfMatches[iCam][jCam].clear();
	m_surfMatches[iCam][jCam].reserve(numMatch);
	for (int i = 0; i < numMatch; i++) {
		if (inlierFlag.data[i] > 0) {
			m_surfMatches[iCam][jCam].add(surfMatches[i].idx1,
					surfMatches[i].idx2, surfMatches[i].dist);
		}
	}
    
    //test
    cout << endl;
    cout << "detected surf points:" << iCam << " - " << m_surfDesc[iCam].rows << endl;
    cout << "detected surf points:" << jCam << " - " << m_surfDesc[jCam].rows << endl;
    cout << "matched surf points" << iCam << " - " << jCam << " : " << m_surfMatches[iCam][jCam].num << endl;
}

void InitMap::matchCornerNCCBetween(int iCam, int jCam, double minNcc,
		double maxEpi, double maxDisp) {
	PtrVec<NCCBlock> nccBlks1, nccBlks2;

	getNCCBlocks(*m_pImgGray[iCam], m_corners[iCam], nccBlks1, 0.3);
	getNCCBlocks(*m_pImgGray[jCam], m_corners[jCam], nccBlks2, 0.3);

	//get the epipolar error and NCC matching score matrices
	Mat_d epiMat, nccMat;
	getEpiNccMat(F[iCam][jCam], m_corners[iCam], m_corners[jCam], nccBlks1,
			nccBlks2, maxEpi, minNcc, epiMat, nccMat);

	//get seed points
	Mat_d seedPts1, seedPts2;
	getMatchedPts(m_surfMatches[iCam][jCam], m_surfPts[iCam], m_surfPts[jCam],
			seedPts1, seedPts2);

	//get the disparity matrix
	Mat_d dispMat;
	getDisparityMat(m_corners[iCam], m_corners[jCam], seedPts1, seedPts2,
			maxDisp, dispMat);

	Matching& matches = m_cornerMatches[iCam][jCam];
	matches.clear();
	greedyGuidedNCCMatch(nccMat, dispMat, matches);
}

void InitMap::selectImageOrder() {
	Mat_i camMat(numCams, numCams);
	camMat.fill(-1);
	for (int i = 0; i < numCams; i++) {
		for (int j = i; j < numCams; j++) {
			camMat(i, j) = m_surfMatches[i][j].num;
			camMat(j, i) = m_surfMatches[i][j].num;
		}
	}
	//test
	print(camMat);
	selectCameraOrder(numCams, camMat, camOrder);
	//test
	print(camOrder);
}
void InitMap::_surfMatch2Ind(int iCam, int jCam, int num, Mat_i& indices) {
	const Matching& matches =
			iCam < jCam ? m_surfMatches[iCam][jCam] : m_surfMatches[jCam][iCam];

	indices.resize(num, 1);
	indices.fill(-1);

	if (iCam < jCam) {
		for (int i = 0; i < matches.num; i++) {
			assert(matches[i].idx1 >= 0 && matches[i].idx2 >= 0);
			indices[matches[i].idx1] = matches[i].idx2;
		}
	} else {
		for (int i = 0; i < matches.num; i++) {
			assert(matches[i].idx1 >= 0 && matches[i].idx2 >= 0);
			indices[matches[i].idx2] = matches[i].idx1;
		}
	}
}
void InitMap::_cornerMatch2Ind(int iCam, int jCam, int num, Mat_i& indices) {
	const Matching& matches =
			iCam < jCam ?
					m_cornerMatches[iCam][jCam] : m_cornerMatches[jCam][iCam];

	indices.resize(num, 1);
	indices.fill(-1);

	if (iCam < jCam) {
		for (int i = 0; i < matches.num; i++) {
			assert(matches[i].idx1 >= 0 && matches[i].idx2 >= 0);
			indices[matches[i].idx1] = matches[i].idx2;
		}
	} else {
		for (int i = 0; i < matches.num; i++) {
			assert(matches[i].idx1 >= 0 && matches[i].idx2 >= 0);
			indices[matches[i].idx2] = matches[i].idx1;
		}
	}
}
void InitMap::getSurfTracks() {
	if (numCams < 2)
		repErr(
				"MapInitalizer::getCornerTracks - at least two cameras are required!\n");

	Mat_i matchingIdx[SLAM_MAX_NUM];
	Mat_i flag[SLAM_MAX_NUM];

	for (int i = 1; i < numCams; i++) {
		int iCam = camOrder.data[i - 1];
		int jCam = camOrder.data[i];
		_surfMatch2Ind(iCam, jCam, m_surfPts[iCam].rows, matchingIdx[i - 1]);
	}

	//the flag records the track id corresponding to this feature point
	for (int i = 0; i < numCams; i++) {
		int iCam = camOrder.data[i];
		flag[iCam].resize(m_surfPts[iCam].rows, 1);
		flag[iCam].fill(-1);
	}

	//find tracks
	for (int i = 0; i < numCams - 1; i++) {
		int iCam = camOrder.data[i];
		int jCam = camOrder.data[i + 1];

		int numPts = m_surfPts[iCam].rows;
		if (i == 0) {
			for (int n = 0; n < numPts; n++) {
				int m = matchingIdx[i].data[n];
				if (m >= 0) {
					double x1 = m_surfPts[iCam].data[2 * n];
					double y1 = m_surfPts[iCam].data[2 * n + 1];
					double x2 = m_surfPts[jCam].data[2 * m];
					double y2 = m_surfPts[jCam].data[2 * m + 1];

					m_surfTracks[m_numSurfTracks].add(i, x1, y1);
					m_surfTracks[m_numSurfTracks].add(i + 1, x2, y2);

					flag[iCam].data[n] = m_numSurfTracks;
					flag[jCam].data[m] = m_numSurfTracks;

					m_numSurfTracks++;
				}
			}
		} else {
			for (int n = 0; n < numPts; n++) {
				if (flag[iCam].data[n] >= 0) { //existing track
					int m = matchingIdx[i].data[n];
					if (m >= 0) { //extend the track
						double x2 = m_surfPts[jCam].data[2 * m];
						double y2 = m_surfPts[jCam].data[2 * m + 1];
						m_surfTracks[flag[iCam].data[n]].add(i + 1, x2, y2);
						flag[jCam].data[m] = flag[iCam].data[n];
					}

				} else {
					//a new track
					int m = matchingIdx[i].data[n];
					if (m >= 0) {
						double x1 = m_surfPts[iCam].data[2 * n];
						double y1 = m_surfPts[iCam].data[2 * n + 1];
						double x2 = m_surfPts[jCam].data[2 * m];
						double y2 = m_surfPts[jCam].data[2 * m + 1];
						m_surfTracks[m_numSurfTracks].add(i, x1, y1);
						m_surfTracks[m_numSurfTracks].add(i + 1, x2, y2);

						flag[iCam].data[n] = m_numSurfTracks;
						flag[jCam].data[m] = m_numSurfTracks;

						m_numSurfTracks++;
					}
				}
			}
		}
	}

	//	//test
	//	for (int i = 0; i < m_numSurfTracks; i++) {
	//		print(m_surfTracks[i]);
	//	}
}
void InitMap::getCornerTracks() {
	if (numCams < 2)
		repErr(
				"MapInitalizer::getCornerTracks - at least two cameras are required!\n");

	Mat_i matchingIdx[SLAM_MAX_NUM];
	Mat_i flag[SLAM_MAX_NUM];

	for (int i = 1; i < numCams; i++) {
		int iCam = camOrder.data[i - 1];
		int jCam = camOrder.data[i];
		_cornerMatch2Ind(iCam, jCam, m_corners[iCam].rows, matchingIdx[i - 1]);
	}

	//the flag records the track id corresponding to this feature point
	for (int i = 0; i < numCams; i++) {
		int iCam = camOrder.data[i];
		flag[iCam].resize(m_corners[iCam].rows, 1);
		flag[iCam].fill(-1);
	}

	//find tracks
	for (int i = 0; i < numCams - 1; i++) {
		int iCam = camOrder.data[i];
		int jCam = camOrder.data[i + 1];

		int numPts = m_corners[iCam].rows;
		if (i == 0) {
			for (int n = 0; n < numPts; n++) {
				int m = matchingIdx[i].data[n];
				if (m >= 0) {
					double x1 = m_corners[iCam].data[2 * n];
					double y1 = m_corners[iCam].data[2 * n + 1];
					double x2 = m_corners[jCam].data[2 * m];
					double y2 = m_corners[jCam].data[2 * m + 1];

					m_cornerTracks[m_numCornerTracks].add(i, x1, y1);
					m_cornerTracks[m_numCornerTracks].add(i + 1, x2, y2);

					flag[iCam].data[n] = m_numCornerTracks;
					flag[jCam].data[m] = m_numCornerTracks;

					m_numCornerTracks++;
				}
			}
		} else {
			for (int n = 0; n < numPts; n++) {
				if (flag[iCam].data[n] >= 0) { //existing track
					int m = matchingIdx[i].data[n];
					if (m >= 0) { //extend the track
						double x2 = m_corners[jCam].data[2 * m];
						double y2 = m_corners[jCam].data[2 * m + 1];
						m_cornerTracks[flag[iCam].data[n]].add(i + 1, x2, y2);
						flag[jCam].data[m] = flag[iCam].data[n];
					}

				} else {
					//a new track
					int m = matchingIdx[i].data[n];
					if (m >= 0) {
						double x1 = m_corners[iCam].data[2 * n];
						double y1 = m_corners[iCam].data[2 * n + 1];
						double x2 = m_corners[jCam].data[2 * m];
						double y2 = m_corners[jCam].data[2 * m + 1];
						m_cornerTracks[m_numCornerTracks].add(i, x1, y1);
						m_cornerTracks[m_numCornerTracks].add(i + 1, x2, y2);

						flag[iCam].data[n] = m_numCornerTracks;
						flag[jCam].data[m] = m_numCornerTracks;

						m_numCornerTracks++;
					}
				}
			}
		}
	}
}
//void InitMap::initFirstTwoCameras(std::vector<const Track2D*>& vecAllTracks, std::map<const Track2D*, bool>& flagAllTracks,int order) {
//	using namespace std;
//	assert(order >= 0 && order < numCams);
//	//use the camera pair that has the largest inter-camera distance 
//	int camId1 = camOrder[order];
//	int camId2 = camOrder[order + 1];
//	vector<const Track2D*> subTracks;
//	selectTracksDuring(vecAllTracks, 0, 1, subTracks);
//	//get feature correspondences from the longest tracks
//	Mat_d pts1, pts2;
//	int nMatched = getMatchingFromTracks(subTracks, order, order + 1, pts1, pts2);
//
//	//re-estimate the fundamental matrix between the first two views
//	Mat_uc inlierFlag;
//	estimateFmat(pts1, pts2, F[camId1][camId2], inlierFlag);
//
//	for (int i = 0; i < nMatched; i++) {
//		if (inlierFlag.data[i] == 0)
//			/*record outliers*/
//			flagAllTracks[subTracks[i]] = true;
//	}
//	double E[9];
//	getEMat(K[camId1], K[camId2], F[camId1][camId2], E);
//
////	double E[9], tF[9];
////	estimateEMat(K[camId1], K[camId2], pts1, pts2, E, 100, epiErrMax, 5);
////	getFMat(invK[camId1], invK[camId2], E, tF);
////
////	Mat_uc inlierFlag(nMatched, 1);
////	for (int i = 0; i < nMatched; i++) {
////		if ( epipolarError(tF,pts1.data+2*i,pts2.data+2*i) > epiErrMax)
////			inlierFlag.data[i] = 0;
////		else
////			inlierFlag.data[i] = 1;
////	}
//
////decompose the essential matrix
//	double Rs[36], ts[12];
//	decompEMat(E, Rs, ts);
//
//	Mat_d normPts1(nMatched, 2), normPts2(nMatched, 2);
//	int nInlier = 0;
//	for (int i = 0; i < nMatched; i++) {
//		if (inlierFlag.data[i] > 0) {
//			normPoint(invK[camId1], pts1.data + 2 * i, normPts1 + 2 * i);
//			normPoint(invK[camId2], pts2.data + 2 * i, normPts2 + 2 * i);
//			nInlier++;
//		}
//	}
//
//	Mat_d matR0, matT0;
//	matEyes(3, matR0);
//	matZeros(3, 1, matT0);
//	memcpy(camR[camId2], matR0.data, sizeof(double) * 9);
//	memcpy(camT[camId2], matT0.data, sizeof(double) * 3);
//
//	selectFacingRT(Rs, ts, nInlier, normPts1.data, normPts2.data, camR[camId1], camT[camId1]);
//
//	//triangulate the corresponding points
//	for (int i = 0; i < nMatched; i++) {
//		if (inlierFlag.data[i] > 0) {
//			double M[3];
//			binTriangulate(K[camId1], camR[camId1], camT[camId1], K[camId2], camR[camId2], camT[camId2], pts1.data + 2 * i, pts2.data + 2 * i, M);
//			Point3d pt3d(M[0], M[1], M[2]);
//			m_3Dpts[subTracks[i]] = pt3d;
//		}
//	}
//
//	Mat_d tmpPts1(nInlier, 2), tmpPts2(nInlier, 2), tmpPts3(nInlier, 3);
//
//	int k = 0;
//	for (int i = 0; i < nMatched; i++) {
//		if (inlierFlag.data[i] > 0) {
//			tmpPts1.data[2 * k] = pts1.data[2 * i];
//			tmpPts1.data[2 * k + 1] = pts1.data[2 * i + 1];
//			tmpPts2.data[2 * k] = pts2.data[2 * i];
//			tmpPts2.data[2 * k + 1] = pts2.data[2 * i + 1];
//			memcpy(tmpPts3.data + 3 * k, m_3Dpts[subTracks[i]].p, sizeof(double) * 3);
//			k++;
//		}
//	}
//
//	savePGM(*m_pImgGray[camId1], "/home/zou/img_01.pgm");
//	savePGM(*m_pImgGray[camId2], "/home/zou/img_02.pgm");
//	writeMat(tmpPts1, "/home/zou/pts1.txt");
//	writeMat(tmpPts2, "/home/zou/pts2.txt");
//	writeMat(tmpPts3, "/home/zou/x.txt");
//	writeMat(3, 3, K[camId1], "/home/zou/K.txt");
//	writeMat(3, 3, camR[camId1], "/home/zou/R1.txt");
//	writeMat(3, 1, camT[camId1], "/home/zou/t1.txt");
//	writeMat(3, 3, camR[camId2], "/home/zou/R2.txt");
//	writeMat(3, 1, camT[camId2], "/home/zou/t2.txt");
////test
//	logInfo("initFirstTwoCameras OK!\n");
//}

//void InitMap::computeExtrinsic() {
//	using namespace std;
//	if (m_numCornerTracks == 0)
//		repErr("MapInitializer::updateEMats - no feature track can be found");
//
////select tracks that are longer than 2
//	vector<const Track2D*> vecAllTracks;
//	map<const Track2D*, bool> flagAllTracks; /*outlier : true, inlier : false*/
//
//	int nSelected = selectTracks(m_numSurfTracks, m_surfTracks, 2, vecAllTracks);
//	nSelected += selectTracks(m_numCornerTracks, m_cornerTracks, 2, vecAllTracks);
//	logInfo("number of long tracks : %d\n", nSelected);
//
//	initFirstTwoCameras(vecAllTracks, flagAllTracks, 0);
//
//}

void InitMap::computeExtrinsic() {
	if (m_numCornerTracks == 0)
		repErr("MapInitializer::updateEMats - no feature track can be found");

	//use the camera pair that has the largest inter-camera distance 
	int firstCam = camOrder[0];
	int lastCam = camOrder[numCams - 1];

	using namespace std;
	//select point trajectories which have longest time duration
	vector<const Track2D*> vecAllTracks;
	int nSelected = selectTracks(m_numSurfTracks, m_surfTracks, numCams,
			vecAllTracks);
	nSelected += selectTracks(m_numCornerTracks, m_cornerTracks, numCams,
			vecAllTracks);
	logInfo("number of long tracks : %d\n", nSelected);

	//get feature correspondences from the longest tracks
	Mat_d pts1, pts2, pts1Norm, pts2Norm;
	getMatchingFromTracks(vecAllTracks, 0, numCams - 1, pts1, pts2);

	int numPts = pts1.rows;
	pts1Norm.resize(numPts, 2);
	pts2Norm.resize(numPts, 2);

	normPoints(invK[firstCam], numPts, pts1, pts1Norm);
	normPoints(invK[lastCam], numPts, pts2, pts2Norm);

	Mat_d tF(3, 3), tE(3, 3), tR0(3, 3), tT0(3, 1), tR1(3, 3), tT1(3, 1);

	Mat_uc inlierFlag;

	int inNumPts = estimateFmat(pts1, pts2, tF, inlierFlag);
	if (inNumPts == 0) {
		repErr("MapInitializer - cannot find essential matrix");
	}

	getEMat(K[firstCam], K[lastCam], tF.data, tE);

	Mat_d inPts1Norm(inNumPts, 2);
	Mat_d inPts2Norm(inNumPts, 2);

	getFlaged2DPoints(pts1Norm, inlierFlag, inPts1Norm);
	getFlaged2DPoints(pts2Norm, inlierFlag, inPts2Norm);

	matEyes(3, tR1);
	matZeros(3, 1, tT1);

	if (getCamPoseFromEMat(tE.data, inNumPts, inPts1Norm.data, inPts2Norm.data,
			tR0.data, tT0.data) == 0)
		repErr("getCamPoseFromEMat - no case is found!");

	Mat_d mapPts;
	mapPts.resize(inNumPts, 3);
	binTriangulatePoints(tR0.data, tT0.data, tR1.data, tT1.data, inNumPts,
			inPts1Norm.data, inPts2Norm.data, mapPts.data);

	memcpy(camR[firstCam], tR0.data, sizeof(double) * 9);
	memcpy(camT[firstCam], tT0.data, sizeof(double) * 3);
	memcpy(camR[lastCam], tR1.data, sizeof(double) * 9);
	memcpy(camT[lastCam], tT1.data, sizeof(double) * 3);

	for (int i = 1; i < numCams - 1; i++) {
		int iCam = camOrder[i];
		Mat_d pts2d;
		getPointsFromTracks(vecAllTracks, i, inlierFlag, pts2d);
		solvePnPRansac(inNumPts, mapPts, pts2d, K[iCam], camR[iCam], camT[iCam],
				2000);
	}
#ifdef DEBUG_MODE
	//	//test
	ImgRGB tmpImg;
	char wndName[256];
	for (int i = 0; i < numCams; i++) {
		int iCam = camOrder[i];
		Mat_d m;
		getPointsFromTracks(vecAllTracks, i, inlierFlag, m);
		//green -- key points
		drawKeyPoints(*m_pImgGray[iCam], m, tmpImg, 0, 255, 0);

		//gray2rgb(*m_pImgGray[firstCam], tmpImg);
		for (int k = 0; k < mapPts.rows; k++) {
			double rm[2];
			project(K[iCam], camR[iCam], camT[iCam], mapPts.M + 3 * k, rm);
			//red -- reprojected point
			drawPoint(tmpImg, rm[0], rm[1], 3, 255, 0, 0);
			drawLine(tmpImg, rm[0], rm[1], m.M[2 * k], m.M[2 * k + 1], 255, 0, 0);
		}
		sprintf(wndName, "img%d(%d)", iCam, i);
		imshow(wndName, tmpImg);
		cv::waitKey(-1);
	}
#endif
}
int InitMap::reconstructTracks(const Track2D tracks[], int numTracks,
		int minTrackLen, int frame0, std::vector<FeaturePoints*>& pFeaturePts,
		MapPointList& mapPts, double maxRpErr) {

	int num = 0;
	for (int k = 0; k < numTracks; k++) {
		if (tracks[k].length() >= minTrackLen) {
			Mat_d ms(numCams, 2);
			Mat_d nms(numCams, 2);
			Mat_d Ks(numCams, 9);
			Mat_d Rs(numCams, 9);
			Mat_d Ts(numCams, 3);

			int npts = 0;
			for (Track2DNode* pTkNode = tracks[k].head.next; pTkNode; pTkNode =
					pTkNode->next) {
				int iCam = camOrder[pTkNode->f];
				ms.data[2 * npts] = pTkNode->x;
				ms.data[2 * npts + 1] = pTkNode->y;

				//normalize the image coordinates of the feature points
				normPoint(invK[iCam], ms.data + 2 * npts, nms.data + 2 * npts);

				memcpy(Ks.data + 9 * npts, K[iCam], sizeof(double) * 9);
				memcpy(Rs.data + 9 * npts, camR[iCam], sizeof(double) * 9);
				memcpy(Ts.data + 3 * npts, camT[iCam], sizeof(double) * 3);

				npts++;
			}

			double M[3];
			triangulateMultiView(npts, Rs.data, Ts.data, nms.data, M);
			bool outlier = false;

			//check re-projection error
			for (int i = 0; i < npts; i++) {
				if (isAtCameraBack(Rs.data + 9 * i, Ts.data + 3 * i, M)) {
					outlier = true;
					break;
				}
				double rm[2];
				project(Ks.data + 9 * i, Rs.data + 9 * i, Ts.data + 3 * i, M,
						rm);
				double err = dist2(ms.data + 2 * i, rm);
				if (err > maxRpErr) {
					outlier = true;
					break;
				}
			}

			//if it is a inlier
			if (!outlier && num < SLAM_FEATURE_HEIGHT * SLAM_FEATURE_WIDTH) {
				MapPoint* pM = mapPts.add(M, frame0);
				//compute the triangulation covariance
				getTriangulateCovMat(npts, Ks.data, Rs.data, Ts.data, M,
						pM->cov, Const::PIXEL_ERR_VAR);

				for (Track2DNode* pTkNode = tracks[k].head.next; pTkNode;
						pTkNode = pTkNode->next) {
					int iCam = camOrder[pTkNode->f];
					FeaturePoint* pm = pFeaturePts[iCam]->add(frame0, iCam,
							pTkNode->x, pTkNode->y);
					//add the feature point to the map point
					pM->addFeature(iCam, pm);
					//compute the NCC block
					pM->nccBlks[iCam].computeScaled(*m_pImgGraySmall[iCam],
							m_imgScale[iCam], pm->m[0], pm->m[1]);

					//record the color of the map point
					if (pTkNode == tracks[k].head.next) {
						int x = max(0, min((int) pTkNode->x, m_pImgRGB[iCam]->w-1));
						int y = max(0, min((int) pTkNode->y, m_pImgRGB[iCam]->h-1));
						pM->setColor((*m_pImgRGB[iCam])(y, x));
					}
				}
				num++;
			}
		}
	}
	return num;
}
int InitMap::removeOutliers(MapPointList& mapPts, double errMax) {
	int removedNum = 0;
	MapPoint* pHead = mapPts.getHead();
	if (!pHead)
		repErr("no map points is reconstructed!");
	for (MapPoint* p = pHead; p; p = p->next) {
		if (p->numVisCam <= 1) {
			p->setFalse();
			removedNum++;
		} else {
			double rm[2];
			for (int i = 0; i < SLAM_MAX_NUM; i++) {
				if (p->pFeatures[i]) {
					project(K[i], camR[i], camT[i], p->M, rm);
					double d = dist2(p->pFeatures[i]->m, rm);
					if (d >= errMax) {
						p->isFalse();
						removedNum++;
						break;
					}
				}
			}
		}
	}
	return removedNum;
}

void InitMap::save(int curFrame, const char* dir_path,
		vector<FeaturePoints*>& featPts, MapPointList& mapPts) {
	writeMat(camOrder, "%s/camOrder.txt", dir_path);

	char filePath[1024];
	sprintf(filePath, "%s/camRT.txt", dir_path);

	ofstream file(filePath);
	if (!file)
		repErr("cannot open '%s' to write!", filePath);

	file << numCams << endl;
	for (int i = 0; i < numCams; ++i) {
		for (int k = 0; k < 9; ++k) {
			file << camR[i][k] << " ";
		}
		for (int k = 0; k < 9; ++k) {
			file << camT[i][k] << " ";
		}
		file << endl;
	}

	file.close();
	sprintf(filePath, "%s/featPts.txt", dir_path);
	file.open(filePath);
	if (!file)
		repErr("cannot open '%s' to write!", filePath);

	file << featPts.size() << endl;
	for (size_t i = 0; i < featPts.size(); ++i) {
		vector<FeaturePoint*> vecFeatPts;
		featPts[i]->getFrame(curFrame, vecFeatPts);

		file << vecFeatPts.size() << endl;
		for (size_t k = 0; k < vecFeatPts.size(); ++k) {
			FeaturePoint* fpt = vecFeatPts[k];
			file << fpt->f << " " << fpt->camId << " " << fpt->x << " "
					<< fpt->y << " " << fpt->mpt->id << endl;
		}
	}

	file.close();
	sprintf(filePath, "%s/mapPts.txt", dir_path);
	file.open(filePath);
	if (!file)
		repErr("cannot open '%s' to write!", filePath);

	int num = 0;
	for (MapPoint* p = mapPts.getHead(); p; p = p->next) {
		num++;
	}

	file << num << endl;
	for (MapPoint* p = mapPts.getHead(); p; p = p->next) {
		file << p->id << " " << p->x << " " << p->y << " " << p->z;
		for (int i = 0; i < 9; ++i) {
			file << " " << p->cov[i];
		}
		file << endl;
	}
}
void InitMap::_linkFeatMapPoints(int curFrame, vector<FeaturePoints*>& featPts,
		MapPointList& mapPts) {

	//build map id map
	map<int, MapPoint*> id_map;
	for (MapPoint* mpt = mapPts.getHead(); mpt; mpt = mpt->next) {
		id_map[mpt->id] = mpt;
	}

	for (size_t i = 0; i < featPts.size(); ++i) {
		for (FeaturePoint* fp = featPts[i]->getFrameHead(curFrame);
				fp != featPts[i]->getFrameTail(curFrame)->next; fp = fp->next) {
			int mpt_id = fp->mpt_id;
			if (id_map.count(mpt_id) > 0) {
				fp->mpt = id_map[mpt_id];
				fp->mpt_id = (longInt) (fp->mpt);
				fp->mpt->pFeatures[fp->camId] = fp;
			}
		}
	}

	for (MapPoint* mpt = mapPts.getHead(); mpt; mpt = mpt->next) {
		mpt->id = (longInt) mpt;
		for (int c = 0; c < SLAM_MAX_NUM; ++c) {
			FeaturePoint* fp = mpt->pFeatures[c];
			if (fp) {
				mpt->nccBlks[c].computeScaled(*m_pImgGraySmall[c],
						m_imgScale[c], fp->x, fp->y);
			}
		}
		mpt->updateVisCamNum(curFrame);
	}
}

void InitMap::load(int curFrame, const char* dir_path,
		vector<FeaturePoints*>& featPts, MapPointList& mapPts) {
	readMat(camOrder, "%s/camOrder.txt", dir_path);
	char filePath[1024];
	sprintf(filePath, "%s/camRT.txt", dir_path);

	ifstream file(filePath);
	if (!file)
		repErr("cannot open '%s' to read!", filePath);

	file >> numCams;
	for (int i = 0; i < numCams; ++i) {
		for (int k = 0; k < 9; ++k) {
			file >> camR[i][k];
		}
		for (int k = 0; k < 9; ++k) {
			file >> camT[i][k];
		}
	}

	file.close();
	sprintf(filePath, "%s/featPts.txt", dir_path);
	file.open(filePath);
	if (!file)
		repErr("cannot open '%s' to read!", filePath);

	size_t nviews;
	file >> nviews;

	assert(nviews == numCams);

	for (size_t i = 0; i < nviews; ++i) {
		size_t npts;
		file >> npts;
		for (size_t k = 0; k < npts; ++k) {
			int f, camId, mpt_id;
			double x, y;
			file >> f >> camId >> x >> y >> mpt_id;
			FeaturePoint* fp = featPts[i]->add(f, camId, x, y);
			fp->mpt_id = mpt_id;
		}
	}
	file.close();

	sprintf(filePath, "%s/mapPts.txt", dir_path);
	file.open(filePath);
	if (!file)
		repErr("cannot open '%s' to read!", filePath);

	int nmappts;
	file >> nmappts;
	for (int k = 0; k < nmappts; ++k) {
		int mpt_id;
		double x, y, z;

		file >> mpt_id >> x >> y >> z;
		MapPoint* mpt = mapPts.add(x, y, z, curFrame);
		mpt->id = mpt_id;

		for (int i = 0; i < 9; ++i) {
			file >> mpt->cov[i];
		}
	}
	file.close();

	_linkFeatMapPoints(curFrame, featPts, mapPts);
}
//test
//#include "MyApp.h"
//#include "SL_CoSLAM.h"
//#include "v3d_gpuklt.h"
//int main(int argc, char** argv) {
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
//	glutCreateWindow("test");
//	glewInit();
//	V3D_GPU::Cg_ProgramBase::initializeCg();
//
//	CoSLAM coSLAM;
//	coSLAM.addVideo("/media/VIDEO_DATA/smithstreet02/sm_95.avi", "/home/tsou/Video/sonycx700E/sonycal.txt", 2);
//	coSLAM.addVideo("/media/VIDEO_DATA/smithstreet02/sm_88.avi", "/home/tsou/Video/sonycx700E/sonycal.txt", 4);
//	coSLAM.addVideo("/media/VIDEO_DATA/smithstreet02/sm_78.avi", "/home/tsou/Video/sonycx700E/sonycal.txt", 0);
////	coSLAM.addVideo("/media/VIDEO_DATA/smithstreet02/sm_87.avi", "/home/tsou/Video/sonycx700E/sonycal.txt", 4);
//
//	coSLAM.init(506);
//	coSLAM.readFrame();
//	coSLAM.initMap();
//	cv::waitKey(-1);
//	return 0;
//}

//#include "SL_BundleAdjust.h"
//int main(int argc, char** argv) {
//	ImgG img1, img2;
//	loadPGM(img1, "/home/zou/img_01.pgm");
//	loadPGM(img2, "/home/zou/img_02.pgm");
//
//	Mat_d K, R1, t1, R2, t2, pts1, pts2, pts3;
//	readMat(K, "/home/zou/K.txt");
//	readMat(R1, "/home/zou/R1.txt");
//	readMat(t1, "/home/zou/t1.txt");
//	readMat(R2, "/home/zou/R2.txt");
//	readMat(t2, "/home/zou/t2.txt");
//	readMat(pts1, "/home/zou/pts1.txt");
//	readMat(pts2, "/home/zou/pts2.txt");
//	readMat(pts3, "/home/zou/x.txt");
//
//	assert(pts1.rows == pts2.rows);
//
//	using namespace std;
//	vector<Mat_d> Ks, Rs, Ts;
//	Ks.push_back(K);
//	Rs.push_back(R1);
//	Ts.push_back(t1);
//
//	Ks.push_back(K);
//	Rs.push_back(R2);
//	Ts.push_back(t2);
//
//	vector<Point3d> vecPts3D;
//	vector<vector<Meas2D> > vecMeas2D;
//
//	int npts = pts3.rows;
//	for (int i = 0; i < npts; i++) {
//		vecPts3D.push_back(Point3d(pts3.data[3 * i], pts3.data[3 * i + 1], pts3.data[3 * i + 2]));
//		vecMeas2D.push_back(vector<Meas2D>());
//		vecMeas2D.back().push_back(Meas2D(0, pts1.data[2 * i], pts1.data[2 * i + 1]));
//		vecMeas2D.back().push_back(Meas2D(1, pts2.data[2 * i], pts2.data[2 * i + 1]));
//	}
//
//	bundleAdjustRobust(1, Ks, Rs, Ts, 0, vecPts3D, vecMeas2D, 12.0, 3, 50);
//	for (int i = 0; i < npts; i++) {
//		memcpy(pts3.data + 3 * i, vecPts3D[i].p, sizeof(double) * 3);
//	}
//
//	memcpy(R1.data, Rs[0].data, sizeof(double) * 9);
//	memcpy(t1.data, Ts[0].data, sizeof(double) * 3);
//	memcpy(R2.data, Rs[1].data, sizeof(double) * 9);
//	memcpy(t2.data, Ts[1].data, sizeof(double) * 3);
//
//	for (int i = 0; i < npts; i++) {
//		double M[3];
//		binTriangulate(K.data, R1.data, t1.data, K.data, R2.data, t2.data, pts1.data + 2 * i, pts2.data + 2 * i, M);
//		memcpy(pts3.data + 3 * i, M, sizeof(double) * 3);
//	}
//
//	writeMat(R1, "/home/zou/R1.txt");
//	writeMat(t1, "/home/zou/t1.txt");
//	writeMat(R2, "/home/zou/R2.txt");
//	writeMat(t2, "/home/zou/t2.txt");
//	writeMat(pts3, "/home/zou/x.txt");
//
//	ImgRGB outImg;
//	drawMatching(img1, pts1, img2, pts2, outImg, 1.0, 0);
//	ImgRGB outImg1, outImg2;
//	drawReprojectedPoints(img1, K, R1, t1, pts1.rows, pts3, pts1, outImg1);
//	drawReprojectedPoints(img2, K, R2, t2, pts2.rows, pts3, pts2, outImg2);
//	imshow("matched", outImg);
//	imshow("img1", outImg1);
//	imshow("img2", outImg2);
//	cv::waitKey(-1);
//	return 0;
//}

