/*
 * SL_MergeCameraGroup.cpp
 *
 *  Created on: 2011-6-8
 *      Author: tsou
 */

#include "SL_MergeCameraGroup.h"
#include "app/SL_CoSLAM.h"
#include "slam/SL_PtrVec.h"
#include "slam/SL_CoSLAMHelper.h"
#include "slam/SL_FeatureMatching.h"

#include "matching/SL_GuidedNCCMatcher.h"
#include "matching/SL_SurfMatching.h"
#include "matching/SL_StereoMatcherHelper.h"

#include "math/SL_LinAlg.h"
#include "util/SL_Graph.h"
#include "util/SL_Utility.h"

#include "geometry/SL_Distortion.h"
#include "geometry/SL_EMatWrapper.h"
#include "geometry/SL_AbsoluteOrientation.h"
#include "geometry/SL_5point.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_RigidTransform.h"
#include "geometry/SL_Geometry.h"

#include "tools/SL_Print.h"

#include "imgproc/SL_ImageIO.h"
#include "tools/GUI_ImageViewer.h"
#include "tools/SL_DrawCorners.h"

#include <map>
#include <set>

MergeCameraGroup::MergeCameraGroup() :
		m_pCurKeyFrm(0), m_pFirstConstrainFrm(0), m_pFixedKeyFrm(0), m_gid1(-1), m_gid2(
				-1), m_nMergeInfo(0), m_b3D3D(false) {
	memset(imgW, 0, sizeof(int) * SLAM_MAX_NUM);
	memset(imgH, 0, sizeof(int) * SLAM_MAX_NUM);
	memset(m_mergeInfo, 0, sizeof(MergeInfo) * SLAM_MAX_NUM * SLAM_MAX_NUM);
}

MergeCameraGroup::~MergeCameraGroup() {
}
void MergeCameraGroup::setCurrentFrame(KeyFrame* kf) {
	m_pCurKeyFrm = kf;
}
void MergeCameraGroup::setImageSize(int iCam, int w, int h) {
	imgW[iCam] = w;
	imgH[iCam] = h;
}
int MergeCameraGroup::checkPossibleMergable(int minNum, double minAreaRatio,
		double maxCamDist) {
	m_nMergeInfo = 0;
	for (int i = 0; i < m_pCurKeyFrm->nGroup; i++) {
		for (int j = i + 1; j < m_pCurKeyFrm->nGroup; j++) {
			checkPossibleMergableBetween(i, j, m_pCurKeyFrm->camGroups[i],
					m_pCurKeyFrm->camGroups[j], minNum, minAreaRatio,
					maxCamDist);
		}
	}
	return m_nMergeInfo;
}
int MergeCameraGroup::checkPossibleMergableBetween(int groupId1, int groupId2,
		const CameraGroup& group1, const CameraGroup& group2, int minInNum,
		double minInAreaRatio, double maxCamDist) {

	for (int i = 0; i < group1.num; i++) {
		int iCam = group1.camIds[i];
		for (int j = 0; j < group2.num; j++) {
			int jCam = group2.camIds[j];
			if (checkViewOverlap(iCam, jCam, minInNum, minInAreaRatio)
					&& checkCamDist(iCam, jCam, maxCamDist)) {
				m_mergeInfo[m_nMergeInfo].set(m_pCurKeyFrm->f, iCam, groupId1,
						m_pCurKeyFrm->f, jCam, groupId2);
				m_nMergeInfo++;
			}
		}
	}
	return m_nMergeInfo;
}

bool MergeCameraGroup::checkCamDist(int iCam, int jCam, double maxCamDist) {
	double M1[3], M2[3];
	getCamCenter(m_pCurKeyFrm->pPose[iCam]->cam, M1);
	getCamCenter(m_pCurKeyFrm->pPose[jCam]->cam, M2);

	double d = dist3(M1, M2);
	if (d > maxCamDist)
		return false;
	return true;
}

bool MergeCameraGroup::checkViewOverlapFromTo(int iCam, int jCam, int minInNum,
		double minInAreaRatio) {
	//get the 3D map points corresponding to iCam-th camera
	KeyPose* kp = m_pCurKeyFrm->pPose[iCam];
	list<MapPoint*> mapPts;
	int nMapPts = 0;
	for (FeaturePoint* fp = kp->pHead; fp && fp != kp->pTail->next;
			fp = fp->next) {
		if (fp->mpt
				&& (fp->mpt->isLocalStatic() || fp->mpt->isLocalDynamic())) {
			mapPts.push_back(fp->mpt);
			nMapPts++;
		}
	}
	cout << "the number of map points:" << mapPts.size() << endl;
	//check the number of map points projected onto the jCam-th camera
	int nInCam = 0;
	const double* K2 = m_pCurKeyFrm->pPose[jCam]->K;
	CamPoseItem* cam2 = m_pCurKeyFrm->pPose[jCam]->cam;
	vector<double> ms;
	ms.reserve(2 * nMapPts);

	for (list<MapPoint*>::iterator iter = mapPts.begin(); iter != mapPts.end();
			iter++) {
		MapPoint* mpt = *iter;
		double m[2];
		project(K2, cam2->R, cam2->t, mpt->M, m);
		if (m[0] >= 0 && m[0] < imgW[jCam] && m[1] >= 0 && m[1] < imgH[jCam]) {
			ms.push_back(m[0]);
			ms.push_back(m[1]);
			nInCam++;
		}
	}

	//test
	cout << "cam#" << iCam << " -> cam#" << jCam << ": nIncam - " << nInCam
			<< endl;

	if (nInCam >= minInNum) {
		//check the convex hull of those projections
		vector<double> cxh;
		get2DConvexHull(ms, cxh);


		ImgG mask(imgW[jCam], imgH[jCam]);
		poly2Mask(cxh, mask);

		int inNum = 0;
		int totalNum = 0;
		kp = m_pCurKeyFrm->pPose[jCam];


		for (FeaturePoint* fp = kp->pHead; fp && fp != kp->pTail->next;
				fp = fp->next) {
			//if (fp->mpt && !fp->mpt->isFalse() && !fp->mpt->isUncertain()) {
			if (fp->mpt
					&& (fp->mpt->isLocalStatic() || fp->mpt->isLocalDynamic())) {
				mapPts.push_back(fp->mpt);
				int x = (int) fp->x;
				int y = (int) fp->y;
				if (mask(y, x) > 0) {
					inNum++;
				}
				totalNum++;
			}
		}

		cout << "inNum:" << inNum << " totalNum:" << totalNum << endl;
		if (inNum > 50 || inNum >= minInAreaRatio * totalNum) {
			return true;
		}
	}
	return false;
}

bool MergeCameraGroup::checkViewOverlap(int iCam, int jCam, int minInNum,
		double minInAreaRatio) {
	return checkViewOverlapFromTo(iCam, jCam, minInNum, minInAreaRatio)
			&& checkViewOverlapFromTo(jCam, iCam, minInNum, minInAreaRatio);
}

void MergeCameraGroup::storeFeaturePoints() {
	//store the feature points
	for (int i = 0; i < m_pCurKeyFrm->nCam; i++) {
		m_featPoints[i].clear();
		for (FeaturePoint* fp = m_pCurKeyFrm->pPose[i]->pHead;
				fp != m_pCurKeyFrm->pPose[i]->pTail->next; fp = fp->next) {
			m_featPoints[i].push_back(fp);
		}
	}
}

int MergeCameraGroup::computeEMat(int iCam, int jCam, const Mat_d& surfPts1,
		const Mat_d& surfPts2, const Matching& matches, Matching& newMatches,
		double E[9], int nRansac, double maxEpiErr, int minInlierNum) {

	const double* K1 = m_pCurKeyFrm->pPose[iCam]->K;
	const double* K2 = m_pCurKeyFrm->pPose[jCam]->K;

	estimateEMat(K1, K2, surfPts1, surfPts2, matches, E, nRansac, maxEpiErr,
			minInlierNum);

	int nm = matches.num;
	Mat_d pts1m(nm, 2), pts2m(nm, 2);
	Mat_d pts1mNorm(nm, 2), pts2mNorm(nm, 2);

	//get the matched points
	for (int i = 0; i < nm; i++) {
		int idx1 = matches[i].idx1;
		int idx2 = matches[i].idx2;

		pts1m.data[2 * i] = surfPts1.data[2 * idx1];
		pts1m.data[2 * i + 1] = surfPts1.data[2 * idx1 + 1];

		pts2m.data[2 * i] = surfPts2[2 * idx2];
		pts2m.data[2 * i + 1] = surfPts2[2 * idx2 + 1];
	}

	double iK1[9], iK2[9];
	mat33Inv(K1, iK1);
	mat33Inv(K2, iK2);
	double F[9];
	getFMat(iK1, iK2, E, F);

	//find the inliers
	Mat_uc inlierFlag(nm, 1);
	double epiErr;
	int inlierNum = evaluateEMat(nm, pts2m.data, pts1m.data, F, maxEpiErr,
			epiErr, inlierFlag.data);
	assert(inlierNum);
	if (inlierNum < minInlierNum)
		return 0;

	newMatches.clear();
	newMatches.reserve(nm);
	for (int i = 0; i < nm; i++) {
		if (inlierFlag.data[i] > 0) {
			int idx1 = matches[i].idx1;
			int idx2 = matches[i].idx2;
			newMatches.add(idx1, idx2, 0);
		}
	}
	return inlierNum;
}
void MergeCameraGroup::getCameraPose(const double E[9], double R[9],
		double t[3]) {
	//decompose the relative camera pose
	double Rs[36], ts[12];
	decompEMat(E, Rs, ts);

	double y0[3] = { 0, 1, 0 };

	//check the orientation of the camera Y-axis
	for (int i = 0; i < 4; i++) {
		double org[3], xp[3], yp[3], zp[3];
		getCameraCenterAxes(Rs + 9 * i, ts + 3 * i, org, xp, yp, zp);
		if (innerProd3(y0, yp) > 0) {
			memcpy(R, Rs + 9 * i, sizeof(double) * 9);
			memcpy(t, ts + 3 * i, sizeof(double) * 3);
		}
	}

}
/*match feature points between iCam-th camera and jCam-th camera*/
int MergeCameraGroup::matchFeaturePointsNCC(int iCam, int jCam,
		vector<FeaturePoint*>& featPts1, vector<FeaturePoint*>& featPts2,
		const Mat_d& surfPts1, const Mat_d& surfpts2, Matching& surfMatches,
		const double E[9], Matching& featMatches, double maxEpiErr,
		double minNcc, double maxDisp) {

	//compute the NCC blocks
	PtrVec<NCCBlock> nccblks1, nccblks2;
	getScaledNCCBlocks(m_pCurKeyFrm->pPose[iCam]->imgSmall,
			m_pCurKeyFrm->pPose[iCam]->imgScale, featPts1, nccblks1);
	getScaledNCCBlocks(m_pCurKeyFrm->pPose[jCam]->imgSmall,
			m_pCurKeyFrm->pPose[jCam]->imgScale, featPts2, nccblks2);

	Mat_d fpts1, fpts2;
	vecFeatPt2Mat(featPts1, fpts1);
	vecFeatPt2Mat(featPts2, fpts2);

	const double* K1 = m_pCurKeyFrm->pPose[iCam]->K;
	const double* K2 = m_pCurKeyFrm->pPose[jCam]->K;

	double iK1[9], iK2[9];
	mat33Inv(K1, iK1);
	mat33Inv(K2, iK2);

	//get the epipolar error and NCC matching score matrices
	Mat_d epiMat, nccMat;
	double Et[9], F[9];
	mat33Trans(E, Et);
	getFMat(iK2, iK1, Et, F);

	getEpiNccMat(F, fpts1, fpts2, nccblks1, nccblks2, maxEpiErr, minNcc, epiMat,
			nccMat);

	//get seed points
	Mat_d seedPts1, seedPts2;
	getMatchedPts(surfMatches, surfPts1, surfpts2, seedPts1, seedPts2);

	//get the disparity matrix
	Mat_d dispMat;
	getDisparityMat(fpts1, fpts2, seedPts1, seedPts2, maxDisp, dispMat);

	featMatches.clear();
	greedyGuidedNCCMatch(nccMat, dispMat, featMatches);

	return featMatches.num;
}
int MergeCameraGroup::detectSURFPoints(int iCam, Mat_d& surfPts,
		vector<float>& surfDesc, int &dimDesc) {
	ImgG& img = m_pCurKeyFrm->pPose[iCam]->img;
	dimDesc = ::detectSURFPoints(img, surfPts, surfDesc);
	return surfPts.rows;
}
int MergeCameraGroup::matchSURFPoints(int iCam, int jCam, const Mat_d& pts1,
		const Mat_d& pts2, int dimDesc, vector<float>&desc1,
		vector<float>& desc2, Matching& matches) {
	matchSurf(dimDesc, desc1, desc2, matches, 0.8, 0.6);
	return matches.num;
}
int MergeCameraGroup::matchMergableCameras() {
	const int nRansac = 200;
	const int minInlierNum = 15;

	assert(m_nMergeInfo > 0);
	map<pair<int, int> , vector<MergeInfo> > mergeInfos;

	//classify the merge information into different groups
	for (int i = 0; i < m_nMergeInfo; i++) {
		int gId1 = m_mergeInfo[i].groupId1;
		int gId2 = m_mergeInfo[i].groupId2;
		assert(gId1 < gId2);
		m_mergeInfo[i].valid = false;
		mergeInfos[make_pair(gId1, gId2)].push_back(m_mergeInfo[i]);
	}

	int numMatched = 0;
	typedef map<pair<int, int> , vector<MergeInfo> >::iterator MergeInfoIter;

	bool matched = false;
	int maxiCam = -1;
	int maxjCam = -1;
	int maxgId1 = -1;
	int maxgId2 = -1;
	int maxk = -1;

	Matching featMatches[SLAM_MAX_NUM * SLAM_MAX_NUM];
	int k = 0;
	for (MergeInfoIter iter = mergeInfos.begin(); iter != mergeInfos.end();
			iter++) {
		//choose the best matched camera pair
		int gId1 = iter->first.first;
		int gId2 = iter->first.second;
		int nMinNCCMatchNum = 20;

		vector<MergeInfo>& mgInfos = iter->second;
		for (size_t i = 0; i < mgInfos.size(); i++) {
			int iCam = mgInfos[i].camId1;
			int jCam = mgInfos[i].camId2;

			Mat_d surfPts1, surfPts2;
			vector<float> desc1, desc2;
			int dimDesc = 0;

			detectSURFPoints(iCam, surfPts1, desc1, dimDesc);
			detectSURFPoints(jCam, surfPts2, desc2, dimDesc);

			Matching matches, newMatches;
			if (matchSURFPoints(iCam, jCam, surfPts1, surfPts2, dimDesc, desc1,
					desc2, matches) < 25) {
				m_mergeInfo[i].valid = false;
				continue;
			}

			//use SURF to guide NCC matching
			double E[9];
			if (computeEMat(iCam, jCam, surfPts1, surfPts2, matches, newMatches,
					E, nRansac, 2.0, minInlierNum) > 0) {
				assert(
						!m_featPoints[iCam].empty() && !m_featPoints[jCam].empty());

				matchFeaturePointsNCC(iCam, jCam, m_featPoints[iCam],
						m_featPoints[jCam], surfPts1, surfPts2, newMatches, E,
						featMatches[k], 20.0, 0.45, 150);
				matched = true;
				if (featMatches[k].num > nMinNCCMatchNum) {
					//choose the best matched camera pair for merge
					nMinNCCMatchNum = featMatches[i].num;
					maxk = k;
					maxiCam = iCam;
					maxjCam = jCam;
					maxgId1 = gId1;
					maxgId2 = gId2;
				}
				k++;
			}
		}
	}
	if (matched) {
		//generate relative camera poses for merging	
		if (m_pCurKeyFrm->camGroups[maxgId1].num
				> m_pCurKeyFrm->camGroups[maxgId2].num) {
			m_gid1 = maxgId1;
			m_gid2 = maxgId2;

			m_camid1 = maxiCam;
			m_camid2 = maxjCam;

		} else {
			m_gid1 = maxgId2;
			m_gid2 = maxgId1;

			m_camid1 = maxjCam;
			m_camid2 = maxiCam;
		}

		if (genMergeInfoVer2(maxgId1, maxgId2, maxiCam, maxjCam,
				featMatches[maxk]))
			numMatched++;
	}
	return numMatched;
}
//test
#include "gui/CoSLAMThread.h"
#include "gui/MyApp.h"
bool MergeCameraGroup::checkMergeMapPoints(
		vector<vector<FeaturePoint*> >& featureTracks,
		vector<MapPoint*>& mapPoints, double thres) {

	assert(
			featureTracks.size() > 0 && featureTracks.size() == mapPoints.size());
	size_t npts = featureTracks.size();

	vector<double> vec_err;
	for (size_t i = 0; i < npts; i++) {
		MapPoint* mpt = mapPoints[i];
		size_t nviews = featureTracks[i].size();
		for (size_t v = 0; v < nviews; v++) {
			FeaturePoint* fp = featureTracks[i][v];
			double m[2];
			KeyPose* pose = ind_to_pose[make_pair(fp->f, fp->camId)];
			project(pose->K, pose->cam->R, pose->cam->t, mpt->M, m);

			double err = dist2(m, fp->m);
			vec_err.push_back(err);
		}
		mpt->updateVisCamNum(m_pCurKeyFrm->f);
	}

	std::partial_sort(vec_err.begin(), vec_err.begin() + 3, vec_err.end(),
			greater<double>());

	double avg_err = 0;
	for (int i = 0; i < 3; ++i) {
		avg_err += vec_err[i];
	}

	avg_err /= 3;

	//test
	cout << "avg_err:" << avg_err << endl;
	//CoSLAM::ptr->pause();
	if (avg_err > thres)
		return false;

	return true;
}

void MergeCameraGroup::mergeMapPoints(
		vector<vector<FeaturePoint*> >& featureTracks,
		vector<MapPoint*>& mapPoints) {

	assert(
			featureTracks.size() > 0 && featureTracks.size() == mapPoints.size());
	size_t npts = featureTracks.size();
	for (size_t i = 0; i < npts; i++) {
		MapPoint* mpt = mapPoints[i];
		size_t nviews = featureTracks[i].size();
		for (size_t v = 0; v < nviews; v++) {
			FeaturePoint* fp0 = featureTracks[i][v];
			//update the pointer to the corresponding map point
			for (FeaturePoint* fp = fp0; fp; fp = fp->preFrame) {
				fp->mpt = mpt;
			}
			mpt->pFeatures[fp0->camId] = fp0;
		}
		mpt->updateVisCamNum(m_pCurKeyFrm->f);
	}
}
void MergeCameraGroup::getCamIdsInBothGroups(const CameraGroup* group1,
		const CameraGroup* group2, vector<int>& camIds) {

	camIds.clear();

	Mat_i camFlags(1, SLAM_MAX_NUM);
	camFlags.fill(0);

	const CameraGroup* group = group1;
	for (int c = 0; c < group->num; c++) {
		int camId = group->camIds[c];
		camFlags.data[camId] = 1;
	}
	group = group2;
	for (int g = 0; g < group->num; g++) {
		int camId = group->camIds[g];
		camFlags.data[camId] = 1;
	}
	for (int c = 0; c < SLAM_MAX_NUM; c++) {
		if (camFlags.data[c] > 0)
			camIds.push_back(c);
	}
}
int MergeCameraGroup::buildIdMapForSelfKeyPoses(const vector<int>& camIds,
		int nSelfMotion) {
	vector<vector<KeyPose*> > vecPoses;

	KeyFrame* min_kf = m_pCurKeyFrm;
	for (size_t i = 0; i < camIds.size(); ++i) {
		int c = camIds[i];
		int nself = 0;
		for (KeyFrame* kf = m_pCurKeyFrm->prev; kf && nself < nSelfMotion; kf =
				kf->prev) {
			if (kf->pPose[c]->bSelfMotion) {
				if (min_kf->f >= kf->pPose[c]->frame) {
					min_kf = kf;
				}
				nself++;
			}
		}
	}

	//test
	cout << "current frame:" << m_pCurKeyFrm->f << endl;
	cout << "min frame:" << min_kf->f << endl;

	KeyFrame* kfs[2];
	kfs[0] = m_pCurKeyFrm;
	kfs[1] = min_kf;

	m_pFirstConstrainFrm = min_kf;

	int nPose = 0;
	for (int i = 0; i < 2; ++i) {
		KeyFrame* kf = kfs[i];
		for (size_t j = 0; j < camIds.size(); ++j) {
			int c = camIds[j];
			KeyPose* pose = kf->pPose[c];
			order_to_pose[nPose] = pose;
			pose_to_order[pose] = nPose;
			ind_to_order[make_pair(pose->cam->f, pose->cam->camId)] = nPose;
			ind_to_pose[make_pair(pose->cam->f, pose->cam->camId)] = pose;
			nPose++;
		}
	}
	return nPose;
}

bool MergeCameraGroup::genMergeInfoVer2(int gId1, int gId2, int maxiCam,
		int maxjCam, const Matching& featMatches) {
	using namespace std;

	getCamIdsInBothGroups(&m_pCurKeyFrm->camGroups[gId1],
			&m_pCurKeyFrm->camGroups[gId2], m_camIds);

	int nPose = buildIdMapForSelfKeyPoses(m_camIds, 2);

	//test
	cout << "nPose : " << nPose << endl;

	vector<vector<FeaturePoint*> > merged_featpts;
	vector<MapPoint*> merged_mappts;

	getCorresFeatPtsAndMapPts(gId1, gId2, m_featPoints[maxiCam],
			m_featPoints[maxjCam], featMatches, merged_featpts, merged_mappts);

	if (!checkMergeMapPoints(merged_featpts, merged_mappts))
		return false;

	//use the the coordinates of one original map point as that of the merged map point
	mergeMapPoints(merged_featpts, merged_mappts);

	//test
	cout << "merged_mappts:" << merged_mappts.size() << endl;

	getUnMergedFeatureTracks(gId1, gId2, merged_featpts, merged_mappts,
			(int) merged_mappts.size() * 20);
	//test
//	cout << "merged_mappts:" << merged_mappts.size() << endl;
//	CoSLAM::ptr->pause();

	int nNewFeat = getCorresFeatPtsInPrevSelfKeyPoses(merged_featpts);

	//test
	cout << "nNewFeat:" << nNewFeat << endl;

	vector<vector<FeaturePoint*> > featpts;
	vector<MapPoint*> mappts;

	for (size_t i = 0; i < merged_featpts.size(); ++i) {
		if (merged_featpts[i].size() > 1) {
			featpts.push_back(merged_featpts[i]);
			mappts.push_back(merged_mappts[i]);
		}
	}

	vector<Mat_d> Ks, Rs, ts;
	for (int n = 0; n < nPose; ++n) {
		KeyPose* pose = order_to_pose[n];
		Mat_d K, R, t;
		K.cloneFrom(pose->K, 3, 3);
		R.cloneFrom(pose->cam->R, 3, 3);
		t.cloneFrom(pose->cam->t, 3, 1);

		Ks.push_back(K);
		Rs.push_back(R);
		ts.push_back(t);
	}

	//for bundle adjustment
	vector<vector<Meas2D> > meas2ds;
	vector<Point3d> pts3d;

	int npts = (int) mappts.size();
	for (int i = npts - 1; i >= 0; --i) {
		vector<Meas2D> meas2d;
		map<pair<int, int> , bool> flag;
		for (size_t j = 0; j < featpts[i].size(); j++) {
			FeaturePoint* fp = featpts[i][j];
			assert(ind_to_order.count(make_pair(fp->f, fp->camId)) > 0);

			//to avoid multiple points in the same view
			if (flag.count(make_pair(fp->f, fp->camId)) > 0) {
				fp->mpt = 0;
				continue;
			}
			int cam_id = ind_to_order[make_pair(fp->f, fp->camId)];
			double* m = fp->m;
			meas2d.push_back(Meas2D(cam_id, m[0], m[1]));
			flag[make_pair(fp->f, fp->camId)] = true;
		}
		meas2ds.push_back(meas2d);
		double* M = mappts[i]->M;
		pts3d.push_back(Point3d(M[0], M[1], M[2]));
	}

	//bundle adjust robust
	bundleAdjustRobust(1, Ks, Rs, ts, 1, pts3d, meas2ds, 800, 1, 100);
	bundleAdjustRobust(1, Ks, Rs, ts, 1, pts3d, meas2ds, 30, 5, 30);

	//write back both camera poses and the 3D coordinates for these merged map points
	vector<Mat_d> oldRs, oldTs;
	for (int n = 0; n < nPose; ++n) {
		KeyPose* pose = order_to_pose[n];

		oldRs.push_back(Mat_d());
		oldRs.back().cloneFrom(pose->cam->R, 3, 3);
		oldTs.push_back(Mat_d());
		oldTs.back().cloneFrom(pose->cam->t, 3, 1);

		//copy back the new camera poses
		memcpy(pose->cam->R, Rs[n].data, sizeof(double) * 9);
		memcpy(pose->cam->t, ts[n].data, sizeof(double) * 3);
	}

	//copy back the new 3D coordinates
	for (int i = 0; i < npts; ++i) {
		mappts[npts - i - 1]->M[0] = pts3d[i].M[0];
		mappts[npts - i - 1]->M[1] = pts3d[i].M[1];
		mappts[npts - i - 1]->M[2] = pts3d[i].M[2];
	}

	//recover
	for (int n = 0; n < nPose; ++n) {
		KeyPose* pose = order_to_pose[n];
		memcpy(pose->cam->R, oldRs[n].data, sizeof(double) * 9);
		memcpy(pose->cam->t, oldTs[n].data, sizeof(double) * 3);
	}

	//end of debug
	///////////////////////////////////////////////////////////////////////////////////

	//compute relative camera poses in current key frame
	int camId1 = m_pCurKeyFrm->camGroups[gId1].camIds[0];
	for (int j = 0; j < m_pCurKeyFrm->camGroups[gId2].num; j++) {
		int camId2 = m_pCurKeyFrm->camGroups[gId2].camIds[j];

		int p = ind_to_order[make_pair(m_pCurKeyFrm->f, camId1)];
		int q = ind_to_order[make_pair(m_pCurKeyFrm->f, camId2)];

		double dR[9], dt[3], E[9];
		getRigidTransFromToWithEMat(Rs[p].data, ts[p].data, Rs[q].data,
				ts[q].data, dR, dt, E);
		addMergeInfo(true, m_pCurKeyFrm->f, camId1, gId1, m_pCurKeyFrm->f,
				camId2, gId2, E, dR, dt);
	}

	int camId2 = m_pCurKeyFrm->camGroups[gId2].camIds[0];
	for (int i = 1; i < m_pCurKeyFrm->camGroups[gId1].num; i++) {
		int camId1 = m_pCurKeyFrm->camGroups[gId1].camIds[i];
		int p = ind_to_order[make_pair(m_pCurKeyFrm->f, camId1)];
		int q = ind_to_order[make_pair(m_pCurKeyFrm->f, camId2)];

		double E[9], dR[9], dt[3];
		getRigidTransFromToWithEMat(Rs[p].data, ts[p].data, Rs[q].data,
				ts[q].data, dR, dt, E);
		addMergeInfo(true, m_pCurKeyFrm->f, camId1, gId1, m_pCurKeyFrm->f,
				camId2, gId2, E, dR, dt);
	}

	//compute relative camera poses in previous key frame
	int f1 = m_pFirstConstrainFrm->f;
	int f2 = m_pCurKeyFrm->f;
	for (size_t i = 0; i < m_camIds.size(); ++i) {
		int c = m_camIds[i];
		double E[9], dR[9], dt[3];

		int p = ind_to_order[make_pair(f2, c)];
		int q = ind_to_order[make_pair(f1, c)];

		getRigidTransFromToWithEMat(Rs[p].data, ts[p].data, Rs[q].data,
				ts[q].data, dR, dt, E);
		addMergeInfo(true, f2, c, -1, f1, c, -1, E, dR, dt);
	}

	return true;
}
int MergeCameraGroup::getCorresFeatPtsInPrevSelfKeyPoses(
		vector<vector<FeaturePoint*> >& featpts) {

	size_t npts = featpts.size();
	vector<vector<FeaturePoint*> > featpts_prev;
	featpts_prev.resize(npts);

//scan the feature points
	int k = 0;
	for (size_t i = 0; i < npts; i++) {
		size_t nviews = featpts[i].size();
		for (size_t v = 0; v < nviews; v++) {
			FeaturePoint* fp0 = featpts[i][v]->preFrame;
			if (!fp0)
				continue;
			for (FeaturePoint* fp = fp0; fp; fp = fp->preFrame) {
				int c = fp->camId;
				int f = fp->f;
				if (ind_to_pose.count(make_pair(f, c)) > 0) {
					featpts_prev[i].push_back(fp);
					k++;
				}
			}
		}
	}

//combine the feature points in previous key frame
	for (size_t i = 0; i < npts; i++) {
		size_t nprev = featpts_prev[i].size();
		for (size_t j = 0; j < nprev; ++j) {
			featpts[i].push_back(featpts_prev[i][j]);
		}
	}

	return k;
}
int MergeCameraGroup::getCorresFeatPtsAndMapPts(int gId1, int gId2,
		vector<FeaturePoint*>& featPts1, vector<FeaturePoint*>& featPts2,
		const Matching& featMatches, vector<vector<FeaturePoint*> >& featTracks,
		vector<MapPoint*>& mapPts) {

	int nCam = m_pCurKeyFrm->nCam;
	Mat_i camFlags(1, nCam);
	camFlags.fill(0);

	CameraGroup* group = &m_pCurKeyFrm->camGroups[gId1];
	for (int g = 0; g < group->num; g++) {
		int camId = group->camIds[g];
		camFlags.data[camId] = 1;
	}

	group = &m_pCurKeyFrm->camGroups[gId2];
	for (int g = 0; g < group->num; g++) {
		int camId = group->camIds[g];
		camFlags.data[camId] = 1;
	}

	int nm = featMatches.num;
	for (int i = 0; i < nm; i++) {
		int id1 = featMatches[i].idx1;
		int id2 = featMatches[i].idx2;

		FeaturePoint* fp1 = featPts1[id1];
		FeaturePoint* fp2 = featPts2[id2];

		//scan the corresponding feature points in other views
		if (fp1->mpt && fp2->mpt && fp1->f == fp2->f) {
			vector<FeaturePoint*> tk;
			for (int c = 0; c < nCam; c++) {
				if (camFlags.data[c] <= 0)
					continue;
				MapPoint* mpt = fp1->mpt;
				if (mpt->pFeatures[c] && mpt->pFeatures[c]->f == fp1->f) {
					tk.push_back(mpt->pFeatures[c]);
				}
				mpt = fp2->mpt;
				if (mpt->pFeatures[c] && mpt->pFeatures[c]->f == fp1->f) {
					tk.push_back(mpt->pFeatures[c]);
				}
			}
			featTracks.push_back(tk);
			mapPts.push_back(fp1->mpt);
		}
	}
	return (int) mapPts.size();
}
int MergeCameraGroup::getUnMergedFeatureTracks(int gId1, int gId2,
		vector<vector<FeaturePoint*> >& featTracks, vector<MapPoint*>& mapPts,
		int maxNum) {
	int nCam = m_pCurKeyFrm->nCam;
	Mat_i camFlags(1, nCam);
	camFlags.fill(0);

	CameraGroup* group = &m_pCurKeyFrm->camGroups[gId1];
	for (int g = 0; g < group->num; g++) {
		int camId = group->camIds[g];
		camFlags.data[camId] = 1;
	}
	group = &m_pCurKeyFrm->camGroups[gId2];
	for (int g = 0; g < group->num; g++) {
		int camId = group->camIds[g];
		camFlags.data[camId] = 1;
	}

	assert(featTracks.size() == mapPts.size());

	map<MapPoint*, bool> mapPtsFlag;
	for (size_t i = 0; i < mapPts.size(); i++) {
		mapPtsFlag[mapPts[i]] = true;
	}

//scan the map points that are not merged;
	int f = m_pCurKeyFrm->f;
	vector<MapPoint*> unmergedMapPts;
	for (int c = 0; c < nCam; c++) {
		if (camFlags.data[c] <= 0)
			continue;
		for (size_t i = 0; i < m_featPoints[c].size(); i++) {
			MapPoint* mpt = m_featPoints[c][i]->mpt;
			if (mpt) {
				if (mapPtsFlag.count(mpt) == 0) {
					unmergedMapPts.push_back(mpt);
				}
			}
		}
	}

//sort these unmerged map points with its number of visible views
	vector<int> numVis;
	vector<int> ind;
	for (size_t i = 0; i < unmergedMapPts.size(); ++i) {
		numVis.push_back(unmergedMapPts[i]->numVisCam);
		ind.push_back((int) i);
	}
	sortWithInd((int) numVis.size(), &numVis[0], &ind[0], false);

	int num = 0;
	for (size_t i = 0; i < ind.size(); i++) {
		vector<FeaturePoint*> tk;
		int k = ind[i];
		MapPoint* mpt = unmergedMapPts[k];
		for (int c = 0; c < nCam; c++) {
			if (camFlags.data[c] <= 0)
				continue;
			if (mpt->pFeatures[c] && mpt->pFeatures[c]->f == f) {
				tk.push_back(mpt->pFeatures[c]);
			}
		}
		if (tk.size() >= 1 && num < maxNum) {
			featTracks.push_back(tk);
			mapPts.push_back(mpt);
			num++;
		}
	}

	return num;
}

void MergeCameraGroup::searchFirstKeyFrameForMerge(int nMaxKeyFrms) {
	int firstFrameForConstraint = m_pFirstConstrainFrm->f;
	int n = 0;
	bool find_samegroup = false;
	for (KeyFrame* kf = m_pCurKeyFrm; kf && n <= nMaxKeyFrms && !find_samegroup;
			kf = kf->prev) {
		if (kf->f >= firstFrameForConstraint)
			continue;
		for (int g = 0; g < kf->nGroup; ++g) {
			if (kf->camGroups[g].isCameraIn(m_camid1)
					&& kf->camGroups[g].isCameraIn(m_camid2)) {
				find_samegroup = true;
				break;
			}
		}
		m_pFixedKeyFrm = kf;
		n++;
	}
	//test
	cout << m_pFixedKeyFrm->f << endl;

	assert(m_pFixedKeyFrm);
}
void MergeCameraGroup::_constructGraphForKeyFrms(KeyFrame* frame0,
		GlobalPoseGraph& graph) {

	assert(frame0);
	graph.clear();

	int nTotalNode = 0;
	for (KeyFrame* kf = frame0; kf; kf = kf->next)
		nTotalNode += kf->nCam;

	graph.reserve(nTotalNode, nTotalNode * 3); //allocate enough nodes and edges

	m_keyPoseNodeId.clear();
	m_pKeyPoses.clear();
	m_pKeyPoses.reserve(nTotalNode);
	graph.nFixedNode = 0;

	map<int, bool> camFlag;
	for (size_t i = 0; i < m_camIds.size(); ++i) {
		int c = m_camIds[i];
		camFlag[c] = true;
	}

	//add nodes
	for (KeyFrame* kf = frame0; kf; kf = kf->next) {
		for (size_t i = 0; i < m_camIds.size(); ++i) {
			int c = m_camIds[i];
			CamPoseNode* pNode = graph.newNode();
			CamPoseItem* cam = kf->pPose[c]->cam;
			pNode->set(kf->f, c, cam->R, cam->t);
			m_keyPoseNodeId[cam] = pNode->id;
			pNode->fixed = kf == frame0 ? true : false;
			m_pKeyPoses.push_back(cam);
			if (pNode->fixed)
				graph.nFixedNode++;
		}
	}

	//add links
	double R[9], t[3];
	for (KeyFrame* kf = frame0; kf; kf = kf->next) {
		//connect poses in the same group
		for (int g = 0; g < kf->nGroup; g++) {

			//get cameras to be merged
			vector<int> camids;
			for (int i = 0; i < kf->camGroups[g].num; ++i) {
				int c = kf->camGroups[g].camIds[i];
				if (camFlag.count(c) > 0) {
					camids.push_back(c);
				}
			}

			if (camids.size() > 1 && kf->f <= m_pFirstConstrainFrm->f) {
				size_t i = 1;
				for (; i < camids.size(); i++) {
					int camId1 = camids[i - 1];
					int camId2 = camids[i];
					CamPoseItem* cam1 = kf->pPose[camId1]->cam;
					CamPoseItem* cam2 = kf->pPose[camId2]->cam;
					int nodeId1 = m_keyPoseNodeId[cam1];
					int nodeId2 = m_keyPoseNodeId[cam2];
					getRigidTransFromTo(cam1->R, cam1->t, cam2->R, cam2->t, R,
							t);

					CamPoseEdge* pEdge = graph.addEdge();
					pEdge->set(nodeId1, nodeId2, R, t);
				}
				if (camids.size() > 2) {
					int camId1 = camids[i - 1];
					int camId2 = camids[0];
					CamPoseItem* cam1 = kf->pPose[camId1]->cam;
					CamPoseItem* cam2 = kf->pPose[camId2]->cam;
					int nodeId1 = m_keyPoseNodeId[cam1];
					int nodeId2 = m_keyPoseNodeId[cam2];
					getRigidTransFromTo(cam1->R, cam1->t, cam2->R, cam2->t, R,
							t);
					CamPoseEdge* pEdge = graph.addEdge();
					pEdge->set(nodeId1, nodeId2, R, t);
				}
			}
		}

		if (kf != frame0) {
			//connect poses at the successive frames
			for (size_t i = 0; i < m_camIds.size(); ++i) {
				int c = m_camIds[i];
				CamPoseItem* cam1 = kf->prev->pPose[c]->cam;
				CamPoseItem* cam2 = kf->pPose[c]->cam;

				int nodeId1 = m_keyPoseNodeId[cam1];
				int nodeId2 = m_keyPoseNodeId[cam2];
				getRigidTransFromTo(cam1->R, cam1->t, cam2->R, cam2->t, R, t);
				CamPoseEdge* pEdge = graph.addEdge();
				pEdge->set(nodeId1, nodeId2, R, t);
			}
		}
	}

	//add constraint links
	graph.nConstraintEdge = 0;
	for (int i = 0; i < m_nMergeInfo; i++) {
		if (m_mergeInfo[i].valid == true) {
			int f1 = m_mergeInfo[i].frame1;
			int c1 = m_mergeInfo[i].camId1;

			int f2 = m_mergeInfo[i].frame2;
			int c2 = m_mergeInfo[i].camId2;

			KeyPose* pose1 = ind_to_pose[make_pair(f1, c1)];
			KeyPose* pose2 = ind_to_pose[make_pair(f2, c2)];

			CamPoseItem* cam1 = pose1->cam;
			CamPoseItem* cam2 = pose2->cam;

			int nodeId1 = m_keyPoseNodeId[cam1];
			int nodeId2 = m_keyPoseNodeId[cam2];

			CamPoseEdge* pEdge = graph.addEdge();
			pEdge->set(nodeId1, nodeId2, m_mergeInfo[i].R, m_mergeInfo[i].t, 0);
			pEdge->uncertainScale = true;
			pEdge->constraint = true;

			graph.poseNodes[nodeId1].constraint = true;
			graph.poseNodes[nodeId2].constraint = true;
			graph.nConstraintEdge++;
		}
	}
}

void MergeCameraGroup::_constructGraphForAllFrms(KeyFrame* frame0,
		GlobalPoseGraph graphs[]) {
	assert(frame0);
	int numCams = frame0->nCam;
	for (int c = 0; c < numCams; c++) {
		graphs[c].clear();
		m_camNodeId[c].clear();
		m_cams[c].clear();
		int nTotalNode = 0;
		for (CamPoseItem* cam = frame0->pPose[c]->cam; cam; cam = cam->next)
			nTotalNode++;

		graphs[c].reserve(nTotalNode, nTotalNode);

		//add nodes
		for (CamPoseItem* cam = frame0->pPose[c]->cam; cam; cam = cam->next) {
			CamPoseNode* node = graphs[c].newNode();
			node->set(cam->f, cam->camId, cam->R, cam->t);

			//record the pointers
			m_camNodeId[c][cam] = node->id;
			m_cams[c].push_back(cam);
		}

		//set fixed nodes
		for (KeyPose* kp = frame0->pPose[c]; kp; kp = kp->next) {
			CamPoseItem* cam = kp->cam;
			int id = m_camNodeId[c][cam];
			CamPoseNode* fixedNode = &graphs[c].poseNodes[id];
			fixedNode->fixed = true;
		}

		//add edges
		CamPoseItem* cam0 = frame0->pPose[c]->cam->next;
		assert(cam0);
		for (CamPoseItem* cam = cam0; cam; cam = cam->next) {
			int id1 = m_camNodeId[c][cam->pre];
			int id2 = m_camNodeId[c][cam];
			CamPoseEdge* edge = graphs[c].addEdge();

			double R[9], t[3];
			getRigidTransFromTo(cam->pre->R, cam->pre->t, cam->R, cam->t, R, t);
			edge->set(id1, id2, R, t);
		}
	}
}
void MergeCameraGroup::recomputeKeyCamPoses() {

	keyGraph.computeNewCameraRotations();
	keyGraph.computeNewCameraTranslations4();

	//update the camera poses
	for (int i = 0; i < keyGraph.nNodes; i++) {
		CamPoseItem* cam = m_pKeyPoses[i];
		memcpy(cam->R, keyGraph.poseNodes[i].newR, sizeof(double) * 9);
		memcpy(cam->t, keyGraph.poseNodes[i].newt, sizeof(double) * 3);

		//update the camera poses in camGraph
		int c = cam->camId;
		int nodeId = m_camNodeId[c][cam];
		//		camGraphs[c].poseNodes[nodeId].set(cam->R,cam->t);
		memcpy(camGraphs[c].poseNodes[nodeId].R, m_cams[c][nodeId]->R,
				sizeof(double) * 9);
		memcpy(camGraphs[c].poseNodes[nodeId].t, m_cams[c][nodeId]->t,
				sizeof(double) * 3);
	}
}
void MergeCameraGroup::recomputeAllCameraPoses() {
	int nCam = m_pFixedKeyFrm->nCam;
	for (int c = 0; c < nCam; c++) {
		camGraphs[c].computeNewCameraRotations();
		camGraphs[c].computeNewCameraTranslations();
		//update the camera poses;
		for (int i = 0; i < camGraphs[c].nNodes; i++) {
			CamPoseItem* cam = m_cams[c][i];
			memcpy(cam->R, camGraphs[c].poseNodes[i].newR, sizeof(double) * 9);
			memcpy(cam->t, camGraphs[c].poseNodes[i].newt, sizeof(double) * 3);
		}
	}
}
int MergeCameraGroup::mergeMatchedGroups(CameraGroup groups[SLAM_MAX_NUM],
		int& nGroup) {
	Mat_i A(nGroup, nGroup);
	A.fill(0);

	for (int i = 0; i < m_nMergeInfo; i++) {
		if (m_mergeInfo[i].valid) {
			int iGroup = m_mergeInfo[i].groupId1;
			int jGroup = m_mergeInfo[i].groupId2;

			A.data[iGroup * nGroup + jGroup] = 1;
			A.data[jGroup * nGroup + iGroup] = 1;
		}
	}

	vector<Mat_i> conns;
	findConnectedComponents(A, conns);

	CameraGroup tGroups[SLAM_MAX_NUM];
	for (int i = 0; i < nGroup; i++) {
		tGroups[i].copy(groups[i]);
	}

	int k = 0;
	for (size_t i = 0; i < conns.size(); i++) {
		groups[i].clear();
		for (int g = 0; g < conns[i].n; g++) {
			int gid = conns[i].data[g];
			for (int c = 0; c < tGroups[gid].num; c++) {
				groups[i].addCam(tGroups[gid].camIds[c]);
			}
		}
		k++;
	}
	nGroup = k;

	//test
	for (int g = 0; g < nGroup; ++g) {
		cout << " group " << g << ": ";
		for (int i = 0; i < groups[g].num; ++i) {
			cout << groups[g].camIds[i] << " " ;
		}
		cout << endl;
	}
	
	int mg = -1;
	for (int g = 0; g < nGroup && mg < 0; ++g) {
		for (int i = 0; i < groups[g].num; ++i) {
			if (groups[g].camIds[i] == m_camid1
					|| groups[g].camIds[i] == m_camid2) {
				mg = g;
				break;
			}
		}
	}
	assert(mg >= 0);
	return mg;
}
void MergeCameraGroup::recomputeMapPoints(vector<MapPoint*>& mapPoints,
		double pixel_var) {
//update the 3D position of map point between the key frame firstFrame and lastFrame
//re- triangulate these map points
	int numCams = m_pCurKeyFrm->nCam;
	for (size_t i = 0; i < mapPoints.size(); i++) {
		updateStaticPointPositionAtKeyFrms(numCams, mapPoints[i], pixel_var);
	}
}
void MergeCameraGroup::mergeMatchedFeaturePoints() {
	for (int i = 0; i < m_nMergeInfo; i++) {
		typedef vector<pair<FeaturePoint*, FeaturePoint*> > MatchedFeaturePoints;
		MatchedFeaturePoints& matchedFeatPts = m_mergeInfo[i].matchedFeatPts;
		for (size_t s = 0; s < matchedFeatPts.size(); s++) {

		}
	}
}
void MergeCameraGroup::printMergeInfo() {
	logInfo("#####merge info######\n");
	for (int i = 0; i < m_nMergeInfo; i++) {
		if (m_mergeInfo[i].valid && m_mergeInfo[i].groupId1 >= 0) {
			logInfo("[group#%d - group#%d] ", m_mergeInfo[i].groupId1,
					m_mergeInfo[i].groupId2);
			logInfo(" cam#%d - cam#%d\n", m_mergeInfo[i].camId1,
					m_mergeInfo[i].camId2);
			logInfo("valied: true\n");
			logInfo("R:\n");
			printMat(1, 9, m_mergeInfo[i].R);
			logInfo("t:\n");
			printMat(1, 3, m_mergeInfo[i].t);
		}
	}
	logInfo("#####end of merge info######\n");
}

void getRigidTransFromToWithEMat(const double* R0, const double* T0,
		const double* R1, const double* T1, double dR[3], double dT[3],
		double E[9]) {
	Mat_d I, t0;
	matEyes(3, I);
	matZeros(3, 1, t0);
	getRigidTransFromTo(R0, T0, R1, T1, dR, dT);
	formEMat(I.data, t0.data, dR, dT, E);
}
double getMaxRepErr(const vector<Mat_d> & Ks, const vector<Mat_d>& Rs,
		const vector<Mat_d>& ts, const vector<vector<Meas2D> >& meas2d,
		const vector<Point3d>& pts3d) {
	assert(
			Ks.size() == Rs.size() && Rs.size() == ts.size() && meas2d.size() == pts3d.size());
	assert(pts3d.size() > 5);

	vector<double> vec_err;
	for (size_t i = 0; i < pts3d.size(); ++i) {
		for (size_t j = 0; j < meas2d[i].size(); ++j) {
			int viewid = meas2d[i][j].viewId;

			const double* K = Ks[viewid].data;
			const double* R = Rs[viewid].data;
			const double* t = ts[viewid].data;

			double m[2];
			project(K, R, t, pts3d[i].M, m);
			double e = dist2(m, meas2d[i][j].m);
			vec_err.push_back(e);
		}
	}
	partial_sort(vec_err.begin(), vec_err.begin() + 5, vec_err.end(),
			greater<double>());

	double s = 0;
	for (size_t i = 0; i < 5; ++i) {
		s += vec_err[i];
	}
	return s / 5;
}
