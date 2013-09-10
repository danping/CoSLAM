/*
 * SL_InterCamPoseEstimator.cpp
 *
 *  Created on: 2011-7-26
 *      Author: zou
 */

#include "SL_InterCamPoseEstimator.h"
#include "slam/SL_IntraCamPoseEstimator.h"
#include "SL_GlobParam.h"
#include "geometry/SL_Quaternion.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"

#include <cassert>
#include <map>

void InterCamPoseEstimator::addMapPoints() {
	using namespace std;
	int numCams = m_pCoSLAM->numCams;

	vector<FeaturePoint*> vecStaticFeatPts[SLAM_MAX_NUM];
	vector<FeaturePoint*> vecDynamicFeatPts[SLAM_MAX_NUM];
	for (int c = 0; c < numCams; c++) {
		m_pCoSLAM->slam[c].chooseStaticFeatPts(vecStaticFeatPts[c]);
		m_pCoSLAM->slam[c].chooseDynamicFeatPts(vecDynamicFeatPts[c]);
	}

	//set initial camera poses
	Ks.clear();
	Rs.clear();
	Ts.clear();
	for (int c = 0; c < numCams; c++) {
		Ks.push_back(Mat_d(3, 3, m_pCoSLAM->slam[c].K.data));
		Rs.push_back(Mat_d(3, 3, m_pCoSLAM->slam[c].m_camPos.current()->R));
		Ts.push_back(Mat_d(3, 1, m_pCoSLAM->slam[c].m_camPos.current()->t));
	}

	//add static points
	vecPts3D.clear();
	vecMeas2D.clear();
	for (int c = 0; c < numCams; c++) {
		for (size_t i = 0; i < vecStaticFeatPts[c].size(); i++) {
			FeaturePoint* fp = vecStaticFeatPts[c][i];
			if (!fp->mpt)
				continue;
			vecPts3D.push_back(Point3d(fp->mpt->x, fp->mpt->y, fp->mpt->z));
			vecMeas2D.push_back(vector<Meas2D>());
			vecMeas2D.back().push_back(Meas2D(c, fp->x, fp->y));
		}
	}

	m_numStatic = vecPts3D.size();

	//add dynamic points
	map<MapPoint*, int> mapDynPts;

	for (int c = 0; c < numCams; c++) {
		for (size_t i = 0; i < vecDynamicFeatPts[c].size(); i++) {
			FeaturePoint* fp = vecDynamicFeatPts[c][i];
			mapDynPts[fp->mpt]++;
		}
	}

	const int maxDyn = 60;
	int k = 0;
	int curFrame = m_pCoSLAM->curFrame;
	for (map<MapPoint*, int>::iterator iter = mapDynPts.begin(); iter != mapDynPts.end(); iter++) {
		MapPoint* mpt = iter->first;
//		if( mpt->numVisCam != numCams)
//			continue;

		if( k > maxDyn)
			continue;

		vecPts3D.push_back(Point3d(mpt->x, mpt->y, mpt->z));
		vecMeas2D.push_back(vector<Meas2D>());
		for (int c = 0; c < numCams; c++) {
			FeaturePoint* fp = mpt->pFeatures[c];
			if (fp && fp->f == curFrame) {
				vecMeas2D.back().push_back(Meas2D(c, fp->x, fp->y));
			}
		}
		k++;
	}

	m_numDynamic = vecPts3D.size() - m_numStatic;

	//test
	printf("static:%d dynamic:%d\n", m_numStatic, m_numDynamic);
}
void InterCamPoseEstimator::apply() {
	assert(m_numStatic > 0);
	//call robust bundle adjustment
	bundleAdjustRobust(0, Ks, Rs, Ts, m_numStatic, vecPts3D, vecMeas2D, sigma, maxIter, maxIterEachStep);

	int numCams = m_pCoSLAM->numCams;
	int curFrame = m_pCoSLAM->curFrame;
	for (int c = 0; c < numCams; c++) {
		CamPoseItem* camPos = m_pCoSLAM->slam[c].m_camPos.add(curFrame, c, Rs[c].data, Ts[c].data);
		m_pCoSLAM->slam[c].updateCamParamForFeatPts(m_pCoSLAM->slam[c].K.data, camPos);
	}

	int numOut = 0;
	for (int c = 0; c < numCams; c++) {
		std::vector<Track2DNode*> nodes;
		int num = m_pCoSLAM->slam[c].getStaticMappedTrackNodes(nodes);

		double rm[2], var[4], ivar[4];
		for (int i = 0; i < num; i++) {
			double* pM = nodes[i]->pt->mpt->M;
			double* pCov = nodes[i]->pt->mpt->cov;
			project(Ks[c], Rs[c], Ts[c], pM, rm);
			getProjectionCovMat(Ks[c], Rs[c], Ts[c], pM, pCov, var, Const::PIXEL_ERR_VAR);
			mat22Inv(var, ivar);
			double err = mahaDist2(rm, nodes[i]->pt->m, ivar);
			if (err < 2) { //inlier
				nodes[i]->pt->reprojErr = err;
				seqTriangulate(Ks[c], Rs[c], Ts[c], nodes[i]->pt->m, pM, pCov, Const::PIXEL_ERR_VAR);
				project(Ks[c], Rs[c], Ts[c], pM, rm);
				getProjectionCovMat(Ks[c], Rs[c], Ts[c], pM, pCov, var, Const::PIXEL_ERR_VAR);
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
	}

//	for (int j = 0; j < meas_vec.rows; j++) {
//		int mapId = measMapInd[j];
//		int camId = measViewInd[j];
//		double* M = mapPts.data + 3 * mapId;
//		double* m = meas_vec.data + 2 * j;
//
//		double rm[2], var[4], ivar[4];
//		//compute re-projection error
//		project(Ks.data + 9 * camId, Rs.data + 9 * camId, Ts.data + 3 * camId, M, rm);
//		getProjectionCovMat(Ks.data + 9 * camId, Rs.data + 9 * camId, Ts.data + 3 * camId, M, measFeat[j]->mpt->cov, var, GlobParam::PIXEL_ERR_VAR);
//		mat22Inv(var, ivar);
//		double err = mahaDist2(rm, m, ivar);
//		if (err > 1.2) {
//			//outlier
//			measFeat[j]->reprojErr = err;
//			if (measFeat[j]->mpt->isLocalStatic()) {
//				measFeat[j]->mpt->setUncertain();
//				m_numStatic--;
//			} else {
//				measFeat[j]->mpt->setFalse();
//				measFeat[j]->reprojErr = err;
//			}
//		} else {
//			double* pM = measFeat[j]->mpt->data;
//			double* pCov = measFeat[j]->mpt->cov;
//			seqTriangulate(Ks.data + 9 * camId, Rs.data + 9 * camId, Ts.data + 3 * camId, measFeat[j]->m, pM, pCov, GlobParam::PIXEL_ERR_VAR);
//		}
//	}
}
