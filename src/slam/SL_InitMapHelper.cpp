/*
 * SL_InitMapHelper.cpp
 *
 *  Created on: 2011-1-9
 *      Author: Danping Zou
 */

#include "SL_InitMapHelper.h"
#include "geometry/SL_Distortion.h"
#include <queue>

void selectCameraOrder(int camNum, const int* distMat, Mat_i& order) {
	int maxVal = 0;
	int iMax = -1, jMax = -1;
	const int* pDistMat = distMat;
	for (int i = 0; i < camNum; i++) {
		for (int j = i; j < camNum; j++) {
			if (pDistMat[j] > maxVal) {
				maxVal = pDistMat[j];
				iMax = i;
				jMax = j;
			}
		}
		pDistMat += camNum;
	}

	Mat_uc flag(camNum, 1);
	flag.fill(0);
	flag[iMax] = 1;
	flag[jMax] = 1;

	std::list<int> cams;
	cams.push_back(iMax);
	cams.push_back(jMax);

	while (maxVal > 0) {
		int iHead = cams.front();
		int iTail = cams.back();

		int iMaxHead = -1;
		int maxValHead = 0;
		pDistMat = distMat + camNum * iHead;
		for (int j = 0; j < camNum; j++) {
			if (flag[j] > 0)
				continue;
			else if (maxValHead < pDistMat[j]) {
				maxValHead = pDistMat[j];
				iMaxHead = j;
			}
		}

		int iMaxTail = -1;
		int maxValTail = 0;
		pDistMat = distMat + camNum * iTail;
		for (int j = 0; j < camNum; j++) {
			if (flag[j] > 0)
				continue;
			else if (maxValTail < pDistMat[j]) {
				maxValTail = pDistMat[j];
				iMaxTail = j;
			}
		}
		if (iMaxHead < 0 && iMaxTail < 0) {
			break;
		}

		if (maxValHead > maxValTail) {
			cams.push_front(iMaxHead);
			flag[iMaxHead] = 1;
		} else if (maxValHead < maxValTail) {
			cams.push_back(iMaxTail);
			flag[iMaxTail] = 1;
		}

	}

	order.resize(cams.size(), 1);

	int k = 0;
	std::list<int>::iterator iter;
	for (iter = cams.begin(); iter != cams.end(); iter++) {
		order.data[k++] = *iter;
	}
	order.rows = k;
	if (order.rows != camNum) {
		repErr("selectCameraOrder - not all camera have the intersection of views");
	}
}

int selectTracks(int numTracks, const Track2D featureTracks[], int lMin, std::vector<const Track2D*>& tracks) {
	int k = 0;
	for (int i = 0; i < numTracks; i++) {
		if (featureTracks[i].length() >= lMin) {
			tracks.push_back(featureTracks + i);
			k++;
		}
	}
	return k;
}
int selectTracksDuring(std::vector<const Track2D*>& vecTracks, int f1, int f2, std::vector<const Track2D*>& tracks) {
	int k = 0;
	for (size_t i = 0; i < vecTracks.size(); i++) {
		if (vecTracks[i]->f1 <= f1 && vecTracks[i]->f2 >= f2) {
			tracks.push_back(vecTracks[i]);
			k++;
		}
	}
	return k;
}
int getMatchingFromTracks(std::vector<const Track2D*>& vecPTracks, int f1, int f2, Mat_d& pts1, Mat_d& pts2) {
	if (f1 >= f2)
		repErr("getMatchingFrameTracks - f1 >= f2\n");
	pts1.clear();
	pts2.clear();
	size_t nTracks = vecPTracks.size();

	pts1.resize(nTracks, 2);
	pts2.resize(nTracks, 2);

	int k = 0;
	for (size_t i = 0; i < nTracks; i++) {
		Track2DNode* p = vecPTracks[i]->head.next;
		Track2DNode* tail = vecPTracks[i]->tail->next;
		bool bFoundPts1 = false;
		while (p != tail) {
			if (p->f == f1) {
				pts1.data[2 * k] = p->x;
				pts1.data[2 * k + 1] = p->y;
				bFoundPts1 = true;
			}
			if (bFoundPts1 && p->f == f2) {
				pts2.data[2 * k] = p->x;
				pts2.data[2 * k + 1] = p->y;
				k++;
				break; //found a matching in the track, jump to the next track
			}
			p = p->next;
		}
	}
	pts1.rows = k;
	pts2.rows = k;
	return k;
}
int getMatchingFromTracks(int nTracks, const Track2D* vecPTracks, int f1, int f2, Mat_d& pts1, Mat_d& pts2) {
	if (f1 >= f2)
		repErr("getMatchingFrameTracks - f1 >= f2\n");

	pts1.resize(nTracks, 2);
	pts2.resize(nTracks, 2);

	int k = 0;
	for (int i = 0; i < nTracks; i++) {
		Track2DNode* p = vecPTracks[i].head.next;
		Track2DNode* tail = vecPTracks[i].tail->next;
		bool bFoundPts1 = false;
		while (p != tail) {
			if (p->f == f1) {
				pts1.data[2 * k] = p->x;
				pts1.data[2 * k + 1] = p->y;
				bFoundPts1 = true;
			}
			if (bFoundPts1 && p->f == f2) {
				pts2.data[2 * k] = p->x;
				pts2.data[2 * k + 1] = p->y;
				k++;
				break; //found a matching in the track, jump to the next track
			}
			p = p->next;
		}
	}
	pts1.rows = k;
	pts2.rows = k;
	return k;
}
/* get points at frame f*/
void getPointsFromTracks(std::vector<const Track2D*>& vecPTracks, int f, Mat_d& pts) {
	pts.clear();
	size_t nTracks = vecPTracks.size();

	pts.resize(nTracks, 2);

	int k = 0;
	for (size_t i = 0; i < nTracks; i++) {
		Track2DNode* p = vecPTracks[i]->head.next;
		Track2DNode* tail = vecPTracks[i]->tail->next;
		while (p != tail) {
			if (p->f == f) {
				pts.data[2 * k] = p->x;
				pts.data[2 * k + 1] = p->y;
				k++;
				break;
			}
			p = p->next;
		}
	}
	pts.rows = k;
}
void getPointsFromTracks(std::vector<const Track2D*>& vecPTracks, int f, const Mat_uc& inlierFlag, Mat_d& pts) {
	pts.clear();
	size_t nTracks = vecPTracks.size();

	pts.resize(nTracks, 2);

	int k = 0;
	for (size_t i = 0; i < nTracks; i++) {
		if (inlierFlag[i] == 0)
			continue;
		Track2DNode* p = vecPTracks[i]->head.next;
		Track2DNode* tail = vecPTracks[i]->tail->next;
		while (p != tail) {
			if (p->f == f) {
				pts.data[2 * k] = p->x;
				pts.data[2 * k + 1] = p->y;
				k++;
				break;
			}
			p = p->next;
		}
	}
	pts.rows = k;
}

void TracksToMeasurments(const double invK[][9], std::vector<const Track2D*> vecPTracks, int nviews, Mat_d& meas, Mat_c& vmask) {

	int nTrack = vecPTracks.size();
	int nMeas = 0;
	for (int i = 0; i < nTrack; i++) {
		nMeas += vecPTracks[i]->length();
	}

	meas.resize(nMeas, 2);
	vmask.resize(nTrack, nviews);
	vmask.fill(0);

	double tmpPt[2];
	int k = 0;
	for (int i = 0; i < nTrack; i++) {
		Track2DNode* p = vecPTracks[i]->head.next;
		Track2DNode* tail = vecPTracks[i]->tail->next;
		while (p != tail) {
			int f = p->f;
			tmpPt[0] = p->x;
			tmpPt[1] = p->y;
			normPoint(invK[f], tmpPt, meas + 2 * k);
			vmask.data[i * nviews + f] = true;
			k++;
			p = p->next;
		}
	}
}
void getMatchIndices(Matching& matches, int maxInd, Mat_i& ind, bool inverse) {
	ind.resize(maxInd, 1);
	ind.fill(-1);
	if (!inverse) {
		for (int i = 0; i < matches.num; i++) {
			int idx1 = matches[i].idx1;
			int idx2 = matches[i].idx2;
			ind.data[idx1] = idx2;
		}
	} else {
		for (int i = 0; i < matches.num; i++) {
			int idx1 = matches[i].idx1;
			int idx2 = matches[i].idx2;
			ind.data[idx2] = idx1;
		}
	}
}
//test
//#include "SL_Debug.h"
//int main(){
//	Mat_i camMat(4,4);
//	camMat.fill(-1);
//	camMat(0,1) = 266;
//	camMat(0,2) = 313;
//	camMat(0,3) = 284;
//	camMat(1,2) = 300;
//	camMat(1,3) = 279;
//	camMat(2,3) = 320;
//	camMat(1,0) = camMat(0,1);
//	camMat(2,0) = camMat(0,2);
//	camMat(3,0) = camMat(0,3);
//	camMat(2,1) = camMat(1,2);
//	camMat(3,1) = camMat(1,3);
//	camMat(3,2) = camMat(2,3);
//	
//	print(camMat);
//	Mat_i order;
//	selectCameraOrder(4,camMat,order);
//	print(order);
//	return 0;
//}
