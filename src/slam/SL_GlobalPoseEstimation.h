/*
 * SL_GlobalPoseEstimation.h
 *
 *  Created on: 2011-6-11
 *      Author: tsou
 */

#ifndef SL_GLOBALPOSEESTIMATION_H_
#define SL_GLOBALPOSEESTIMATION_H_
#include "math/SL_Matrix.h"
#include <fstream>
using namespace std;
class CamPoseNode {
public:
	/*the global camera pose of camera #id*/
	int id;
	int frame;
	int camId;
	bool fixed;
	double R[9];
	double t[3];
	//to store the result
	double newR[9];
	double newt[3];
	bool constraint;
public:
	CamPoseNode() :
			id(-1), frame(-1), camId(-1), fixed(false), constraint(false) {
	}
	void set(const double RMat[9], const double tMat[3]) {
		memcpy(R, RMat, sizeof(double) * 9);
		memcpy(newR, RMat, sizeof(double) * 9);
		memcpy(t, tMat, sizeof(double) * 3);
		memcpy(newt, tMat, sizeof(double) * 3);
	}
	void set(int f, int cameraId, const double RMat[9], const double tMat[3]) {
		frame = f;
		camId = cameraId;
		set(RMat, tMat);
	}
};
class CamPoseEdge {
public:
	/*rigid transformation form camera #id1 to camera #id2*/
	int id1, id2;
	bool constraint;
	bool uncertainScale;
	double R[9];
	double t[3];
	double s; //scale
	int scaleId; //> 0 share unified scale with other edges
public:
	CamPoseEdge() :
			id1(-1), id2(-1), constraint(0), uncertainScale(false), s(0), scaleId(
					-1) {
	}
	void set(int nodeId1, int nodeId2, const double RMat[9],
			const double tMat[3]) {
		id1 = nodeId1;
		id2 = nodeId2;
		memcpy(R, RMat, sizeof(double) * 9);
		memcpy(t, tMat, sizeof(double) * 3);
		s = 0;
	}
	void set(int nodeId1, int nodeId2, const double RMat[9],
			const double tMat[3], int sId) {
		set(nodeId1, nodeId2, RMat, tMat);
		scaleId = sId;
	}
};

class GlobalPoseGraph {
public:
	GlobalPoseGraph();
	virtual ~GlobalPoseGraph();
public:
	int nNodes;
	CamPoseNode* poseNodes;
	int nEdges;
	CamPoseEdge* poseEdges;
	int nFixedNode;
	int nConstraintEdge;

	int nMaxNodes;
	int nMaxEdges;
	void reserve(int nMaxNodes, int nMaxEdges);
	void clear();

	CamPoseNode* newNode() {
		assert(nNodes < nMaxNodes);
		poseNodes[nNodes].id = nNodes;
		return &poseNodes[nNodes++];
	}
	CamPoseEdge* addEdge() {
		assert(nEdges < nMaxEdges);
		return &poseEdges[nEdges++];
	}

	void computeNewCameraRotations();
	void computeNewCameraTranslations();
	void computeNewCameraTranslations4();

	void getRConstraint(Mat_d& C);
	void computeNewCameraRotations2(const Mat_d& C);
	void getTConstraint(Mat_d& C);
	void computeNewCameraTranslations2(const Mat_d& C);

	void computeNewCameraRotations3(const Mat_d& C);
	void getTConstraint3(Mat_d& C, Mat_i& indScale, int& numScale);
	void computeNewCameraTranslations3(const Mat_d& C, Mat_i& indScale,
			int numScale);
public:
	void save(ofstream& file) const;
	void load(ifstream& file);
};

void write(const GlobalPoseGraph& posegraph, const char* fmtstr, ...);
void read(GlobalPoseGraph& posegraph, const char* fmtstr, ...);

#endif /* SL_GLOBALPOSEESTIMATION_H_ */
