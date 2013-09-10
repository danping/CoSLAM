/*
 * SL_GlobalPoseEstimation.cpp
 *
 *  Created on: 2011-6-11
 *      Author: tsou
 */

#include "SL_Define.h"
#include "SL_error.h"
#include "SL_GlobalPoseEstimation.h"

#include "math/SL_LinAlgWarper.h"
#include "math/SL_SparseMat.h"
#include "math/SL_SparseLinearSystem.h"
#include "geometry/SL_RigidTransform.h"
#include <vector>

GlobalPoseGraph::GlobalPoseGraph() :
		nNodes(0), poseNodes(0), nEdges(0), poseEdges(0), nFixedNode(0) {
	// TODO Auto-generated constructor stub
}

GlobalPoseGraph::~GlobalPoseGraph() {
	clear();
}
void GlobalPoseGraph::reserve(int numMaxNodes, int numMaxEdges) {
	clear();
	nMaxNodes = numMaxNodes;
	nMaxEdges = numMaxEdges;

	poseNodes = new CamPoseNode[nMaxNodes];
	poseEdges = new CamPoseEdge[nMaxEdges];
}
void GlobalPoseGraph::clear() {
	if (poseNodes)
		delete[] poseNodes;
	if (poseEdges)
		delete[] poseEdges;

	poseNodes = 0;
	poseEdges = 0;

	nNodes = 0;
	nEdges = 0;

	nFixedNode = 0;
	nConstraintEdge = 0;

	nMaxNodes = 0;
	nMaxEdges = 0;
}
void GlobalPoseGraph::computeNewCameraRotations() {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		int i = poseEdges[k].id1;
		int j = poseEdges[k].id2;
		if (!poseNodes[i].fixed || !poseNodes[j].fixed)
			indR.data[k] = nValidEdge++;
	}

	Mat_i indC(nNodes, 1);
	indC.fill(-1);
	int nValidNode = 0;
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed) {
			indC.data[k] = nValidNode++;
		}
	}
	Triplets T;
	int m = 9 * nValidEdge; //number of constraints
	int n = 9 * nValidNode; //number of unknowns
	int nnz = nValidEdge * 36;

	double* b = new double[m];
	double* x = new double[n];
	T.reserve(m, n, nnz);

	memset(b, 0, sizeof(double) * m);

	for (int i = 0; i < nEdges; i++) {
		if (indR.data[i] < 0)
			continue;

		int id1 = poseEdges[i].id1;
		int id2 = poseEdges[i].id2;

		double* R = poseEdges[i].R;

		int r = indR.data[i];
		int c1 = indC.data[id1];
		int c2 = indC.data[id2];

		if (!poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			T.add(9 * r, 9 * c2, 1.0);
			T.add(9 * r + 1, 9 * c2 + 1, 1.0);
			T.add(9 * r + 2, 9 * c2 + 2, 1.0);
			T.add(9 * r + 3, 9 * c2 + 3, 1.0);
			T.add(9 * r + 4, 9 * c2 + 4, 1.0);
			T.add(9 * r + 5, 9 * c2 + 5, 1.0);
			T.add(9 * r + 6, 9 * c2 + 6, 1.0);
			T.add(9 * r + 7, 9 * c2 + 7, 1.0);
			T.add(9 * r + 8, 9 * c2 + 8, 1.0);

			T.add(9 * r, 9 * c1, -R[0]);
			T.add(9 * r, 9 * c1 + 1, -R[1]);
			T.add(9 * r, 9 * c1 + 2, -R[2]);
			T.add(9 * r + 1, 9 * c1, -R[3]);
			T.add(9 * r + 1, 9 * c1 + 1, -R[4]);
			T.add(9 * r + 1, 9 * c1 + 2, -R[5]);
			T.add(9 * r + 2, 9 * c1, -R[6]);
			T.add(9 * r + 2, 9 * c1 + 1, -R[7]);
			T.add(9 * r + 2, 9 * c1 + 2, -R[8]);

			T.add(9 * r + 3, 9 * c1 + 3, -R[0]);
			T.add(9 * r + 3, 9 * c1 + 4, -R[1]);
			T.add(9 * r + 3, 9 * c1 + 5, -R[2]);
			T.add(9 * r + 4, 9 * c1 + 3, -R[3]);
			T.add(9 * r + 4, 9 * c1 + 4, -R[4]);
			T.add(9 * r + 4, 9 * c1 + 5, -R[5]);
			T.add(9 * r + 5, 9 * c1 + 3, -R[6]);
			T.add(9 * r + 5, 9 * c1 + 4, -R[7]);
			T.add(9 * r + 5, 9 * c1 + 5, -R[8]);

			T.add(9 * r + 6, 9 * c1 + 6, -R[0]);
			T.add(9 * r + 6, 9 * c1 + 7, -R[1]);
			T.add(9 * r + 6, 9 * c1 + 8, -R[2]);
			T.add(9 * r + 7, 9 * c1 + 6, -R[3]);
			T.add(9 * r + 7, 9 * c1 + 7, -R[4]);
			T.add(9 * r + 7, 9 * c1 + 8, -R[5]);
			T.add(9 * r + 8, 9 * c1 + 6, -R[6]);
			T.add(9 * r + 8, 9 * c1 + 7, -R[7]);
			T.add(9 * r + 8, 9 * c1 + 8, -R[8]);

			memset(b + 9 * r, 0, sizeof(double) * 9);
		} else if (poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			T.add(9 * r, 9 * c2, 1.0);
			T.add(9 * r + 1, 9 * c2 + 1, 1.0);
			T.add(9 * r + 2, 9 * c2 + 2, 1.0);
			T.add(9 * r + 3, 9 * c2 + 3, 1.0);
			T.add(9 * r + 4, 9 * c2 + 4, 1.0);
			T.add(9 * r + 5, 9 * c2 + 5, 1.0);
			T.add(9 * r + 6, 9 * c2 + 6, 1.0);
			T.add(9 * r + 7, 9 * c2 + 7, 1.0);
			T.add(9 * r + 8, 9 * c2 + 8, 1.0);

			double tmp[9];
			mat33AB(R, poseNodes[id1].R, tmp);
			mat33Trans(tmp, b + 9 * r);

		} else if (!poseNodes[id1].fixed && poseNodes[id2].fixed) {
			T.add(9 * r, 9 * c1, -R[0]);
			T.add(9 * r, 9 * c1 + 1, -R[1]);
			T.add(9 * r, 9 * c1 + 2, -R[2]);
			T.add(9 * r + 1, 9 * c1, -R[3]);
			T.add(9 * r + 1, 9 * c1 + 1, -R[4]);
			T.add(9 * r + 1, 9 * c1 + 2, -R[5]);
			T.add(9 * r + 2, 9 * c1, -R[6]);
			T.add(9 * r + 2, 9 * c1 + 1, -R[7]);
			T.add(9 * r + 2, 9 * c1 + 2, -R[8]);

			T.add(9 * r + 3, 9 * c1 + 3, -R[0]);
			T.add(9 * r + 3, 9 * c1 + 4, -R[1]);
			T.add(9 * r + 3, 9 * c1 + 5, -R[2]);
			T.add(9 * r + 4, 9 * c1 + 3, -R[3]);
			T.add(9 * r + 4, 9 * c1 + 4, -R[4]);
			T.add(9 * r + 4, 9 * c1 + 5, -R[5]);
			T.add(9 * r + 5, 9 * c1 + 3, -R[6]);
			T.add(9 * r + 5, 9 * c1 + 4, -R[7]);
			T.add(9 * r + 5, 9 * c1 + 5, -R[8]);

			T.add(9 * r + 6, 9 * c1 + 6, -R[0]);
			T.add(9 * r + 6, 9 * c1 + 7, -R[1]);
			T.add(9 * r + 6, 9 * c1 + 8, -R[2]);
			T.add(9 * r + 7, 9 * c1 + 6, -R[3]);
			T.add(9 * r + 7, 9 * c1 + 7, -R[4]);
			T.add(9 * r + 7, 9 * c1 + 8, -R[5]);
			T.add(9 * r + 8, 9 * c1 + 6, -R[6]);
			T.add(9 * r + 8, 9 * c1 + 7, -R[7]);
			T.add(9 * r + 8, 9 * c1 + 8, -R[8]);

			mat33Trans(poseNodes[id2].R, b + 9 * r);

			b[9 * r] = -b[9 * r];
			b[9 * r + 1] = -b[9 * r + 1];
			b[9 * r + 2] = -b[9 * r + 2];
			b[9 * r + 3] = -b[9 * r + 3];
			b[9 * r + 4] = -b[9 * r + 4];
			b[9 * r + 5] = -b[9 * r + 5];
			b[9 * r + 6] = -b[9 * r + 6];
			b[9 * r + 7] = -b[9 * r + 7];
			b[9 * r + 8] = -b[9 * r + 8];
		}
	}

	sparseSolveLin(T, b, x);
	
//	//test
//	write(T,"/home/tsou/T.txt");
//	writeMat(m,1,b,"/home/tsou/b.txt");
//	writeMat(n,1,x,"/home/tsou/x.txt");

	//get the approximated rotations
	for (int k = 0; k < nNodes; k++) {
		int c = indC.data[k];
		if (c >= 0) {
			double tmp[9];
			mat33Trans(x + 9 * c, tmp);
			approxRotationMat(tmp, poseNodes[k].newR);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		} else {
			memcpy(poseNodes[k].newR, poseNodes[k].R, sizeof(double) * 9);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		}
	}
	delete[] b;
	delete[] x;
}
void GlobalPoseGraph::computeNewCameraTranslations() {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		int i = poseEdges[k].id1;
		int j = poseEdges[k].id2;
		if (!poseNodes[i].fixed || !poseNodes[j].fixed)
			indR.data[k] = nValidEdge++;
	}

	Mat_i indC(nNodes, 1);
	indC.fill(-1);
	int nValidNode = 0;
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed) {
			indC.data[k] = nValidNode++;
		}
	}

	Triplets T;
	int m = 3 * nValidEdge;
	int n = 3 * nValidNode + nConstraintEdge;
	int nnz = 2 * (nValidEdge * 12 + nConstraintEdge * 3);

	double* b = new double[m];
	double* x = new double[n];
	T.reserve(m, n, nnz);

	memset(b, 0, sizeof(double) * m);

	int nScale = 0;
	for (int k = 0; k < nEdges; k++) {
		if (indR.data[k] < 0)
			continue;

		int id1 = poseEdges[k].id1;
		int id2 = poseEdges[k].id2;
		double* t = poseEdges[k].t;
		double* R = poseEdges[k].R;

		int i = indC.data[id1];
		int j = indC.data[id2];
		int r = indR.data[k];

		//valid edge gives a constraint
		if (!poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			//Tj - R_{ij} T_i = T_{ij}
			//fill b
			T.add(3 * r, 3 * j, 1.0);
			T.add(3 * r + 1, 3 * j + 1, 1.0);
			T.add(3 * r + 2, 3 * j + 2, 1.0);

			T.add(3 * r, 3 * i, -R[0]);
			T.add(3 * r, 3 * i + 1, -R[1]);
			T.add(3 * r, 3 * i + 2, -R[2]);
			T.add(3 * r + 1, 3 * i, -R[3]);
			T.add(3 * r + 1, 3 * i + 1, -R[4]);
			T.add(3 * r + 1, 3 * i + 2, -R[5]);
			T.add(3 * r + 2, 3 * i, -R[6]);
			T.add(3 * r + 2, 3 * i + 1, -R[7]);
			T.add(3 * r + 2, 3 * i + 2, -R[8]);

			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + nScale, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + nScale, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + nScale, -t[2]);
			} else {
				memcpy(b + 3 * r, t, sizeof(double) * 3);
			}
		} else if (!poseNodes[id1].fixed && poseNodes[id2].fixed) {
			//-R_{ij}T_i = T_{ij} - T_j

			T.add(3 * r, 3 * i, -R[0]);
			T.add(3 * r, 3 * i + 1, -R[1]);
			T.add(3 * r, 3 * i + 2, -R[2]);
			T.add(3 * r + 1, 3 * i, -R[3]);
			T.add(3 * r + 1, 3 * i + 1, -R[4]);
			T.add(3 * r + 1, 3 * i + 2, -R[5]);
			T.add(3 * r + 2, 3 * i, -R[6]);
			T.add(3 * r + 2, 3 * i + 1, -R[7]);
			T.add(3 * r + 2, 3 * i + 2, -R[8]);

			double* tj = poseNodes[id2].t;
			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + nScale, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + nScale, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + nScale, -t[2]);

				b[3 * r] = -tj[0];
				b[3 * r + 1] = -tj[1];
				b[3 * r + 2] = -tj[2];
			} else {
				b[3 * r] = t[0] - tj[0];
				b[3 * r + 1] = t[1] - tj[1];
				b[3 * r + 2] = t[2] - tj[2];
			}
		} else if (poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			//T_j = T_{ij} + R_{ij} T_i
			T.add(3 * r, 3 * j, 1.0);
			T.add(3 * r + 1, 3 * j + 1, 1.0);
			T.add(3 * r + 2, 3 * j + 2, 1.0);

			double* ti = poseNodes[id1].t;
			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + nScale, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + nScale, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + nScale, -t[2]);
				mat33AB(R, ti, b + 3 * r);
			} else {
				mat33ProdVec(R, ti, t, b + 3 * r, 1.0, 1.0);
			}
		}
		if (poseEdges[k].uncertainScale)
			nScale++;
	}

	sparseSolveLin(T, b, x);

	//write back 
	for (int k = 0; k < nNodes; k++) {
		int c = indC.data[k];
		if (c < 0) {
			memcpy(poseNodes[k].newR, poseNodes[k].R, sizeof(double) * 9);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		} else {
			memcpy(poseNodes[k].newt, x + 3 * c, sizeof(double) * 3);
		}
	}

	int num = 0;
	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].uncertainScale) {
			poseEdges[k].s = x[3 * nValidNode + num];
			num++;
		}
	}
	delete[] b;
	delete[] x;
}

void GlobalPoseGraph::computeNewCameraTranslations4() {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		int i = poseEdges[k].id1;
		int j = poseEdges[k].id2;
		if (!poseNodes[i].fixed || !poseNodes[j].fixed)
			indR.data[k] = nValidEdge++;
	}

	Mat_i indC(nNodes, 1);
	indC.fill(-1);
	int nValidNode = 0;
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed) {
			indC.data[k] = nValidNode++;
		}
	}

	//////////////////////////////////////////////////
	//get the number of unified scale
	Mat_i sIdFlag(nEdges, 1);
	sIdFlag.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].constraint) {
			int sId = poseEdges[k].scaleId;
			assert(sId >= 0);
			sIdFlag.data[sId] = sId;
		}
	}

	Mat_i indScale(nEdges, 1);
	indScale.fill(-1);

	int numScale = 0;
	for (int k = 0; k < nEdges; k++) {
		int sId = sIdFlag.data[k];
		if (sId >= 0) {
			indScale.data[sId] = numScale;
			numScale++;
		}
	}
	////////////////////////////////////////////////

	Triplets T;
	int m = 3 * nValidEdge;
	int n = 3 * nValidNode + numScale;
	int nnz = 2 * (nValidEdge * 12 + nConstraintEdge * 3);

	double* b = new double[m];
	double* x = new double[n];
	T.reserve(m, n, nnz);

	memset(b, 0, sizeof(double) * m);

	for (int k = 0; k < nEdges; k++) {
		if (indR.data[k] < 0)
			continue;

		int id1 = poseEdges[k].id1;
		int id2 = poseEdges[k].id2;
		double* t = poseEdges[k].t;
		double* R = poseEdges[k].R;
		int sId = poseEdges[k].scaleId;

		int i = indC.data[id1];
		int j = indC.data[id2];
		int c = indScale.data[sId];
		int r = indR.data[k];

		//valid edge gives a constraint
		if (!poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			//Tj - R_{ij} T_i = T_{ij}
			//fill b
			T.add(3 * r, 3 * j, 1.0);
			T.add(3 * r + 1, 3 * j + 1, 1.0);
			T.add(3 * r + 2, 3 * j + 2, 1.0);

			T.add(3 * r, 3 * i, -R[0]);
			T.add(3 * r, 3 * i + 1, -R[1]);
			T.add(3 * r, 3 * i + 2, -R[2]);
			T.add(3 * r + 1, 3 * i, -R[3]);
			T.add(3 * r + 1, 3 * i + 1, -R[4]);
			T.add(3 * r + 1, 3 * i + 2, -R[5]);
			T.add(3 * r + 2, 3 * i, -R[6]);
			T.add(3 * r + 2, 3 * i + 1, -R[7]);
			T.add(3 * r + 2, 3 * i + 2, -R[8]);

			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + c, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + c, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + c, -t[2]);
			} else {
				memcpy(b + 3 * r, t, sizeof(double) * 3);
			}
		} else if (!poseNodes[id1].fixed && poseNodes[id2].fixed) {
			//-R_{ij}T_i = T_{ij} - T_j

			T.add(3 * r, 3 * i, -R[0]);
			T.add(3 * r, 3 * i + 1, -R[1]);
			T.add(3 * r, 3 * i + 2, -R[2]);
			T.add(3 * r + 1, 3 * i, -R[3]);
			T.add(3 * r + 1, 3 * i + 1, -R[4]);
			T.add(3 * r + 1, 3 * i + 2, -R[5]);
			T.add(3 * r + 2, 3 * i, -R[6]);
			T.add(3 * r + 2, 3 * i + 1, -R[7]);
			T.add(3 * r + 2, 3 * i + 2, -R[8]);

			double* tj = poseNodes[id2].t;
			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + c, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + c, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + c, -t[2]);

				b[3 * r] = -tj[0];
				b[3 * r + 1] = -tj[1];
				b[3 * r + 2] = -tj[2];
			} else {
				b[3 * r] = t[0] - tj[0];
				b[3 * r + 1] = t[1] - tj[1];
				b[3 * r + 2] = t[2] - tj[2];
			}
		} else if (poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			//T_j = T_{ij} + R_{ij} T_i
			T.add(3 * r, 3 * j, 1.0);
			T.add(3 * r + 1, 3 * j + 1, 1.0);
			T.add(3 * r + 2, 3 * j + 2, 1.0);

			double* ti = poseNodes[id1].t;
			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + c, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + c, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + c, -t[2]);
				mat33AB(R, ti, b + 3 * r);
			} else {
				mat33ProdVec(R, ti, t, b + 3 * r, 1.0, 1.0);
			}
		}
	}

	sparseSolveLin(T, b, x);

	//write back 
	for (int k = 0; k < nNodes; k++) {
		int c = indC.data[k];
		if (c < 0) {
			memcpy(poseNodes[k].newR, poseNodes[k].R, sizeof(double) * 9);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		} else {
			memcpy(poseNodes[k].newt, x + 3 * c, sizeof(double) * 3);
		}
	}

	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].uncertainScale) {
			int sId = poseEdges[k].scaleId;
			if (sId < 0)
				continue;
			poseEdges[k].s = x[3 * nValidNode + indScale.data[sId]];
		}
	}
	delete[] b;
	delete[] x;
}

//#include "SL_Print.h"
//#include "SL_WriteRead.h"
void GlobalPoseGraph::getRConstraint(Mat_d& C) {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].constraint)
			indR.data[k] = nValidEdge++;
	}
	int nValidNode = 0;
	Mat_i indC(nNodes, 1);
	indC.fill(-1);
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed && poseNodes[k].constraint)
			indC.data[k] = nValidNode++;
	}

	C.resize(nValidEdge * 9, nValidNode * 9);
	C.fill(0);
	for (int i = 0; i < nEdges; i++) {
		if (indR.data[i] < 0)
			continue;

		int id1 = poseEdges[i].id1;
		int id2 = poseEdges[i].id2;

		int r = indR.data[i];
		int c1 = indC.data[id1];
		int c2 = indC.data[id2];
		assert(c1 >= 0 && c2 >= 0);

		double* R = poseEdges[i].R;

		C(9 * r, 9 * c2) = 1.0;
		C(9 * r + 1, 9 * c2 + 1) = 1.0;
		C(9 * r + 2, 9 * c2 + 2) = 1.0;
		C(9 * r + 3, 9 * c2 + 3) = 1.0;
		C(9 * r + 4, 9 * c2 + 4) = 1.0;
		C(9 * r + 5, 9 * c2 + 5) = 1.0;
		C(9 * r + 6, 9 * c2 + 6) = 1.0;
		C(9 * r + 7, 9 * c2 + 7) = 1.0;
		C(9 * r + 8, 9 * c2 + 8) = 1.0;

		C(9 * r, 9 * c1) = -R[0];
		C(9 * r, 9 * c1 + 1) = -R[1];
		C(9 * r, 9 * c1 + 2) = -R[2];
		C(9 * r + 1, 9 * c1) = -R[3];
		C(9 * r + 1, 9 * c1 + 1) = -R[4];
		C(9 * r + 1, 9 * c1 + 2) = -R[5];
		C(9 * r + 2, 9 * c1) = -R[6];
		C(9 * r + 2, 9 * c1 + 1) = -R[7];
		C(9 * r + 2, 9 * c1 + 2) = -R[8];

		C(9 * r + 3, 9 * c1 + 3) = -R[0];
		C(9 * r + 3, 9 * c1 + 4) = -R[1];
		C(9 * r + 3, 9 * c1 + 5) = -R[2];
		C(9 * r + 4, 9 * c1 + 3) = -R[3];
		C(9 * r + 4, 9 * c1 + 4) = -R[4];
		C(9 * r + 4, 9 * c1 + 5) = -R[5];
		C(9 * r + 5, 9 * c1 + 3) = -R[6];
		C(9 * r + 5, 9 * c1 + 4) = -R[7];
		C(9 * r + 5, 9 * c1 + 5) = -R[8];

		C(9 * r + 6, 9 * c1 + 6) = -R[0];
		C(9 * r + 6, 9 * c1 + 7) = -R[1];
		C(9 * r + 6, 9 * c1 + 8) = -R[2];
		C(9 * r + 7, 9 * c1 + 6) = -R[3];
		C(9 * r + 7, 9 * c1 + 7) = -R[4];
		C(9 * r + 7, 9 * c1 + 8) = -R[5];
		C(9 * r + 8, 9 * c1 + 6) = -R[6];
		C(9 * r + 8, 9 * c1 + 7) = -R[7];
		C(9 * r + 8, 9 * c1 + 8) = -R[8];
	}
}
void GlobalPoseGraph::computeNewCameraRotations2(const Mat_d& C) {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		int i = poseEdges[k].id1;
		int j = poseEdges[k].id2;
		//skip the constraint edges
		if (!poseEdges[k].constraint
				&& (!poseNodes[i].fixed || !poseNodes[j].fixed)) {
			indR.data[k] = nValidEdge++;
		}
	}

	Mat_i indC(nNodes, 1);
	int nValidNode = 0, nUnknownNode = 0, nConstraintNode = 0;
	indC.fill(-1);

	std::vector<int> unfixedIds, constraintIds;
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed) {
			if (!poseNodes[k].constraint) {
				unfixedIds.push_back(k);
				nUnknownNode++;
			} else {
				constraintIds.push_back(k);
				nConstraintNode++;
			}
			nValidNode++;
		}
	}

	for (int i = 0; i < nUnknownNode; i++) {
		indC.data[unfixedIds[i]] = i;
	}
	for (int i = 0; i < nConstraintNode; i++) {
		indC.data[constraintIds[i]] = i + nUnknownNode;
	}

	Triplets T;
	int m = 9 * nValidEdge; //number of constraints
	int n = 9 * nValidNode; //number of unknowns
	int nnz = nValidEdge * 36;
	T.reserve(m, n, nnz);

	Mat_d b(m, 1);
	b.fill(0);

	//fill T
	for (int i = 0; i < nEdges; i++) {
		if (indR.data[i] < 0)
			continue;

		int id1 = poseEdges[i].id1;
		int id2 = poseEdges[i].id2;

		int r = indR.data[i];
		int c1 = indC.data[id1];
		int c2 = indC.data[id2];
		if (c1 < 0 && c2 < 0)
			continue;

		double* R = poseEdges[i].R;

		if (!poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			T.add(9 * r, 9 * c2, 1.0);
			T.add(9 * r + 1, 9 * c2 + 1, 1.0);
			T.add(9 * r + 2, 9 * c2 + 2, 1.0);
			T.add(9 * r + 3, 9 * c2 + 3, 1.0);
			T.add(9 * r + 4, 9 * c2 + 4, 1.0);
			T.add(9 * r + 5, 9 * c2 + 5, 1.0);
			T.add(9 * r + 6, 9 * c2 + 6, 1.0);
			T.add(9 * r + 7, 9 * c2 + 7, 1.0);
			T.add(9 * r + 8, 9 * c2 + 8, 1.0);

			T.add(9 * r, 9 * c1, -R[0]);
			T.add(9 * r, 9 * c1 + 1, -R[1]);
			T.add(9 * r, 9 * c1 + 2, -R[2]);
			T.add(9 * r + 1, 9 * c1, -R[3]);
			T.add(9 * r + 1, 9 * c1 + 1, -R[4]);
			T.add(9 * r + 1, 9 * c1 + 2, -R[5]);
			T.add(9 * r + 2, 9 * c1, -R[6]);
			T.add(9 * r + 2, 9 * c1 + 1, -R[7]);
			T.add(9 * r + 2, 9 * c1 + 2, -R[8]);

			T.add(9 * r + 3, 9 * c1 + 3, -R[0]);
			T.add(9 * r + 3, 9 * c1 + 4, -R[1]);
			T.add(9 * r + 3, 9 * c1 + 5, -R[2]);
			T.add(9 * r + 4, 9 * c1 + 3, -R[3]);
			T.add(9 * r + 4, 9 * c1 + 4, -R[4]);
			T.add(9 * r + 4, 9 * c1 + 5, -R[5]);
			T.add(9 * r + 5, 9 * c1 + 3, -R[6]);
			T.add(9 * r + 5, 9 * c1 + 4, -R[7]);
			T.add(9 * r + 5, 9 * c1 + 5, -R[8]);

			T.add(9 * r + 6, 9 * c1 + 6, -R[0]);
			T.add(9 * r + 6, 9 * c1 + 7, -R[1]);
			T.add(9 * r + 6, 9 * c1 + 8, -R[2]);
			T.add(9 * r + 7, 9 * c1 + 6, -R[3]);
			T.add(9 * r + 7, 9 * c1 + 7, -R[4]);
			T.add(9 * r + 7, 9 * c1 + 8, -R[5]);
			T.add(9 * r + 8, 9 * c1 + 6, -R[6]);
			T.add(9 * r + 8, 9 * c1 + 7, -R[7]);
			T.add(9 * r + 8, 9 * c1 + 8, -R[8]);

			memset(b.data + 9 * r, 0, sizeof(double) * 9);
		} else if (poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			T.add(9 * r, 9 * c2, 1.0);
			T.add(9 * r + 1, 9 * c2 + 1, 1.0);
			T.add(9 * r + 2, 9 * c2 + 2, 1.0);
			T.add(9 * r + 3, 9 * c2 + 3, 1.0);
			T.add(9 * r + 4, 9 * c2 + 4, 1.0);
			T.add(9 * r + 5, 9 * c2 + 5, 1.0);
			T.add(9 * r + 6, 9 * c2 + 6, 1.0);
			T.add(9 * r + 7, 9 * c2 + 7, 1.0);
			T.add(9 * r + 8, 9 * c2 + 8, 1.0);

			double tmp[9];
			mat33AB(R, poseNodes[id1].R, tmp);
			mat33Trans(tmp, b.data + 9 * r);

		} else if (!poseNodes[id1].fixed && poseNodes[id2].fixed) {
			T.add(9 * r, 9 * c1, -R[0]);
			T.add(9 * r, 9 * c1 + 1, -R[1]);
			T.add(9 * r, 9 * c1 + 2, -R[2]);
			T.add(9 * r + 1, 9 * c1, -R[3]);
			T.add(9 * r + 1, 9 * c1 + 1, -R[4]);
			T.add(9 * r + 1, 9 * c1 + 2, -R[5]);
			T.add(9 * r + 2, 9 * c1, -R[6]);
			T.add(9 * r + 2, 9 * c1 + 1, -R[7]);
			T.add(9 * r + 2, 9 * c1 + 2, -R[8]);

			T.add(9 * r + 3, 9 * c1 + 3, -R[0]);
			T.add(9 * r + 3, 9 * c1 + 4, -R[1]);
			T.add(9 * r + 3, 9 * c1 + 5, -R[2]);
			T.add(9 * r + 4, 9 * c1 + 3, -R[3]);
			T.add(9 * r + 4, 9 * c1 + 4, -R[4]);
			T.add(9 * r + 4, 9 * c1 + 5, -R[5]);
			T.add(9 * r + 5, 9 * c1 + 3, -R[6]);
			T.add(9 * r + 5, 9 * c1 + 4, -R[7]);
			T.add(9 * r + 5, 9 * c1 + 5, -R[8]);

			T.add(9 * r + 6, 9 * c1 + 6, -R[0]);
			T.add(9 * r + 6, 9 * c1 + 7, -R[1]);
			T.add(9 * r + 6, 9 * c1 + 8, -R[2]);
			T.add(9 * r + 7, 9 * c1 + 6, -R[3]);
			T.add(9 * r + 7, 9 * c1 + 7, -R[4]);
			T.add(9 * r + 7, 9 * c1 + 8, -R[5]);
			T.add(9 * r + 8, 9 * c1 + 6, -R[6]);
			T.add(9 * r + 8, 9 * c1 + 7, -R[7]);
			T.add(9 * r + 8, 9 * c1 + 8, -R[8]);

			mat33Trans(poseNodes[id2].R, b.data + 9 * r);

			b.data[9 * r] = -b.data[9 * r];
			b.data[9 * r + 1] = -b.data[9 * r + 1];
			b.data[9 * r + 2] = -b.data[9 * r + 2];
			b.data[9 * r + 3] = -b.data[9 * r + 3];
			b.data[9 * r + 4] = -b.data[9 * r + 4];
			b.data[9 * r + 5] = -b.data[9 * r + 5];
			b.data[9 * r + 6] = -b.data[9 * r + 6];
			b.data[9 * r + 7] = -b.data[9 * r + 7];
			b.data[9 * r + 8] = -b.data[9 * r + 8];
		}
	}

	//split the T into two sub matrices T1, T2;
	Triplets T1, T2;
	tripletsSplitCol(T, nUnknownNode * 9, T1, T2);

	//C is the constraint matrix
	Mat_d CT;
	matTrans(C, CT);
	//QR : C^T = QR
	Mat_d Q, R;
	matQR(CT, Q, R);

	SparseMat A1, A2, sQ;
	triplets2Sparse(T1, A1);
	triplets2Sparse(T2, A2);
	dense2Sparse(Q, sQ);

	SparseMat sA2Q, subA2Q;
	sparseMatMul(A2, sQ, sA2Q);
	sparseSplitCol(sA2Q, C.m, subA2Q, false);

	Mat_d x(T.n, 1), y(A2.n, 1);
	y.fill(0);
	sparseSolveLin(A1, subA2Q, b.data, x.data, y.data + C.m);
	matAx(Q.m, Q.n, Q, y, x + A1.n);

	//get the approximated rotations
	for (int k = 0; k < nNodes; k++) {
		int c = indC.data[k];
		if (c >= 0) {
			double tmp[9];
			mat33Trans(x + 9 * c, tmp);
			approxRotationMat(tmp, poseNodes[k].newR);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		} else {
			memcpy(poseNodes[k].newR, poseNodes[k].R, sizeof(double) * 9);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		}
	}
}

void GlobalPoseGraph::getTConstraint(Mat_d& C) {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].constraint)
			indR.data[k] = nValidEdge++;
	}
	int nValidNode = 0;
	Mat_i indC(nNodes, 1);
	indC.fill(-1);
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed && poseNodes[k].constraint)
			indC.data[k] = nValidNode++;
	}

	C.resize(nValidEdge * 3, nValidNode * 3 + nConstraintEdge);
	C.fill(0);

	int nScale = 0;
	for (int i = 0; i < nEdges; i++) {
		if (indR.data[i] < 0)
			continue;

		int id1 = poseEdges[i].id1;
		int id2 = poseEdges[i].id2;

		int r = indR.data[i];
		int c1 = indC.data[id1];
		int c2 = indC.data[id2];
		assert(c1 >= 0 && c2 >= 0);

		double* R = poseEdges[i].R;
		double* t = poseEdges[i].t;

		C(3 * r, 3 * c2) = 1.0;
		C(3 * r + 1, 3 * c2 + 1) = 1.0;
		C(3 * r + 2, 3 * c2 + 2) = 1.0;

		C(3 * r, 3 * c1) = -R[0];
		C(3 * r, 3 * c1 + 1) = -R[1];
		C(3 * r, 3 * c1 + 2) = -R[2];
		C(3 * r + 1, 3 * c1) = -R[3];
		C(3 * r + 1, 3 * c1 + 1) = -R[4];
		C(3 * r + 1, 3 * c1 + 2) = -R[5];
		C(3 * r + 2, 3 * c1) = -R[6];
		C(3 * r + 2, 3 * c1 + 1) = -R[7];
		C(3 * r + 2, 3 * c1 + 2) = -R[8];

		C(3 * r, 3 * nValidNode + nScale) = -t[0];
		C(3 * r + 1, 3 * nValidNode + nScale) = -t[1];
		C(3 * r + 2, 3 * nValidNode + nScale) = -t[2];
		nScale++;
	}

//	//test
//	logInfo("T constraint:\n");
//	print(C);
}
void GlobalPoseGraph::computeNewCameraTranslations2(const Mat_d& C) {
	int nValidEdge = 0;

	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		int i = poseEdges[k].id1;
		int j = poseEdges[k].id2;
		if (poseEdges[k].constraint)
			continue;
		if (poseNodes[i].fixed && poseNodes[j].fixed)
			continue;
		indR.data[k] = nValidEdge++;
	}

	Mat_i indC(nNodes, 1);
	int nValidNode = 0, nUnknownNode = 0, nConstraintNode = 0;
	indC.fill(-1);

	std::vector<int> unfixedIds, constraintIds;
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed) {
			if (!poseNodes[k].constraint) {
				unfixedIds.push_back(k);
				nUnknownNode++;
			} else {
				constraintIds.push_back(k);
				nConstraintNode++;
			}
			nValidNode++;
		}
	}

	for (int i = 0; i < nUnknownNode; i++) {
		indC.data[unfixedIds[i]] = i;
	}
	for (int i = 0; i < nConstraintNode; i++) {
		indC.data[constraintIds[i]] = i + nUnknownNode;
	}

	Triplets T;
	int m = 3 * nValidEdge;
	int n = 3 * nValidNode + nConstraintEdge;
	int nnz = 2 * (nValidEdge * 12 + nConstraintEdge * 3);

	Mat_d b(m, 1);
	T.reserve(m, n, nnz);
	b.fill(0);

	int nScale = 0;
	for (int k = 0; k < nEdges; k++) {
		if (indR.data[k] < 0)
			continue;

		int id1 = poseEdges[k].id1;
		int id2 = poseEdges[k].id2;
		double* t = poseEdges[k].t;
		double* R = poseEdges[k].R;

		int i = indC.data[id1];
		int j = indC.data[id2];
		int r = indR.data[k];

		//valid edge gives a constraint
		if (!poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			//fill b
			T.add(3 * r, 3 * j, 1.0);
			T.add(3 * r + 1, 3 * j + 1, 1.0);
			T.add(3 * r + 2, 3 * j + 2, 1.0);

			T.add(3 * r, 3 * i, -R[0]);
			T.add(3 * r, 3 * i + 1, -R[1]);
			T.add(3 * r, 3 * i + 2, -R[2]);
			T.add(3 * r + 1, 3 * i, -R[3]);
			T.add(3 * r + 1, 3 * i + 1, -R[4]);
			T.add(3 * r + 1, 3 * i + 2, -R[5]);
			T.add(3 * r + 2, 3 * i, -R[6]);
			T.add(3 * r + 2, 3 * i + 1, -R[7]);
			T.add(3 * r + 2, 3 * i + 2, -R[8]);

			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + nScale, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + nScale, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + nScale, -t[2]);
			} else {
				memcpy(b + 3 * r, t, sizeof(double) * 3);
			}
		} else if (!poseNodes[id1].fixed && poseNodes[id2].fixed) {
			T.add(3 * r, 3 * i, -R[0]);
			T.add(3 * r, 3 * i + 1, -R[1]);
			T.add(3 * r, 3 * i + 2, -R[2]);
			T.add(3 * r + 1, 3 * i, -R[3]);
			T.add(3 * r + 1, 3 * i + 1, -R[4]);
			T.add(3 * r + 1, 3 * i + 2, -R[5]);
			T.add(3 * r + 2, 3 * i, -R[6]);
			T.add(3 * r + 2, 3 * i + 1, -R[7]);
			T.add(3 * r + 2, 3 * i + 2, -R[8]);

			double* tj = poseNodes[id1].t;
			assert(!poseEdges[k].uncertainScale);
			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + nScale, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + nScale, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + nScale, -t[2]);

				b[3 * r] = -tj[0];
				b[3 * r + 1] = -tj[1];
				b[3 * r + 2] = -tj[2];
			} else {
				b[3 * r] = t[0] - tj[0];
				b[3 * r + 1] = t[1] - tj[1];
				b[3 * r + 2] = t[2] - tj[2];
			}
		} else if (poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			T.add(3 * r, 3 * j, 1.0);
			T.add(3 * r + 1, 3 * j + 1, 1.0);
			T.add(3 * r + 2, 3 * j + 2, 1.0);

			double* ti = poseNodes[id1].t;
			assert(!poseEdges[k].uncertainScale);
			if (poseEdges[k].uncertainScale) {
				T.add(3 * r, 3 * nValidNode + nScale, -t[0]);
				T.add(3 * r + 1, 3 * nValidNode + nScale, -t[1]);
				T.add(3 * r + 2, 3 * nValidNode + nScale, -t[2]);
				mat33AB(R, ti, b + 3 * r);
			} else {
				mat33ProdVec(R, ti, t, b + 3 * r, 1.0, 1.0);
			}
		}
		if (poseEdges[k].uncertainScale)
			nScale++;
	}

	//split the T into two sub matrices T1, T2;
	Triplets T1, T2;
	tripletsSplitCol(T, nUnknownNode * 3, T1, T2);

	//C is the constraint matrix
	Mat_d CT;
	matTrans(C, CT);
	//QR : C^T = QR
	Mat_d Q, R;
	matQR(CT, Q, R);

	SparseMat A1, A2, sQ;
	triplets2Sparse(T1, A1);
	triplets2Sparse(T2, A2);
	dense2Sparse(Q, sQ);

	SparseMat sA2Q, subA2Q;
	sparseMatMul(A2, sQ, sA2Q);
	sparseSplitCol(sA2Q, C.m, subA2Q, false);

	Mat_d x(T.n, 1), y(A2.n, 1);
	sparseSolveLin(A1, subA2Q, b.data, x.data, y.data + C.m);
	matAx(Q.m, Q.n, Q, y, x + A1.n);

	//write back 
	for (int k = 0; k < nNodes; k++) {
		int c = indC.data[k];
		if (c < 0) {
			memcpy(poseNodes[k].newR, poseNodes[k].R, sizeof(double) * 9);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		} else {
			memcpy(poseNodes[k].newt, x + 3 * c, sizeof(double) * 3);
		}
	}

	int num = 0;
	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].uncertainScale) {
			int j = poseEdges[k].s = x[3 * j];
			num++;
		}
	}
}

void GlobalPoseGraph::computeNewCameraRotations3(const Mat_d& C) {
	computeNewCameraRotations2(C);
}
void GlobalPoseGraph::getTConstraint3(Mat_d& C, Mat_i& indScale,
		int& numScale) {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].constraint)
			indR.data[k] = nValidEdge++;
	}
	C.clear();

	//get the number of unified scale
	Mat_i sIdFlag(nEdges, 1);
	sIdFlag.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].constraint) {
			int sId = poseEdges[k].scaleId;
			assert(sId >= 0);
			sIdFlag.data[sId] = sId;
		}
	}

	indScale.resize(nEdges, 1);
	indScale.fill(-1);

	numScale = 0;
	for (int k = 0; k < nEdges; k++) {
		int sId = sIdFlag.data[k];
		if (sId >= 0) {
			indScale.data[sId] = numScale;
			numScale++;
		}
	}

	int nValidNode = 0;
	Mat_i indC(nNodes, 1);
	indC.fill(-1);
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed && poseNodes[k].constraint)
			indC.data[k] = nValidNode++;
	}

	C.resize(nValidEdge * 3, nValidNode * 3 + numScale);
	C.fill(0);

	for (int i = 0; i < nEdges; i++) {
		if (indR.data[i] < 0)
			continue;

		int id1 = poseEdges[i].id1;
		int id2 = poseEdges[i].id2;
		int sId = poseEdges[i].scaleId;

		int r = indR.data[i];
		int c1 = indC.data[id1];
		int c2 = indC.data[id2];
		int c3 = indScale.data[sId] + 3 * nValidNode;
		assert(c1 >= 0 && c2 >= 0 && c3 >= 0);

		double* R = poseEdges[i].R;
		double* t = poseEdges[i].t;

		C(3 * r, 3 * c2) = 1.0;
		C(3 * r + 1, 3 * c2 + 1) = 1.0;
		C(3 * r + 2, 3 * c2 + 2) = 1.0;

		C(3 * r, 3 * c1) = -R[0];
		C(3 * r, 3 * c1 + 1) = -R[1];
		C(3 * r, 3 * c1 + 2) = -R[2];
		C(3 * r + 1, 3 * c1) = -R[3];
		C(3 * r + 1, 3 * c1 + 1) = -R[4];
		C(3 * r + 1, 3 * c1 + 2) = -R[5];
		C(3 * r + 2, 3 * c1) = -R[6];
		C(3 * r + 2, 3 * c1 + 1) = -R[7];
		C(3 * r + 2, 3 * c1 + 2) = -R[8];

		C(3 * r, c3) = -t[0];
		C(3 * r + 1, c3) = -t[1];
		C(3 * r + 2, c3) = -t[2];
	}
//	//test
//	logInfo("T constraint:\n");
//	print(C);
}
void GlobalPoseGraph::computeNewCameraTranslations3(const Mat_d& C,
		Mat_i& indScale, int numScale) {
	int nValidEdge = 0;
	Mat_i indR(nEdges, 1);
	indR.fill(-1);
	for (int k = 0; k < nEdges; k++) {
		int i = poseEdges[k].id1;
		int j = poseEdges[k].id2;
		if (poseEdges[k].constraint)
			continue;
		if (poseNodes[i].fixed && poseNodes[j].fixed)
			continue;
		indR.data[k] = nValidEdge++;
	}

	Mat_i indC(nNodes, 1);
	int nValidNode = 0, nUnfixedNode = 0, nConstraintNode = 0;
	indC.fill(-1);

	std::vector<int> unfixedIds, constraintIds;
	for (int k = 0; k < nNodes; k++) {
		if (!poseNodes[k].fixed) {
			if (!poseNodes[k].constraint) {
				unfixedIds.push_back(k);
				nUnfixedNode++;
			} else {
				constraintIds.push_back(k);
				nConstraintNode++;
			}
			nValidNode++;
		}
	}

	for (int i = 0; i < nUnfixedNode; i++) {
		indC.data[unfixedIds[i]] = i;
	}
	for (int i = 0; i < nConstraintNode; i++) {
		indC.data[constraintIds[i]] = i + nUnfixedNode;
	}

	Triplets T;
	int m = 3 * nValidEdge;
	int n = 3 * nValidNode + numScale;
	int nnz = 2 * (nValidEdge * 12 + nConstraintEdge * 3);

	Mat_d b(m, 1);
	T.reserve(m, n, nnz);
	b.fill(0);

	for (int k = 0; k < nEdges; k++) {
		if (indR.data[k] < 0)
			continue;

		int id1 = poseEdges[k].id1;
		int id2 = poseEdges[k].id2;
		double* t = poseEdges[k].t;
		double* R = poseEdges[k].R;

		int i = 3 * indC.data[id1];
		int j = 3 * indC.data[id2];
		int r = indR.data[k];

		//valid edge gives a constraint
		if (!poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			//fill b
			T.add(3 * r, j, 1.0);
			T.add(3 * r + 1, j + 1, 1.0);
			T.add(3 * r + 2, j + 2, 1.0);

			T.add(3 * r, i, -R[0]);
			T.add(3 * r, i + 1, -R[1]);
			T.add(3 * r, i + 2, -R[2]);
			T.add(3 * r + 1, i, -R[3]);
			T.add(3 * r + 1, i + 1, -R[4]);
			T.add(3 * r + 1, i + 2, -R[5]);
			T.add(3 * r + 2, i, -R[6]);
			T.add(3 * r + 2, i + 1, -R[7]);
			T.add(3 * r + 2, i + 2, -R[8]);

			memcpy(b + 3 * r, t, sizeof(double) * 3);
		} else if (!poseNodes[id1].fixed && poseNodes[id2].fixed) {
			T.add(3 * r, i, -R[0]);
			T.add(3 * r, i + 1, -R[1]);
			T.add(3 * r, i + 2, -R[2]);
			T.add(3 * r + 1, i, -R[3]);
			T.add(3 * r + 1, i + 1, -R[4]);
			T.add(3 * r + 1, i + 2, -R[5]);
			T.add(3 * r + 2, i, -R[6]);
			T.add(3 * r + 2, i + 1, -R[7]);
			T.add(3 * r + 2, i + 2, -R[8]);

			double* tj = poseNodes[id2].t;
			b[3 * r] = t[0] - tj[0];
			b[3 * r + 1] = t[1] - tj[1];
			b[3 * r + 2] = t[2] - tj[2];
		} else if (poseNodes[id1].fixed && !poseNodes[id2].fixed) {
			T.add(3 * r, j, 1.0);
			T.add(3 * r + 1, j + 1, 1.0);
			T.add(3 * r + 2, j + 2, 1.0);

			double* ti = poseNodes[id1].t;
			mat33ProdVec(R, ti, t, b + 3 * r, 1.0, 1.0);
		}
	}

	//split the T into two sub matrices T1, T2;
	Triplets T1, T2;
	tripletsSplitCol(T, nUnfixedNode * 3, T1, T2);

	//C is the constraint matrix
	Mat_d CT;
	matTrans(C, CT);
	//QR : C^T = QR
	Mat_d Q, R;
	matQR(CT, Q, R);

	SparseMat A1, A2, sQ;
	triplets2Sparse(T1, A1);
	triplets2Sparse(T2, A2);
	dense2Sparse(Q, sQ);

	SparseMat sA2Q, subA2Q;
	sparseMatMul(A2, sQ, sA2Q);
	sparseSplitCol(sA2Q, C.m, subA2Q, false);

	Mat_d x(T.n, 1), y(A2.n, 1);
	sparseSolveLin(A1, subA2Q, b.data, x.data, y.data + C.m);
	matAx(Q.m, Q.n, Q, y, x + A1.n);

	//print(x);

	//write back 
	for (int k = 0; k < nNodes; k++) {
		int c = indC.data[k];
		if (c < 0) {
			memcpy(poseNodes[k].newR, poseNodes[k].R, sizeof(double) * 9);
			memcpy(poseNodes[k].newt, poseNodes[k].t, sizeof(double) * 3);
		} else {
			memcpy(poseNodes[k].newt, x + 3 * c, sizeof(double) * 3);
		}
	}

	for (int k = 0; k < nEdges; k++) {
		if (poseEdges[k].uncertainScale) {
			int sId = poseEdges[k].scaleId;
			if (sId < 0)
				continue;
			poseEdges[k].s = x[3 * nValidNode + indScale[sId]];
		}
	}
}

void GlobalPoseGraph::save(ofstream& file) const {
	file << nNodes << endl;
	file << nEdges << endl;
	for (int i = 0; i < nNodes; ++i) {
		file << poseNodes[i].id << " " << poseNodes[i].frame << " "
				<< poseNodes[i].fixed;

		for (int k = 0; k < 9; ++k)
			file << " " << poseNodes[i].R[k];

		file << " " << poseNodes[i].t[0] << " " << poseNodes[i].t[1] << " "
				<< poseNodes[i].t[2];

		for (int k = 0; k < 9; ++k)
			file << " " << poseNodes[i].newR[k];

		file << " " << poseNodes[i].newt[0] << " " << poseNodes[i].newt[1]
				<< " " << poseNodes[i].newt[2];

		file << " " << poseNodes[i].constraint << endl;
	}

	for (int i = 0; i < nEdges; ++i) {
		file << poseEdges[i].id1 << " " << poseEdges[i].id2 << " "
				<< poseEdges[i].constraint << " "
				<< poseEdges[i].uncertainScale;

		for (int k = 0; k < 9; ++k) {
			file << " " << poseEdges[i].R[k];
		}

		file << " " << poseEdges[i].t[0] << " " << poseEdges[i].t[1] << " "
				<< poseEdges[i].t[2];

		file << " " << poseEdges[i].s << " " << poseEdges[i].scaleId << endl;
	}

	file << nFixedNode << endl;
	file << nConstraintEdge << endl;

}
void GlobalPoseGraph::load(ifstream& file) {
	int numNodes, numEdges;
	file >> numNodes;
	file >> numEdges;

	reserve(2 * numNodes, 2 * numEdges);

	nNodes = numNodes;
	nEdges = numEdges;

	//read nodes
	for (int i = 0; i < nNodes; ++i) {
		file >> poseNodes[i].id >> poseNodes[i].frame >> poseNodes[i].fixed;
		for (int k = 0; k < 9; ++k) {
			file >> poseNodes[i].R[k];
		}
		file >> poseNodes[i].t[0] >> poseNodes[i].t[1] >> poseNodes[i].t[2];

		for (int k = 0; k < 9; ++k) {
			file >> poseNodes[i].newR[k];
		}
		file >> poseNodes[i].newt[0] >> poseNodes[i].newt[1]
				>> poseNodes[i].newt[2];
		file >> poseNodes[i].constraint;
	}

	//read edges
	for (int i = 0; i < nEdges; ++i) {
		file >> poseEdges[i].id1 >> poseEdges[i].id2;
		file >> poseEdges[i].constraint >> poseEdges[i].uncertainScale;
		for (int k = 0; k < 9; ++k) {
			file >> poseEdges[i].R[k];
		}
		file >> poseEdges[i].t[0] >> poseEdges[i].t[1] >> poseEdges[i].t[2];
		file >> poseEdges[i].s;
		file >> poseEdges[i].scaleId;
	}
}

void write(const GlobalPoseGraph& posegraph, const char* fmtstr, ...) {
	char buf[1024];
	GET_FMT_STR(fmtstr, buf)

	ofstream file(buf);
	if (!file)
		repErr("cannot open '%s' to write.", buf);

	posegraph.save(file);
	logInfo("write '%s' ok!\n", buf);
}
void read(GlobalPoseGraph& posegraph, const char* fmtstr, ...) {
	char buf[1024];
	GET_FMT_STR(fmtstr, buf)

	ifstream file(buf);
	if (!file)
		repErr("cannot open '%s' to read.", buf);
	
	posegraph.load(file);
	logInfo("read '%s' ok!", buf);
}