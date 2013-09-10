#include "SL_FeatureMatching.h"

void getEpiNccMat(const double* F, const Mat_d& pts1, const Mat_d& pts2,
	const PtrVec<NCCBlock>& blks1, const PtrVec<NCCBlock>& blks2,
	double epiMax, double nccMin, Mat_d& epiMat, Mat_d& nccMat,
	double wNone) {

		int M = pts1.rows;
		int N = pts2.rows;

		int M0 = (int) blks1.size();
		int N0 = (int) blks2.size();

		assert(M == M0 && N == N0);

		epiMat.resize(M, N);
		nccMat.resize(M, N);

		for (int i = 0; i < M; i++) {
			double* pEpiData = epiMat.data + i * N;
			double* pNccData = nccMat.data + i * N;

			for (int j = 0; j < N; j++) {
				double epiErr = epipolarError(F, pts1.data + 2 * i,
					pts2.data + 2 * j);
				if (epiErr <= epiMax) {
					//compute the NCC
					double ncc = matchNCCBlock(blks1[i], blks2[j]);
					if (ncc >= nccMin) {
						*pEpiData = epiErr;
						*pNccData = ncc;
					} else {
						*pEpiData = wNone;
						*pNccData = wNone;
					}
				} else {
					*pEpiData = wNone;
					*pNccData = wNone;
				}
				pEpiData++;
				pNccData++;
			}

		}
}