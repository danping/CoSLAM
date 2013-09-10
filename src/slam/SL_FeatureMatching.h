#ifndef SL_FEATUREMATCHING_H_
#define SL_FEATUREMATCHING_H_
#include "matching/SL_StereoMatcherHelper.h"
#include "SL_NCCBlock.h"
/*compute epipolar error matrix and NCC distance matrix together*/
void getEpiNccMat(const double* F,
	const Mat_d& pts1,
	const Mat_d& pts2,
	const PtrVec<NCCBlock>& blks1,
	const PtrVec<NCCBlock>& blks2,
	double epiMax,
	double nccMin,
	Mat_d& epiMat,
	Mat_d& nccMat,
	double wNone = -1);
#endif