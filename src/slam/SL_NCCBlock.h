/*
 * SL_NCCBlock.h
 *
 *  Created on: 2011-2-5
 *      Author: Danping Zou
 */

#ifndef SL_NCCBLOCK_H_
#define SL_NCCBLOCK_H_
#include "SL_PtrVec.h"
#include "math/SL_Matrix.h"
#include "imgproc/SL_Image.h"
#include <vector>

#define SL_NCCBLK_LEN 121
#define SL_NCCBLK_HW 5

class FeaturePoint;
class NCCBlock {
public:
	/*pointer to the image point*/
	const FeaturePoint* pt;
	/*NCC*/
	double A; //A = sum(I(:))
	double B; //B = sum(I(:).^2)
	double C; //C = 1/(121*B-A*A).^0.5
	/*Image data block*/
	unsigned char I[SL_NCCBLK_LEN]; //I
	double avgI;
public:
	NCCBlock() {
	}
	NCCBlock(const NCCBlock& other) {
		pt = other.pt;
		A = other.A;
		B = other.B;
		C = other.C;
		memcpy(I, other.I, SL_NCCBLK_LEN);
	}
	NCCBlock(const FeaturePoint* fp) :
		pt(fp) {
	}
	void copy(const NCCBlock& other) {
		pt = other.pt;
		A = other.A;
		B = other.B;
		C = other.C;
		memcpy(I, other.I, SL_NCCBLK_LEN);
	}
	bool compute(const ImgG& img, double x, double y);
	bool computeScaled(const ImgG& scaledImg, double scale, double x0, double y0);
public:
	template<class Archive>
	void serialize(Archive& ar, const unsigned int version) {
		ar & A;
		ar & B;
		ar & C;
		ar & I;
		ar & avgI;
	}
};

void getNCCBlock(const ImgG& img, double x, double y, NCCBlock& blk);
inline void getScaledNCCBlock(const ImgG& scaledImg, double scale, double x0, double y0, NCCBlock& blk) {
	getNCCBlock(scaledImg, x0 * scale, y0 * scale, blk);
}
/**
 * get NCC blocks from the already scaled image
 */
void getScaledNCCBlocks(const ImgG& scaledImg,
		double scale,
		const std::vector<FeaturePoint*>& vecFeatPts,
		PtrVec<NCCBlock>& nccBlks);

void getScaledNCCBlocks(const ImgG& scaledImg,
		double scale,
		const Mat_d& featpts,
		PtrVec<NCCBlock>& nccBlks);

void getNCCBlocks(const ImgG& scaledImg, Mat_d& pts, PtrVec<NCCBlock>& nccBlks, double scale = 1.0);
void getNCCBlocks(const ImgG& scaledImg,
		const std::vector<FeaturePoint*>& vecFeatPts,
		PtrVec<NCCBlock>& nccBlks,
		double scale = 1.0);

double matchNCCBlock(NCCBlock* pBlk1, NCCBlock* pBlk2);
#endif /* SL_NCCBLOCK_H_ */
