/*
 * SL_NCCBlock.cpp
 *
 *  Created on: 2011-2-5
 *      Author: Danping Zou
 */

#include "SL_Define.h"
#include "SL_NCCBlock.h"
#include "SL_FeaturePoint.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

bool NCCBlock::compute(const ImgG& img, double x, double y) {
	int W = img.w;
	int H = img.h;
	int bw = 2 * SL_NCCBLK_HW + 1;

	int x0 = (int) x;
	int y0 = (int) y;

	/* ignore the feature points at boundaries*/
	if (x0 - SL_NCCBLK_HW < 0 || x0 + SL_NCCBLK_HW >= W || y0 - SL_NCCBLK_HW < 0
			|| y0 + SL_NCCBLK_HW >= H)
		return false;

	int j;
	avgI = 0;
	for (j = 0; j < SL_NCCBLK_LEN; ++j) {
		int y = j / bw;
		int x = j - y * bw;

		x = x0 + x - SL_NCCBLK_HW;
		y = y0 + y - SL_NCCBLK_HW;

		I[j] = img.data[y * W + x];
		avgI += I[j];
	}
	avgI /= SL_NCCBLK_LEN;
	/* compute NCC parameters */
	double a = 0, b = 0;
	for (j = 0; j < SL_NCCBLK_LEN; ++j) {
		a += I[j];
		b += double(I[j]) * I[j];
	}
	A = a;
	B = b;
	C = 1 / sqrt(SL_NCCBLK_LEN * b - a * a);
	return true;
}

bool NCCBlock::computeScaled(const ImgG& scaledImg, double scale, double x0,
		double y0) {
	return compute(scaledImg, x0 * scale, y0 * scale);
}

void getNCCBlock(const ImgG& img, double x, double y, NCCBlock& blk) {
	cv::Mat cvImg(img.rows, img.cols, CV_8UC1, img.data);
	cv::Size patchSize(2 * SL_NCCBLK_HW + 1, 2 * SL_NCCBLK_HW + 1);
	cv::Mat patch;
	cv::getRectSubPix(cvImg, patchSize, cv::Point2d(x, y), patch);

	memcpy(blk.I, patch.data, SL_NCCBLK_LEN);

	/*compute NCC parameters*/
	double a = 0, b = 0;
	blk.avgI = 0;
	for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
		a += blk.I[j];
		b += double(blk.I[j]) * blk.I[j];
		blk.avgI += blk.I[j];
	}
	blk.A = a;
	blk.B = b;
	blk.C = 1 / sqrt(SL_NCCBLK_LEN * b - a * a);
	blk.avgI /= SL_NCCBLK_LEN;
}
void getNCCBlocks(const ImgG& scaledImg, Mat_d& pts, PtrVec<NCCBlock>& nccBlks,
		double scale) {
	if (pts.empty())
		return;

	nccBlks.clear();
	nccBlks.reserve(pts.rows * 2);

	int bw = 2 * SL_NCCBLK_HW + 1;

	if (scale == 1.0) {
		cv::Mat cvImg(scaledImg.rows, scaledImg.cols, CV_8UC1, scaledImg.data);
		cv::Size patchSize(bw, bw);
		cv::Mat patch;

		for (int i = 0; i < pts.rows; i++) {
			double x = pts.data[2 * i];
			double y = pts.data[2 * i + 1];
			cv::getRectSubPix(cvImg, patchSize, cv::Point2d(x, y), patch);
			NCCBlock* blk = new NCCBlock(0);
			/* copy image block */
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				blk->I[j] = patch.data[j];
			}
			/* compute NCC parameters */
			double a = 0, b = 0;
			blk->avgI = 0;
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				a += blk->I[j];
				b += double(blk->I[j]) * blk->I[j];
				blk->avgI += blk->I[j];
			}
			blk->A = a;
			blk->B = b;
			blk->C = 1 / sqrt(SL_NCCBLK_LEN * b - a * a);
			blk->avgI /= SL_NCCBLK_LEN;
			nccBlks.push_back(blk);
		}
	} else {
		cv::Mat cvOrgImg(scaledImg.rows, scaledImg.cols, CV_8UC1,
				scaledImg.data);
		cv::Mat cvImg;
		cv::resize(cvOrgImg, cvImg, cv::Size(), scale, scale);
		cv::Size patchSize(bw, bw);
		cv::Mat patch;

		for (int i = 0; i < pts.rows; i++) {
			double x = pts.data[2 * i] * scale;
			double y = pts.data[2 * i + 1] * scale;
			cv::getRectSubPix(cvImg, patchSize, cv::Point2d(x, y), patch);
			NCCBlock* blk = new NCCBlock(0);

			/* copy image block */
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				blk->I[j] = patch.data[j];
			}
			/* compute NCC parameters */
			double a = 0, b = 0;
			blk->avgI = 0;
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				a += blk->I[j];
				b += double(blk->I[j]) * blk->I[j];
				blk->avgI += blk->I[j];
			}
			blk->A = a;
			blk->B = b;
			blk->C = 1 / sqrt(SL_NCCBLK_LEN * b - a * a);
			blk->avgI /= SL_NCCBLK_LEN;
			nccBlks.push_back(blk);
		}
	}
}

void getScaledNCCBlocks(const ImgG& scaledImg, double scale,
		const std::vector<FeaturePoint*>& vecFeatPts,
		PtrVec<NCCBlock>& nccBlks) {
	if (vecFeatPts.empty())
		return;

	int npts = (int) vecFeatPts.size();
	nccBlks.clear();
	nccBlks.reserve(npts * 2);

	for (int i = 0; i < npts; i++) {
		NCCBlock* blk = new NCCBlock();
		getScaledNCCBlock(scaledImg, scale, vecFeatPts[i]->x, vecFeatPts[i]->y,
				*blk);
		nccBlks.push_back(blk);
	}
}

void getScaledNCCBlocks(const ImgG& scaledImg, double scale,
		const Mat_d& featpts, PtrVec<NCCBlock>& nccBlks) {
	if (featpts.empty())
		return;

	int npts = (int) featpts.m;
	nccBlks.clear();
	nccBlks.reserve(npts * 2);

	for (int i = 0; i < npts; i++) {
		NCCBlock* blk = new NCCBlock();
		getScaledNCCBlock(scaledImg, scale, featpts.data[2 * i],
				featpts.data[2 * i + 1], *blk);
		nccBlks.push_back(blk);
	}
}

void getNCCBlocks(const ImgG& scaledImg,
		const std::vector<FeaturePoint*>& vecFeatPts, PtrVec<NCCBlock>& nccBlks,
		double scale) {
	if (vecFeatPts.empty())
		return;

	nccBlks.clear();
	nccBlks.reserve(vecFeatPts.size() * 2);
	int bw = 2 * SL_NCCBLK_HW + 1;

	if (scale == 1.0) {
		cv::Mat cvImg(scaledImg.rows, scaledImg.cols, CV_8UC1, scaledImg.data);
		cv::Size patchSize(bw, bw);
		cv::Mat patch;

		for (size_t i = 0; i < vecFeatPts.size(); i++) {
			FeaturePoint* pFt = vecFeatPts[i];
			cv::getRectSubPix(cvImg, patchSize, cv::Point2d(pFt->x, pFt->y),
					patch);
			NCCBlock* blk = new NCCBlock(pFt);
			/* copy image block */
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				blk->I[j] = patch.data[j];
			}
			/* compute NCC parameters */
			double a = 0, b = 0;
			blk->avgI = 0;
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				a += blk->I[j];
				b += double(blk->I[j]) * blk->I[j];
				blk += blk->I[j];
			}
			blk->A = a;
			blk->B = b;
			blk->C = 1 / sqrt(SL_NCCBLK_LEN * b - a * a);
			blk->avgI /= SL_NCCBLK_LEN;
			nccBlks.push_back(blk);
		}
	} else {
		cv::Mat cvOrgImg(scaledImg.rows, scaledImg.cols, CV_8UC1,
				scaledImg.data);
		cv::Mat cvImg;
		cv::resize(cvOrgImg, cvImg, cv::Size(), scale, scale);

		cv::Size patchSize(bw, bw);
		cv::Mat patch;

		for (size_t i = 0; i < vecFeatPts.size(); i++) {
			FeaturePoint* pFt = vecFeatPts[i];
			cv::getRectSubPix(cvImg, patchSize,
					cv::Point2d(pFt->x * scale, pFt->y * scale), patch);
			NCCBlock* blk = new NCCBlock(pFt);
			/* copy image block */
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				blk->I[j] = patch.data[j];
			}
			/* compute NCC parameters */
			double a = 0, b = 0;
			blk->avgI = 0;
			for (int j = 0; j < SL_NCCBLK_LEN; ++j) {
				a += blk->I[j];
				b += double(blk->I[j]) * blk->I[j];
				blk->avgI += blk->I[j];
			}
			blk->A = a;
			blk->B = b;
			blk->C = 1 / sqrt(SL_NCCBLK_LEN * b - a * a);
			blk->avgI /= SL_NCCBLK_LEN;
			nccBlks.push_back(blk);
		}
	}
}
double matchNCCBlock(NCCBlock* pBlk1, NCCBlock* pBlk2) {
	double d = 0;
	for (int i = 0; i < SL_NCCBLK_LEN; i++) {
		d += double(pBlk1->I[i]) * pBlk2->I[i];
	}
	return (SL_NCCBLK_LEN * d - pBlk1->A * pBlk2->A) * pBlk1->C * pBlk2->C;
}
