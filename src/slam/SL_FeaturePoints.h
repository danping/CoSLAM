/*
 * SL_FeaturePoints.h
 *
 *  Created on: 2011-1-12
 *      Author: Danping Zou
 */

#ifndef SL_FEATUREPOINTS_H_
#define SL_FEATUREPOINTS_H_
#include "SL_FeaturePoint.h"
#include <vector>
using namespace std;
class FeaturePoints {
public:
	int num;
	FeaturePoint hd;
	FeaturePoint* tail;

	//maintaining pointer to the first and the last map point at certain frame
	map<int, int> frameNum;
	map<int, FeaturePoint*> frameHead;
	map<int, FeaturePoint*> frameTail;
public:
	FeaturePoints();
	~FeaturePoints();
public:
	void clear();
	FeaturePoint* add(int f, int camId, double x, double y);
	FeaturePoint* add(FeaturePoint* fp);
	void removeFrame(int f);
	void removeBefore(int f);
	
	int totalFrameNum(int f) const;

	FeaturePoint* getFrameHead(int f) const {
		if (frameHead.count(f) == 0)
			return 0;
		return frameHead.at(f);
	}
	FeaturePoint* getFrameTail(int f) const {
		if (frameTail.count(f) == 0)
			return 0;
		return frameTail.at(f);
	}
	void getFrame(int f, vector<FeaturePoint*>& featPts) const;
public:
	//Debug
	void print() const;
	void print(const char* filePath) const;
	void printFrame(int f) const;
};
#endif /* SL_FEATUREPOINTS_H_ */
