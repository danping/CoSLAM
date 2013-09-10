/*
 * SL_FeaturePoints.cpp
 *
 *  Created on: 2011-1-12
 *      Author: Danping Zou
 */

#include "SL_FeaturePoints.h"
#include "SL_MapPoint.h"
#include "SL_error.h"

FeaturePoints::FeaturePoints() :
		num(0), hd(), tail(0), frameHead(), frameTail() {

}
FeaturePoints::~FeaturePoints() {
	clear();
}
void FeaturePoints::clear() {
	FeaturePoint* pt = hd.next;
	while (pt) {
		FeaturePoint* q = pt;
		pt = pt->next;
		delete q;
	}
	num = 0;
	hd.pre = 0;
	hd.next = 0;
	tail = 0;
	frameNum.clear();
	frameHead.clear();
	frameTail.clear();
}
FeaturePoint* FeaturePoints::add(FeaturePoint* pt) {
	int f = pt->f;
	if (hd.next == 0) {
		hd.next = pt;
		pt->pre = &hd;
		frameHead[f] = pt;
		frameTail[f] = pt;
	} else {
		//find the existing queue for the source frame
		if (frameHead.count(f) == 0) {
			frameHead[f] = pt;
			frameTail[f] = pt;
			if (tail->next == 0) {
				//append
				tail->next = pt;
				pt->pre = tail;
			} else {
				//insert
				pt->next = tail->next;
				pt->next->pre = pt;

				tail->next = pt;
				pt->pre = tail;
			}
		} else {
			FeaturePoint* ft = frameTail[f];
			if (ft->next == 0) {
				//append
				ft->next = pt;
				pt->pre = ft;
			} else {
				//insert
				pt->next = ft->next;
				pt->next->pre = pt;

				ft->next = pt;
				pt->pre = ft;
			}

			frameTail[f] = pt;
		}
	}
	tail = pt;
	frameNum[f]++;
	++num;
	return pt;
}
FeaturePoint* FeaturePoints::add(int f, int camId, double x, double y) {
	if (f < 0) {
		repErr("ImgPointSet::add() - error");
	}
	FeaturePoint* pt = new FeaturePoint(f, camId, x, y);
	return add(pt);
}
void FeaturePoints::removeFrame(int f) {
	if (frameHead.count(f) == 0)
		return;

	FeaturePoint* pt = frameHead.at(f);
	FeaturePoint* pe = frameTail.at(f);

	FeaturePoint* pre_head = pt->pre;
	FeaturePoint* next_tail = pe->next;

	int c = 0;
	while (pt != next_tail) {
		FeaturePoint* q = pt;
		pt = pt->next;
		delete q;
		++c;
	}

	if (next_tail == 0) {
		pre_head->next = 0;
		if (pre_head == &hd)
			tail = 0;
		else
			tail = pre_head;
	} else {
		pre_head->next = next_tail;
		next_tail->pre = pre_head;
		tail = next_tail;
	}

	frameHead.erase(f);
	frameTail.erase(f);
	num -= c;
	frameNum[f] = 0;
}
void FeaturePoints::removeBefore(int frame) {
	if (!hd.next || hd.next->f >= frame)
		return;
	int f0 = hd.next->f;

	vector<FeaturePoint*> oldpts, newpts;
	oldpts.reserve(num);
	newpts.reserve(num);
	for (FeaturePoint* p = hd.next; p; p = p->next) {
		if (p->f < frame)
			oldpts.push_back(p);
		else
			newpts.push_back(p);
	}

	//clear the pointers for accessing feature tracks
	for (size_t i = 0; i < newpts.size(); ++i) {
		for (FeaturePoint* p = newpts[i]; p; p = p->preFrame) {
			if (p->preFrame && p->preFrame->f < frame) {
				p->preFrame = 0;
				break;
			}
		}
	}

	//release the old points
	for (size_t i = 0; i < oldpts.size(); ++i) {
		FeaturePoint* p = oldpts[i];
		if (p->mpt) {
			if (p->mpt->pFeatures[p->camId] == p) {
				p->mpt->pFeatures[p->camId] = 0;
			}
		}
		delete p;
		num--;
	}

	hd.next = newpts.empty() ? 0 : newpts[0];
	if (hd.next)
		hd.next->pre = &hd;
	
	for (int f = f0; f < frame; f++) {
		if (frameNum.count(f) > 0) {
			frameNum[f] = 0;
			frameHead.erase(f);
			frameTail.erase(f);
		}
	}
}
int FeaturePoints::totalFrameNum(int f) const {
	if (frameNum.count(f) == 0)
		return 0;
	return frameNum.at(f);
}
void FeaturePoints::getFrame(int f, vector<FeaturePoint*>& featPts) const {
	FeaturePoint* pHead = getFrameHead(f);
	if( !pHead)
		return;
	FeaturePoint* pTail = getFrameTail(f);
	for (FeaturePoint* p = pHead; p != pTail->next; p = p->next) {
		featPts.push_back(p);
	}
}
void FeaturePoints::print() const {
	logInfo("----%d points --- \n", num);
	const FeaturePoint* pt = hd.next;
	while (pt) {
		logInfo("(%d)[%d, %d](%lf,%lf)", pt, pt->f, pt->camId, pt->x, pt->y);
		if (pt->mpt)
			logInfo("---mpt:[%d](%g %g %g)", pt->mpt->firstFrame, pt->mpt->x,
					pt->mpt->y, pt->mpt->z);
		logInfo("\n");
		pt = pt->next;
	}
}
void FeaturePoints::print(const char *filePath) const {
	FILE* fp = fopen(filePath, "w+");
	logInfo("----%d points --- \n", num);
	const FeaturePoint* pt = hd.next;
	while (pt) {
		fprintf(fp, "[%d](%lf,%lf)\n", pt->f, pt->x, pt->y);
		pt = pt->next;
	}
	fclose(fp);
}
void FeaturePoints::printFrame(int f) const {
	if (frameHead.count(f) == 0) {
		logInfo("no points at this frame %d.\n", f);
		return;
	}
	logInfo("----%d points --- \n", totalFrameNum(f));
	const FeaturePoint* pt = frameHead.at(f);
	const FeaturePoint* pe = frameTail.at(f)->next;
	while (pt != pe) {
		logInfo("[%d](%lf,%lf)\n", pt->f, pt->x, pt->y);
		pt = pt->next;
	}
}