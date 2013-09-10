/*
 * SL_Track2D.h
 *
 *  Created on: 2010-11-10
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_TRACK2D_H_
#define SL_TRACK2D_H_
#include "imgproc/SL_Image.h"
#include "slam/SL_FeaturePoint.h"

#include <list>
#include <vector>

class Track2DNode {
public:
	/* track id*/
	int tkid;
	/* pointer to the image point*/
	FeaturePoint* pt;
	/* frame */
	int f;
	/* image coordinates */
	double x, y;

	Track2DNode* pre;
	Track2DNode* next;
public:
	Track2DNode() :
		tkid(0), pt(0), f(-1), x(0), y(0), pre(0), next(0) {
	}
	Track2DNode(int f1, FeaturePoint* p) :
		tkid(0), pt(p), f(f1), x(p->x), y(p->y), pre(0), next(0) {

	}
	Track2DNode(FeaturePoint* p) :
		tkid(0), pt(p), f(p->f), x(p->x), y(p->y), pre(0), next(0) {
	}
	Track2DNode(int f1, double x1, double y1) :
		tkid(0), pt(0), f(f1), x(x1), y(y1), pre(0), next(0) {

	}
};

class Track2D {
public:
	unsigned int id;
	/* trajectory duration */
	int f1, f2;
	/* list head*/
	Track2DNode head;
	/* pointer to list tail*/
	Track2DNode* tail;
public:
	int length() const {
		return f2 - f1 + 1;
	}
	void clear() {
		Track2DNode* p = head.next;
		Track2DNode* q = 0;
		while (p) {
			q = p;
			p = p->next;
			delete q;
		}
		f1 = f2 = -1;
		head.next = 0;
		tail = 0;
	}
	bool empty() const {
		return tail == 0;
	}
	void add(int f, double x, double y) {
		Track2DNode* node = new Track2DNode(f, x, y);
		add(node);
	}
	void add(Track2DNode* node) {
		node->tkid = id;

		if (node->f < f1 || f1 < 0)
			f1 = node->f;
		if (node->f > f2 || f2 < 0)
			f2 = node->f;

		/* add to tail*/
		if (tail == 0) {
			head.next = node;
			node->pre = 0;
			if (node->pt)
				node->pt->preFrame = 0;
		} else {
			tail->next = node;
			node->pre = tail;
			if (node->pt) {
				//connect the successive feature points
				node->pt->nextFrame = 0;
				node->pt->preFrame = tail->pt;
				tail->pt->nextFrame = node->pt;
			}
		}
		node->next = 0;
		tail = node;
	}
	void add(int f, FeaturePoint* pt) {
		Track2DNode* node = new Track2DNode(f, pt);
		add(node);
	}
	void add(FeaturePoint* pt) {
		Track2DNode* node = new Track2DNode(pt);
		add(node);
	}
public:
	Track2D();
	~Track2D();
};

//////////////////////////////////////////////////////////////////////
//The following will be removed
typedef std::list<Track2D*> Track2DPtrList;
typedef Track2D* Track2DPtr;
typedef Track2D** Track2DPtrPtr;

/* select tracks from f1*/
void trackSelect(const Track2DPtrList& tks, int f1, Track2DPtrList& cmptks);

/* get corresponding points from the tracks*/
int trackGetCorrespondenceNum(const Track2DPtrList& tks, int f1, int f2);
int trackGetCorrespondence(const Track2DPtrList& tks, int f1, int f2, double* pts1, double* pts2);
int trackGetCorrespondence(const Track2DPtrList& tks,
		int f1,
		int f2,
		std::vector<Track2DNode*>& nodes1,
		std::vector<Track2DNode*>& nodes2);
void trackNodes2Arr(std::vector<Track2DNode*>& nodes, int i1, int i2, double* pts);

#endif /* SL_TRACK2D_H_ */
