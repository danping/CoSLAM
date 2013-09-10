/*
 * SL_Track2D.cpp
 *
 *  Created on: 2010-11-10
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_Track2D.h"
#include "SL_error.h"

Track2D::Track2D() :
	id(0), f1(-1), f2(-1), head(), tail(0) {
	static unsigned int _id = 0;
	_id++;
	id = _id;
}
Track2D::~Track2D() {
	clear();
}

void trackSelect(const Track2DPtrList& tks , int f1 , Track2DPtrList& cmptks) {
	cmptks.clear();
	Track2DPtrList::const_iterator iter = tks.begin();
	while (iter != tks.end()) {
		if ((*iter)->f1 <= f1)
			cmptks.push_back((*iter));
		++iter;
	}
}

int trackGetCorrespondenceNum(const Track2DPtrList& tks , int f1 , int f2) {
	if (f1 >= f2)
		repErr("tracks_get_corresponds_num() : f1 < f2 is required.");
	//get the number of valid tracks
	Track2DPtrList::const_iterator iter;
	int cn = 0;
	for (iter = tks.begin(); iter != tks.end(); ++iter) {
		Track2D* tk = *iter;
		if (tk->f1 <= f1 && tk->f2 >= f2)
			cn++;
	}
	return cn;
}
int trackGetCorrespondence(const Track2DPtrList& tks , int f1 , int f2 , double* pts1 , double* pts2) {
	if (f1 >= f2)
		repErr("tracks_get_corresponds() : f1 < f2 is required.");

	Track2DPtrList::const_iterator iter;
	int cn = 0;
	for (iter = tks.begin(); iter != tks.end(); ++iter) {
		Track2D* tk = *iter;
		if (tk->f1 <= f1 && tk->f2 >= f2) {
			Track2DNode* p = tk->head.next;
			while (p) {
				if (p->f == f1) {
					pts1[cn * 2] = p->x;
					pts1[cn * 2 + 1] = p->y;
				} else if (p->f == f2) {
					pts2[cn * 2] = p->x;
					pts2[cn * 2 + 1] = p->y;
				}
				p = p->next;
			}
			cn++;
		}
	}
	return cn;
}

int trackGetCorrespondence(const Track2DPtrList& tks , int f1 , int f2 , std::vector<Track2DNode*>& nodes1 , std::vector<
		Track2DNode*>& nodes2) {
	if (f1 >= f2)
		repErr("tracks_get_corresponds() : f1 < f2 is required.");
	Track2DPtrList::const_iterator iter;

	int cn = 0;
	for (iter = tks.begin(); iter != tks.end(); ++iter) {
		Track2D* tk = *iter;
		if (tk->f1 <= f1 && tk->f2 >= f2) {
			Track2DNode* p = tk->head.next;
			while (p) {
				if (p->f == f1) {
					nodes1.push_back(p);
				} else if (p->f == f2) {
					nodes2.push_back(p);
				}
				p = p->next;
			}
			cn++;
		}
	}
	return cn;
}

void trackNodes2Arr(std::vector<Track2DNode*>& nodes , int i1 , int i2 , double* pts) {
	size_t i, n = i2 - i1;
	for (i = 0; i < n; ++i) {
		pts[2 * i] = nodes[i1 + i]->x;
		pts[2 * i + 1] = nodes[i1 + i]->y;
	}
}
