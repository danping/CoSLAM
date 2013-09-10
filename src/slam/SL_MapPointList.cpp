/*
 * SL_MapPointList.cpp
 *
 *  Created on: 2011-1-17
 *      Author: Danping Zou
 */

#include "SL_MapPointList.h"

#include "SL_error.h"
#include <cassert>

MapPointList::MapPointList() :
	num(0), over_head(), pTail(0) {

}
MapPointList::~MapPointList() {
	clear();
}
void MapPointList::clear() {
	MapPoint* pt = over_head.next;
	while (pt) {
		MapPoint* q = pt;
		pt = pt->next;
		delete q;
	}
	num = 0;
	over_head.pre = 0;
	over_head.next = 0;
	pTail = 0;
}
void MapPointList::clearWithoutRelease() {
	num = 0;
	over_head.pre = 0;
	over_head.next = 0;
	pTail = 0;
}
MapPoint* MapPointList::add(double x, double y, double z, int frame) {
	if (frame < 0) {
		repErr("Map::add() - error");
	}
	MapPoint* pt = new MapPoint(x, y, z, frame);
	return add(pt);
}

MapPoint* MapPointList::add(double* xyz, int iSlice) {
	return add(xyz[0], xyz[1], xyz[2], iSlice);
}
MapPoint* MapPointList::add(MapPoint* pt) {
	if (over_head.next == 0) {
		pt->next = 0;
		over_head.next = pt;
		pt->pre = &over_head;
	} else {
		pTail->next = pt;
		pt->pre = pTail;
		pt->next = 0;
	}
	pTail = pt;
	++num;
	return pt;
}
void MapPointList::addN(int frame, int npts, double* xyz) {
	assert( npts > 0);
	int i;
	for (i = 0; i < npts; ++i) {
		add(xyz + 3 * i, frame);
	}
}
MapPoint* MapPointList::remove(MapPoint* pt) {
	MapPoint* prev = pt->pre;
	assert(prev);
	prev->next = pt->next;
	if (pt->next)
		pt->next->pre = prev;
	pt->pre = 0;
	pt->next = 0;

	if (pt == pTail)
		pTail = prev;

	--num;
	return prev;
}
void MapPointList::print() const {
	logInfo("----%d points --- \n", num);
	const MapPoint* pt = over_head.next;
	while (pt) {
		logInfo("[%d](%lf,%lf,%lf) -- ", pt->lastFrame, pt->x, pt->y, pt->z);
		for (int i = 0; i < SLAM_MAX_NUM; i++)
			if (pt->pFeatures[i]) {
				logInfo(" %d(%d)", i, pt->pFeatures[i]);
			}
		logInfo("\n");
		pt = pt->next;
	}
}
//#include "SL_Tictoc.h"
//int main() {
//	//	MapPoint pt;
//	//	pt.x = 1;
//	//	pt.y = 2.0;
//	//	info("%lf,%lf", pt.data[0], pt.data[1]);
//	MapPointList mp;
////	int idx1 = 0;
//	
//	mp.add(1, 1, 1, 0);
//	mp.add(2, 2, 2, 0);
//	mp.add(3, 3, 3, 0);
//	mp.add(4, 4, 4, 3);
//	mp.add(5, 5, 5, 2);
//	MapPoint* pt = mp.add(5, 5, 5, 1);
//	mp.add(6, 6, 6, 3);
//	mp.add(7, 7, 7, 0);
//	mp.add(8, 8, 8, 2);
//
//	mp.print();
//	MapPoint* p = mp.getHead();
//	for (; p != mp.getTail(); p = p->next) {
//		if (p->lastFrame == 2){
//			MapPoint* q = p;
//			p = mp.remove(p);
//			delete q;
//		}
//	}
//	
//	MapPointList& mp0 = mp;
//	MapPointList mp1;
//	mp = mp1;
//
//	mp.print();
//
//	logInfo("after removing frame\n");
//	mp.print();
//	mp.add(9, 9, 9, 3);
//	mp.print();
//	return 0;
//}
