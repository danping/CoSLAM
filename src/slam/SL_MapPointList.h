/*
 * SL_MapPointList.h
 *
 *  Created on: 2011-1-17
 *      Author: Danping Zou
 */

#ifndef SL_MAPPOINTLIST_H_
#define SL_MAPPOINTLIST_H_
#include "SL_MapPoint.h"

class MapPointList {
protected:
	int num; //number of points
	MapPoint over_head; //map head
	MapPoint* pTail; 	//pointer to the last point insert to the map
public:
	MapPointList();
	~MapPointList();
public:
	void clear();
	void clearWithoutRelease();
	MapPoint* add(double x, double y, double z, int frame);
	MapPoint* add(double* xyz, int frame);
	MapPoint* add(MapPoint* pt);
	void addN(int frame, int npts, double* xyz);
public:
	int getNum() const {
		return num;
	}
	int count() const {
		int k = 0;
		for (MapPoint* mpt = over_head.next; mpt; mpt = mpt->next)
			k++;
		return k;
	}
	MapPoint* getHead() const {
		return over_head.next;
	}
	MapPoint* getTail() const {
		return pTail;
	}
	MapPoint* remove(MapPoint* pt);
public:
	//Debug
	void print() const;
};

#endif /* SL_MAPPOINTLIST_H_ */
