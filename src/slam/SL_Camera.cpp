/*
 * SL_Camera.cpp
 *
 *  Created on: 2010-11-21
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_Camera.h"
#include "math/SL_Matrix.h"

CamPose::CamPose() :
		f(-1), camId(-1) {

}
CamPose::CamPose(const double* R_, const double* t_) :
		f(-1), camId(-1) {
	memcpy(R, R_, sizeof(double) * 9);
	memcpy(t, t_, sizeof(double) * 3);
}
CamPose::CamPose(const CamPose& other) {
	operator =(other);
}
CamPose& CamPose::operator =(const CamPose& other) {
	if (&other != this) {
		f = other.f;
		camId = other.camId;
		memcpy(R, other.R, sizeof(double) * 9);
		memcpy(t, other.t, sizeof(double) * 3);
	}
	return *this;
}
CamPoseItem::CamPoseItem() :
		CamPose(), pre(0), next(0) {
}
CamPoseItem::CamPoseItem(double* R_, double* t_) :
		CamPose(R_, t_), pre(0), next(0) {
}

CamPoseList::CamPoseList() :
		num(0), tail(0) {
}
CamPoseList::~CamPoseList() {
	clear();
}
void CamPoseList::clear() {
	CamPoseItem* p = over_head.next;
	while (p) {
		CamPoseItem* q = p;
		p = p->next;
		delete q;
		q = 0;
	}
	over_head.next = 0;
	tail = 0;
	num = 0;
}
CamPoseItem* CamPoseList::add(int f, int camId, const double* R,
		const double* t) {
	CamPoseItem* cam = new CamPoseItem();
	cam->f = f;
	cam->camId = camId;
	memcpy(cam->R, R, sizeof(double) * 9);
	memcpy(cam->t, t, sizeof(double) * 3);

	if (tail == 0) {
		over_head.next = cam;
	} else {
		tail->next = cam;
		cam->pre = tail;
	}
	tail = cam;
	num++;
	return cam;
}