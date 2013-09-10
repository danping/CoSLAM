/*
 * SL_KeyFrmLst.cpp
 *
 *  Created on: 2010-11-25
 *      Author: Danping Zou
 */

#include "SL_KeyPoseList.h"
#include "SL_error.h"
KeyPoseList::KeyPoseList() :
	num(0), head(), tail(0) {
}

KeyPoseList::~KeyPoseList() {
	clear();
}
void KeyPoseList::clear() {
	KeyPose* p = head.next;
	while (p) {
		KeyPose* q = p;
		p = p->next;
		delete q;
	}
	head.next = 0;
	tail = 0;
	num = 0;
}
KeyPose* KeyPoseList::add(int f, CamPoseItem* cam_) {
	if (!cam_)
		repErr("KeyFrmLst::add() error!");
	if (tail == 0) {
		KeyPose* pose = new KeyPose(f, cam_);
		head.next = pose;
		tail = pose;
	} else {
		if (cam_->f < tail->cam->f)
			repErr("KeyFrmLst::add() cam_->f < tail->cam->f");
		KeyPose* frm = new KeyPose(f, cam_);

		tail->next = frm;
		frm->pre = tail;
		tail = frm;
	}
	num++;
	return tail;
}

KeyFrameList::KeyFrameList() :
	num(0), head(), tail(0) {
}

KeyFrameList::~KeyFrameList() {
	clear();
}
void KeyFrameList::clear() {
	KeyFrame* p = head.next;
	while (p) {
		KeyFrame* q = p;
		p = p->next;
		delete q;
	}
	head.next = 0;
	tail = 0;
	num = 0;
}
KeyFrame* KeyFrameList::add(int frame) {
	if (frame < 0)
		repErr("KeyFrmLst::add() error!");

	if (tail == 0) {
		KeyFrame* frm = new KeyFrame(frame);
		head.next = frm;
		tail = frm;
	} else {
		if (frame < tail->f)
			repErr("KeyFrmLst::add() frame (%d) < tail->cam->f(%d)\n", frame, tail->f);
		KeyFrame* frm = new KeyFrame(frame);
		tail->next = frm;
		frm->prev = tail;
		tail = frm;
	}
	num++;
	return tail;
}