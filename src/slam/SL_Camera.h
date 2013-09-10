/*
 * SL_Camera.h
 *
 *  Created on: 2010-11-21
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_CAMERA_H_
#define SL_CAMERA_H_

class CamPose {
public:
	int f; //frame
	int camId; //camera identity
	double R[9]; //rotation
	double t[3]; //translation
public:
	CamPose();
	CamPose(const double* R_, const double* t_);
	CamPose(const CamPose& other);
	CamPose& operator =(const CamPose& other);
};

class CamPoseItem: public CamPose {
public:
	CamPoseItem* pre;
	CamPoseItem* next;
public:
	CamPoseItem();
	CamPoseItem(double* R_, double* t_);
};
class CamPoseList {
protected:
	//number of cameras
	int num;
	//point to the tail of the camera list
	CamPoseItem* tail;
	//head.nextFrame point to the first camera
	CamPoseItem over_head;
public:
	CamPoseList();
	~CamPoseList();
	void clear();
	CamPoseItem* add(int f, int camId, const double* R, const double* t);

	int size() const {
		return num;
	}
	CamPoseItem* current() const {
		return tail;
	}
	CamPoseItem* first() const {
		return over_head.next;
	}
};
#endif /* SL_CAMERA_H_ */
