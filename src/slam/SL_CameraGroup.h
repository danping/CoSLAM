/*
 * SL_CameraGroup.h
 *
 *  Created on: 2011-5-11
 *      Author: Danping Zou
 */

#ifndef SL_CAMERAGROUP_H_
#define SL_CAMERAGROUP_H_
#include "math/SL_Matrix.h"

class CameraGroup {
public:
	int camIds[SLAM_MAX_NUM];
	int num;
public:
	CameraGroup() :
			num(0) {
	}
	void clear() {
		num = 0;
	}
	void setCams(const Mat_i& ids) {
		num = 0;
		for (int i = 0; i < ids.rows; i++) {
			addCam(ids.data[i]);
		}
	}
	void addCam(int camId) {
		camIds[num++] = camId;
	}
	void copy(const CameraGroup& other) {
		memcpy(camIds, other.camIds, sizeof(int) * SLAM_MAX_NUM);
		num = other.num;
	}
	bool isCameraIn(int camId) {
		for (int i = 0; i < num; i++) {
			if (camIds[i] == camId)
				return true;
		}
		return false;
	}
};

#endif /* SL_CAMERAGROUP_H_ */
