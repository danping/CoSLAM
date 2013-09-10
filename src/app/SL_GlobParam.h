/*
 * SL_GlobParam.h
 *
 *  Created on: 2011-7-26
 *      Author: zou
 */

#ifndef SL_GLOBPARAM_H_
#define SL_GLOBPARAM_H_
#include <string>
#include <vector>
using namespace std;
class Param {
public:
	/* number of cameras*/
	static int nCam;
	/* total number of frames*/
	static int nTotalFrame;
	/*minimum feature track length for tri-angulation*/
	static int nMinFeatTrkLen;
	/*maximum reprojection error*/
	static double maxErr;
	/*maximum camera distance between cameras in the same group*/
	static double maxDistRatio;
	/*maximum map points in each frame*/
	static int nMaxMapPts;
	
	static vector<string> videoFilePath;
	static vector<string> camFilePath;
    static vector<int> nSkipFrms;
    static vector<int> nInitFrms;
	
	/* KLT parameters for tracking*/
	static float minCornerness; //*feature detection
    static int minDistance;
	static int nLevels;
	static int windowWidth;
	static float convergeThreshold;
	static float SSD_Threshold; //*tracking
	static bool trackWithGain;
};

class Const {
public:
	static double MAX_EPI_ERR;
	static double PIXEL_ERR_VAR;
};
#endif /* SL_GLOBPARAM_H_ */
