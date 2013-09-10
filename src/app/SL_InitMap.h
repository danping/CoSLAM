/*
 * MapInitializer.h
 *
 *  Created on: 2011-1-4
 *      Author: Danping Zou
 */

#ifndef SL_STEREOMATCH_H_
#define SL_STEREOMATCH_H_
#include "imgproc/SL_Image.h"
#include "tracking/SL_Track2D.h"
#include "math/SL_Matrix.h"
#include "geometry/SL_Point.h"
#include "tools/cvHelper.h"
#include "app/SL_GlobParam.h"
#include "SL_SingleSLAM.h"
#include "slam/SL_MapPointList.h"

class InitMapParam {
public:
	double epiErrMax; //maximum epipolar error
	int minInlierNum; //minimum number of inliers
	int ransacIterNum; //number of iteration for RANSAC
	double robustInlierRatio;
	double dispThres; //displacement threshold
public:
	InitMapParam() {
		epiErrMax = Const::MAX_EPI_ERR;
		minInlierNum = 8;
		ransacIterNum = 1000;
		robustInlierRatio = 0.7;
		dispThres = 70;
	}
};
class InitMap: public InitMapParam {
public:
	const ImgRGB* m_pImgRGB[SLAM_MAX_NUM];
	const ImgG* m_pImgGray[SLAM_MAX_NUM];
	const ImgG* m_pImgGraySmall[SLAM_MAX_NUM];
	double m_imgScale[SLAM_MAX_NUM];

	KpVec m_cvSurfPts[SLAM_MAX_NUM];

	Mat_d m_surfPts[SLAM_MAX_NUM]; //key points after removing the distortion 
	Mat_d m_normSurfPts[SLAM_MAX_NUM]; //key points on normalized image planes

	Mat_f m_surfDesc[SLAM_MAX_NUM]; //store the undistorted surf feature corners
	Matching m_surfMatches[SLAM_MAX_NUM][SLAM_MAX_NUM]; //store the normalized surf feature corners

	Mat_d m_corners[SLAM_MAX_NUM]; //store the undistorted corner points
	Mat_i m_cornerFlag[SLAM_MAX_NUM];
	Mat_d m_normCorners[SLAM_MAX_NUM]; //store the normalized corner points

	Matching m_cornerMatches[SLAM_MAX_NUM][SLAM_MAX_NUM];

	Mat_i camOrder;

	Track2D m_surfTracks[SLAM_MAX_TRACKNUM];
	Track2D m_cornerTracks[SLAM_MAX_TRACKNUM];

	int m_numSurfTracks;
	int m_numCornerTracks;

	std::map<const Track2D*, Point3d> m_3Dpts;

	Mat_d F[SLAM_MAX_NUM][SLAM_MAX_NUM];

	double K[SLAM_MAX_NUM][9];
	double kd[SLAM_MAX_NUM][5];

	double invK[SLAM_MAX_NUM][9];
	double invkd[SLAM_MAX_NUM][7];

	double camR[SLAM_MAX_NUM][9];
	double camT[SLAM_MAX_NUM][3];

	int numCams;
public:
	InitMap();
	~InitMap();
protected:
	void _surfMatch2Ind(int iCam, int jCam, int num, Mat_i& indices);
	void _cornerMatch2Ind(int iCam, int jCam, int num, Mat_i& indices);
	void _refineSurfMatches(int iCam, int jCam, const Matching& surfMatches,
			Matching& surfMatchesRefined);
public:
	void detectSurfFeats(int iCam);
	void detectCorners(int iCam);

	int estimateFmat(const Mat_d& pts1, const Mat_d& pts2, Mat_d& F,
			Mat_uc& inlierFlag);

	/* add cameras*/
	void addCam(const SingleSLAM& cam);
	void addCam(const ImgRGB* imgRGB, const ImgG* imgGray, const ImgG* imgGraySmall,
			const double imgScale, const double* cK, const double* ckd,
			const double* ciK, const double* cikd);

	/* get stereo correspondences between SURF feature points*/
	void matchSurfBetween(int iCam, int jCam);

	/* match corners between views by NCC*/
	void matchCornerNCCBetween(int iCam, int jCam, double minNcc, double maxEpi,
			double maxDisp);

	/* select the image order*/
	void selectImageOrder();
	void getSurfTracks();
	void getCornerTracks();

	/* compute extrinsic parameters of each camera*/
	//void initFirstTwoCameras(std::vector<const Track2D*>& vecAllTracks, std::map<const Track2D*, bool>& flagAllTracks,int order);
	void computeExtrinsic();

	int reconstructTracks(const Track2D* tracks, int numTracks, int minTrackLen,
			int frame0, std::vector<FeaturePoints*>& pFeaturePts,
			MapPointList& mapPts, double maxRpErr = 3.0);
	/* main function for calling*/
	void apply(int frame0, std::vector<FeaturePoints*>& pFeaturePts,
			MapPointList& mapPts);
	/* refine the map by removing points with large reprojection error*/
	int removeOutliers(MapPointList& mapPts, double errMax);
public:
	//for debug
	void drawResult(const ImgG* img1, const ImgG* img2, const KpVec& keyPts1,
			const KpVec& keyPts2, const DMatchVec& matches, const Mat_uc& flag);
	void drawResult(const ImgG* img1, const ImgG* img2, const KpVec& keyPts1,
			const KpVec& keyPts2, const DMatchVec& matches);

	//for debug
	void save(int curFrame, const char* dir_path,
			vector<FeaturePoints*>& featPts, MapPointList& mapPts);
	void _linkFeatMapPoints(int curFrame, vector<FeaturePoints*>& featPts,
			MapPointList& mapPts);
	void load(int curFrame, const char* dir_path,
			vector<FeaturePoints*>& featPts, MapPointList& mapPts);

};
#endif /* SL_STEREOMATCH_H_ */
