/*
 * GPUKLT.cpp
 *
 *  Created on: Mar 24, 2011
 *      Author: Danping Zou
 */

#include "GPUKLT.h"
#include "SL_error.h"
#include "slam/SL_Define.h"
#include "geometry/SL_Distortion.h"


GPUKLT::GPUKLT() :
		m_camId(-1), m_frame(0), m_tks(0), _tracker(0), _features(0) {

}
GPUKLT::~GPUKLT() {
	clear();
}
void GPUKLT::clear() {
	if (m_tks) {
		delete[] m_tks;
		m_tks = 0;
	}
	if (_tracker) {
		_tracker->deallocate();
		delete _tracker;
		_tracker = 0;
	}
	if (_features) {
		delete[] _features;
		_features = 0;
	}
}
void GPUKLT::addToFeaturePoints(int nFeatures,
		V3D_GPU::KLT_TrackedFeature* features, FeaturePoints& ips) {
	double in[2], out[2];
	for (int i = 0; i < nFeatures; i++) {
		if (features[i].status >= 0) {
			assert(features[i].fed < 0);
			//remove the distortion first
			in[0] = features[i].pos[0] * m_W;
			in[1] = features[i].pos[1] * m_H;
			undistorPoint(m_K.data, m_kud.data, in, out);
			if (out[0] >= m_W || out[1] >= m_H)
				continue;
			FeaturePoint* p = ips.add(m_frame, m_camId, out[0], out[1]);
			if (features[i].status == 0) {
				//tracked
				m_tks[i].add(p);
			} else {
				//newly detected;
				m_tks[i].clear();
				m_tks[i].add(p);
			}
		} else
			m_tks[i].clear();
	}
}
void GPUKLT::addToFeaturePoints(int nFeatures,
		V3D_GPU::KLT_TrackedFeature* features,
		std::vector<FeaturePoint*>& pExistFeat, FeaturePoints& ips) {
	double in[2], out[2];
	for (int i = 0; i < nFeatures; i++) {
		if (features[i].status >= 0) {
			//remove the distortion first
			in[0] = features[i].pos[0] * m_W;
			in[1] = features[i].pos[1] * m_H;
			undistorPoint(m_K.data, m_kud.data, in, out);
			if (out[0] >= m_W || out[1] >= m_H)
				continue;

			FeaturePoint* p = 0;
			if (features[i].fed < 0)
				p = ips.add(m_frame, m_camId, out[0], out[1]);
			else
				p = pExistFeat[features[i].fed];
			if (features[i].status == 0) {
				//tracked
				m_tks[i].add(p);
			} else {
				//newly detected;
				m_tks[i].clear();
				m_tks[i].add(p);
			}
		} else
			m_tks[i].clear();
	}
}
void GPUKLT::init(int camId, int W, int H,
		V3D_GPU::KLT_SequenceTrackerConfig* pCfg) {
	assert(pCfg);
	m_camId = camId;
	m_W = W;
	m_H = H;

	const int featuresWidth = SLAM_FEATURE_WIDTH;
	const int featuresHeight = SLAM_FEATURE_HEIGHT;

	if (!_tracker) {
		_tracker = new V3D_GPU::KLT_SequenceTracker(*pCfg);
		_tracker->allocate(m_W, m_H, pCfg->nLevels, featuresWidth,
				featuresHeight);
	}
	m_nMaxCorners = featuresWidth * featuresHeight;
	m_tks = new Track2D[m_nMaxCorners];
	_features = new V3D_GPU::KLT_TrackedFeature[m_nMaxCorners];
}

int GPUKLT::first(int f, const unsigned char* imgData,
		std::vector<FeaturePoint*>& pPresentPts, FeaturePoints& ips) {
	if (pPresentPts.size() == 0)
		return first(f, imgData, ips);

	int nPresent = pPresentPts.size();
	float* corners = new float[nPresent * 3];
	for (int i = 0; i < nPresent; i++) {
		corners[3 * i] = pPresentPts[i]->x / m_W;
		corners[3 * i + 1] = pPresentPts[i]->y / m_H;
		corners[3 * i + 2] = 0;
	}
	m_frame = f;
	int nDetectedFeatures = 0;
	_tracker->detect(imgData, nDetectedFeatures, _features, nPresent, corners);
	_tracker->advanceFrame();

	addToFeaturePoints(nDetectedFeatures, _features, pPresentPts, ips);

	delete[] corners;
	return nDetectedFeatures + nPresent;
}
int GPUKLT::first(int f, const unsigned char* imgData, FeaturePoints& ips) {
	m_frame = f;
	int nDetectedFeatures = 0;
	_tracker->detect(imgData, nDetectedFeatures, _features);
	_tracker->advanceFrame();
	if (nDetectedFeatures == 0)
		repErr("No feature points has been detected!");
	addToFeaturePoints(m_nMaxCorners, _features, ips);
	return nDetectedFeatures;
}

int GPUKLT::next(const unsigned char* imgData, FeaturePoints& ips) {
	assert(_tracker);
	m_frame++;
	int nDetectedFeatures = 0;
	const int nTrackedFrames = 1;
	if (m_frame % nTrackedFrames == 0) {
		//re-detect feature points
		_tracker->redetect(imgData, nDetectedFeatures, _features);
		addToFeaturePoints(m_nMaxCorners, _features, ips);
	} else {
		_tracker->track(imgData, nDetectedFeatures, _features);
		if (nDetectedFeatures == 0)
			repErr("No feature points is tracked!");
		addToFeaturePoints(m_nMaxCorners, _features, ips);
	}
	_tracker->advanceFrame();
	return nDetectedFeatures;
}

int GPUKLT::feedExternFeatPoints(std::vector<FeaturePoint*>& pExternPts) {
	int npts = pExternPts.size();
	float* corners = new float[npts * 3];

	for (int i = 0; i < npts; i++) {
		corners[3 * i] = pExternPts[i]->x / m_W;
		corners[3 * i + 1] = pExternPts[i]->y / m_H;
		corners[3 * i + 2] = 0;
	}

	int* trkIds = new int[m_nMaxCorners];
	int nFed = 0;
	_tracker->feedExternFeaturePoints(npts, corners, trkIds, nFed);

	//initialize tracks
	for (int k = 0; k < nFed; k++) {
		int tkid = trkIds[k];
		assert(tkid >= 0);
		m_tks[tkid].clear();
		m_tks[tkid].add(pExternPts[k]);
	}

//	//test
//	logInfo("%d extern feature points are fed!",nFed);
	_tracker->advanceFrame();
	delete[] corners;
	return nFed;
}
void GPUKLT::detectCorners(int W, int H, const unsigned char* imgData,
		Mat_f& corners, float minCornerness, int minDistance) {
	m_W = W;
	m_H = H;

	V3D_GPU::KLT_SequenceTrackerConfig cfg;
	cfg.nLevels = 1;
	cfg.minDistance = minDistance;
	cfg.minCornerness = minCornerness;

	const int featuresWidth = 32;
	const int featuresHeight = 32;

	if (_tracker)
		clear();
	else {
		_tracker = new V3D_GPU::KLT_SequenceTracker(cfg);
		_tracker->allocate(m_W, m_H, cfg.nLevels, featuresWidth,
				featuresHeight);
	}

	m_nMaxCorners = featuresWidth * featuresHeight;
	_features = new V3D_GPU::KLT_TrackedFeature[m_nMaxCorners];

	int nDetectedFeatures = 0;
	_tracker->detect(imgData, nDetectedFeatures, _features);
	if (!nDetectedFeatures)
		repErr("No feature points has been detected!");

	corners.resize(nDetectedFeatures, 2);
	for (int i = 0, k = 0; i < m_nMaxCorners; i++) {
		if (_features[i].status < 0)
			continue;assert(
				_features[i].pos[0] < 1.0 && _features[i].pos[1] < 1.0);

		corners.data[2 * k] = _features[i].pos[0] * m_W;
		corners.data[2 * k + 1] = _features[i].pos[1] * m_H;
		k++;
	}
}

void GPUKLT::getCurrentCorners(Mat_d& corners) {
	corners.resize(m_nMaxCorners, 2);
	int k = 0;
	for (int i = 0; i < m_nMaxCorners; i++) {
		if (!m_tks[i].empty() && m_tks[i].tail->f == m_frame) {
			corners.data[2 * k] = m_tks[i].tail->x;
			corners.data[2 * k + 1] = m_tks[i].tail->y;
			k++;
		}
	}
	corners.rows = k;
}

//#include <GL/glew.h> 
//#include <GL/glut.h>
//#include "SL_VideoReader.h"
//#include "SL_WriteRead.h"
//#include "math/SL_LinAlg.h"
//#include "geometry/SL_Distortion.h"
//#include "SL_DrawCorners.h"
//#include "GUI_ImageViewer.h"
//#include "SL_DrawCorners.h"
//
//static void drawTracks(ImgRGB& rgb, int f, Track2D* pTracks, int nTracks) {
//	for (int i = 0; i < nTracks; i++) {
//		if (!pTracks[i].empty() && pTracks[i].f2 >= f) {
//			drawTrack2D(rgb, pTracks[i], 255, 0, 0, 1.0, 10);
//		}
//	}
//}
//#include "math/SL_LinAlgWarper.h"
//int main(int argc, char** argv) {
//	try {
//		using namespace V3D_GPU;
//
//		glutInit(&argc, argv);
//		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
//		glutCreateWindow("test");
//		glewInit();
//		Cg_ProgramBase::initializeCg();
//
//		VideoReader vr;
//		vr.open("/media/VIDEO_DATA/smallscale.avi");
//
//		int W = vr._w;
//		int H = vr._h;
//
//		//load camera parameters
//		Mat_d K, k_c, iK, k_ud;
//		K.resize(3, 3);
//		k_c.resize(5, 1);
//		readIntrinDistParam("/media/VIDEO_DATA/sonycx700E/sonycal.txt", K, k_c);
//		iK.resize(3, 3);
//		k_ud.resize(7, 1);
//		matInv(3, K.data, iK.data);
//		invDistorParam(vr._w, vr._h, iK.data, k_c.data, k_ud.data);
//
//		GPUKLT kltTracker;
//		kltTracker.init(0, W, H, 0);
//		kltTracker.setIntrinsicParam(K.data, iK.data, k_ud.data);
//
//		ImgG img(W, H);
//		ImgRGB rgb(W, H);
//		vr.nextFrame();
//		vr.getCurGrayImage(img.data);
//
//		Mat_d pts;
//		FeaturePoints fps;
//
//		Mat_d corners01;
//		matRand(100, 2, corners01);
//		matScale(corners01, H * 0.8);
//		matAddScale(corners01, H * 0.5);
//
//		std::vector<FeaturePoint*> existingPts;
//		for (int i = 0; i < corners01.rows; i++) {
//			FeaturePoint* pPt = fps.add(0, 0, corners01.data[2 * i], corners01.data[2 * i + 1]);
//			existingPts.push_back(pPt);
//		}
//		drawFeatPoints(img, 0, existingPts, rgb);
//		imshow("frame0", rgb);
//		cv::waitKey(-1);
//
//		int nPts = kltTracker.first(0, img.data, existingPts, fps);
//		logInfo("[%d]npts : %d\n", 0, nPts);
//
//		gray2rgb(img, rgb);
//		drawTracks(rgb, 0, kltTracker.m_tks, kltTracker.m_nMaxCorners);
//
//		for (int f = 1; f < 3000; f++) {
//			vr.nextFrame();
//			vr.getCurGrayImage(img.data);
//			int nPts = 0;
//			if (f % 5 == 0)
//				kltTracker.feedExternFeatPoints(existingPts);
//			kltTracker.next(img.data, fps);
//
//			logInfo("[%d]npts : %d\n", f, fps.frameNum[f]);
//
//			gray2rgb(img, rgb);
//			drawTracks(rgb, f, kltTracker.m_tks, kltTracker.m_nMaxCorners);
//			imshow("test1", rgb);
//			cv::waitKey(0.05);
//		}
//	} catch (SL_Exception& e) {
//		logInfo(e.what());
//	}
//	return 0;
//}
//
