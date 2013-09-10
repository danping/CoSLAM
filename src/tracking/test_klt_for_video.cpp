/*
 Copyright (c) 2008-2010 UNC-Chapel Hill & ETH Zurich

 This file is part of GPU-KLT+FLOW.

 GPU-KLT+FLOW is free software: you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation, either version 3 of the License, or (at your option) any
 later version.

 GPU-KLT+FLOW is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with GPU-KLT+FLOW. If not, see <http://www.gnu.org/licenses/>.
 */

#include <GL/glew.h>
#include <GL/glut.h>
#include "CGKLT/v3d_gpupyramid.h"
#include "CGKLT/v3d_gpuklt.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <cstdio>
#include <vector>

using namespace std;
using namespace V3D_GPU;

//#define PREFETCH_VIDEO 1

namespace {

struct PointTrack {
	PointTrack() :
		len(0) {
	}

	void add(float X, float Y) {
		pos[len][0] = X;
		pos[len][1] = Y;
		++len;
	}

	void clear() {
		len = 0;
	}
	bool isValid() const {
		return pos[len - 1][0] >= 0;
	}

	int len;
	float pos[4096][2];
}; // end struct PointTrack

CvCapture * capture = 0;
int width, height;

bool trackWithGain = false;
int const featuresWidth = 32;
int const featuresHeight = 32;
unsigned int const nFeatures = featuresWidth * featuresHeight;
int const nTrackedFrames = 5;
int const nTimedFrames = 400;
int const nLevels = 3;
int const pointListWidth = 64;
int const pointListHeight = 64;

int win, scrwidth, scrheight;
bool initialized = false;

KLT_SequenceTracker * tracker = 0;
vector<IplImage *> allFrames;
vector<float> trueGains;

void reshape(int width, int height) {
	cout << "reshape" << endl;

	scrwidth = width;
	scrheight = height;
	glViewport(0, 0, (GLint) width, (GLint) height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-10, 1010, 10, 1010);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void done();

void drawscene() {
	static int frameNo = 0;

	static int nDetectedFeatures = 0;
	static IplImage * videoFrame;
	static PointTrack * tracks;
	static KLT_TrackedFeature * features = 0;

	if (!initialized) {
		CvSize sz = cvSize(width, height);
		videoFrame = cvCreateImage(sz, 8, 1);

		features = new KLT_TrackedFeature[nFeatures];
		tracks = new PointTrack[nFeatures];

		glewInit();
		Cg_ProgramBase::initializeCg();

		KLT_SequenceTrackerConfig cfg;
		cfg.nIterations = 10;
		cfg.nLevels = nLevels;
		cfg.levelSkip = 1;
		cfg.trackBorderMargin = 20.0f;
		cfg.convergenceThreshold = 0.1f;
		cfg.SSD_Threshold = 3000.0f;
		cfg.trackWithGain = true;

		cfg.minDistance = 8;
		cfg.minCornerness = 10.0f;
		cfg.detectBorderMargin = cfg.trackBorderMargin;

		tracker = new KLT_SequenceTracker(cfg);
		tracker->allocate(width, height, nLevels, featuresWidth, featuresHeight);

		initialized = true;
		cout << "Done with initialization." << endl;
	}

#if !defined(PREFETCH_VIDEO)
	IplImage * im = cvQueryFrame(capture);
	if (im)
		cvCvtColor(im, videoFrame, CV_RGB2GRAY);
	else {
		cerr << "Could not read frame from video source. Exiting..." << endl;
		done();
	}
#else
	//if (frameNo >= allFrames.size()) done();
	videoFrame = allFrames[frameNo % allFrames.size()];
#endif

	int const relFrameNo = (frameNo % nTrackedFrames);

	if (frameNo == 0) {
		tracker->detect((V3D_GPU::uchar *) videoFrame->imageData, nDetectedFeatures, features);
		cout << "nDetectedFeatures = " << nDetectedFeatures << endl;
	} else if (relFrameNo == 0) {
		int nNewFeatures;
		tracker->redetect((V3D_GPU::uchar *) videoFrame->imageData, nNewFeatures, features);
		cout << "nNewFeatures = " << nNewFeatures << endl;
	} else {
		int nPresentFeatures;
		tracker->track((V3D_GPU::uchar *) videoFrame->imageData, nPresentFeatures, features);
		//cout << "nPresentFeatures = " << nPresentFeatures << endl;
	}

	int nTracks = 0;
	for (size_t i = 0; i < nFeatures; ++i) {
		if (features[i].status == 0) {
			tracks[i].add(features[i].pos[0], features[i].pos[1]);
			++nTracks;
		} else if (features[i].status > 0) {
			tracks[i].len = 1;
			tracks[i].pos[0][0] = features[i].pos[0];
			tracks[i].pos[0][1] = features[i].pos[1];
		} else
			tracks[i].clear();
	} // end for (i)

	//cout << "nTracks = " << nTracks << endl;

	glFinish();

	// Draw texture.
	FrameBufferObject::disableFBORendering();
	glViewport(0, 0, scrwidth, scrheight);
	setupNormalizedProjection(true);

	glColor3f(1, 1, 1);
	glActiveTexture(GL_TEXTURE0_ARB);
	glBindTexture(GL_TEXTURE_2D, tracker->getCurrentFrameTextureID());
	glEnable(GL_TEXTURE_2D);
	renderNormalizedQuad();
	glDisable(GL_TEXTURE_2D);

#if 1
	// Draw lines.
	glColor3f(1, 0.3, 0.2);
	for (size_t i = 0; i < nFeatures; ++i) {
		if (tracks[i].len > 1 && tracks[i].isValid()) {
			glBegin(GL_LINE_STRIP);
			for (int j = 0; j < tracks[i].len; ++j)
				glVertex2fv(tracks[i].pos[j]);
			glEnd();
		}
	} // end for (i)

	// Draw newly created corners.
	glPointSize(5);
	glColor3f(0.2, 0.3, 1.0);
	glBegin(GL_POINTS);
	for (size_t i = 0; i < nFeatures; ++i) {
		if (features[i].status > 0)
			glVertex2fv(features[i].pos);
	} // end for (i)
	glEnd();

	glColor3f(0.2, 1.0, 0.3);
	glBegin(GL_POINTS);
	for (size_t i = 0; i < nFeatures; ++i) {
		if (features[i].status == 0)
			glVertex2fv(features[i].pos);
	} // end for (i)
	glEnd();
#endif

	tracker->advanceFrame();
	++frameNo;

	glutSwapBuffers();
} // end drawscene()

void done() {
	FrameBufferObject::disableFBORendering();
	tracker->deallocate();
	exit(0);
}

void keyFunc(unsigned char key, int x, int y) {
	if (key == 27)
		done();
	switch (key) {
	case ' ':
		break;
	}
	glutPostRedisplay();
}

} // end namespace <>

int test_main(int argc, char** argv) {
	unsigned int win;

	glutInitWindowPosition(0, 0);
	glutInitWindowSize(640, 480);
	glutInit(&argc, argv);

	//   if (argc != 3)
	//   {
	//      cout << "Usage: " << argv[0] << " <video> <trackWithGain>" << endl;
	//      return -1;
	//   }

	//capture = cvCreateFileCapture(argv[1]);
	capture = cvCreateFileCapture("/home/Danping Zou/Video/2010-12-24/MVI_0985.avi");

	trackWithGain = atoi(argv[2]);

	width = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	height = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);

	cout << "w = " << width << ", h = " << height << endl;

#if defined(PREFETCH_VIDEO)
	cout << "Reading all frames from the video..." << endl;
	allFrames.reserve(1000);
	{
		CvSize sz = cvSize(width, height);

		for (;;)
		{
			IplImage * im = cvQueryFrame(capture);
			if (im)
			{
				IplImage * im1 = cvCreateImage(sz, 8, 1);
				cvCvtColor(im, im1, CV_RGB2GRAY);
				allFrames.push_back(im1);
			}
			else
			break;
		}
		cout << "allFrames.size() = " << allFrames.size() << endl;
	}
#endif

	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

	if (!(win = glutCreateWindow("GPU KLT Test"))) {
		cerr << "Error, couldn't open window" << endl;
		return -1;
	}

	glutReshapeFunc(reshape);
	glutDisplayFunc(drawscene);
	glutIdleFunc(drawscene);
	glutKeyboardFunc(keyFunc);
	glutMainLoop();

	return 0;
}
//#include "SL_Tictoc.h"
//int main(int argc, char** argv) {
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
//	glutCreateWindow("test");
//	//open video file
//	cv::VideoCapture capture;
//	if (!capture.open("/media/工作学习/SLAM Video/newdynamic13/52/052_out.avi")) {
//		fprintf(stderr, "cannot open the video source");
//	}
//
//	width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
//	height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
//	int videoLen = capture.get(CV_CAP_PROP_FRAME_COUNT);
//
//	//initialize the tracker
//	int nFeatures = 32 * 32;
//	KLT_TrackedFeature* features[3];
//	PointTrack* tracks[3];
//	for (int l = 0; l < 3; l++) {
//		tracks[l] = new PointTrack[nFeatures];
//		features[l] = new KLT_TrackedFeature[nFeatures];
//	}
//
//	glewInit();
//	Cg_ProgramBase::initializeCg();
//
//	KLT_SequenceTrackerConfig cfg;
//	cfg.nIterations = 10;
//	cfg.nLevels = 4;
//	cfg.levelSkip = 1;
//	cfg.trackBorderMargin = 5.0f;
//	cfg.convergenceThreshold = 0.1f;
//	cfg.SSD_Threshold = 3000.0f;
//	cfg.trackWithGain = true;
//
//	cfg.minDistance = 5;
//	cfg.minCornerness = 1500.0f;
//	cfg.detectBorderMargin = cfg.trackBorderMargin;
//
//	int minDist[3] = { 8, 6, 4 };
//	KLT_SequenceTracker* tracker[3];
//	int zoom = 1;
//	for (int i = 0; i < 3; i++) {
//		cfg.minDistance = minDist[i];
//		cfg.nLevels = 3 - i;
//		tracker[i] = new KLT_SequenceTracker(cfg);
//		tracker[i]->allocate(width / zoom, height / zoom, cfg.nLevels, featuresWidth, featuresHeight);
//		zoom *= 2;
//		printf("zoom:%d\n", zoom);
//	}
//
//	cv::Mat gray(height, width, CV_8U);
//	tic();
//	int k = 0;
//	for (;;) {
//		if (!capture.grab()) {
//			printf("end of the file\n");
//			break;
//		}
//		cv::Mat rawFrame;
//		cv::Mat videoFrame[3];
//
//		bool bSucc = capture.retrieve(rawFrame);
//		if (!bSucc)
//			break;
//		cv::cvtColor(rawFrame, videoFrame[0], CV_RGB2GRAY);
//		zoom = 2;
//		for (int l = 1; l < 3; l++) {
//			cv::resize(videoFrame[l - 1], videoFrame[l], cv::Size(width / zoom, height / zoom), CV_INTER_NN);
//			zoom *= 2;
//		}
//		for (int l = 0; l < 1; l++) {
//			int frameNo = k;
//			//			cout << k << std::endl;
//			int nDetectedFeatures = 0;
//
//			int const relFrameNo = (frameNo % nTrackedFrames);
//
//			if (frameNo == 0) {
//				tracker[l]->detect((V3D_GPU::uchar *) videoFrame[l].data, nDetectedFeatures, features[l]);
//				cout << "[" << l << "]nDetectedFeatures = " << nDetectedFeatures << endl;
//			} else if (relFrameNo == 0) {
//				int nNewFeatures;
//				tracker[l]->redetect((V3D_GPU::uchar *) videoFrame[l].data, nDetectedFeatures, features[l]);
//				cout << "[" << l << "nNewFeatures = " << nDetectedFeatures << endl;
//			} else {
//				int nPresentFeatures;
//				tracker[l]->track((V3D_GPU::uchar *) videoFrame[l].data, nDetectedFeatures, features[l]);
//				cout << "[" << l << "nPresentFeatures = " << nDetectedFeatures << endl;
//			}
//
//			int nTracks = 0;
//			for (int i = 0; i < nFeatures; ++i) {
//				if (features[l][i].status == 0) {
//					tracks[l][i].add(features[l][i].pos[0] * width, features[l][i].pos[1] * height);
//					++nTracks;
////					cv::circle(videoFrame[0], cv::Point(features[l][i].pos[0] * videoFrame[0].cols,
////							(features[l][i].pos[1]) * videoFrame[0].rows), 3 * (l + 1), CV_RGB(255,255,255));
//				} else if (features[l][i].status > 0) {
//					tracks[l][i].len = 1;
//					tracks[l][i].pos[0][0] = features[l][i].pos[0] * width;
//					tracks[l][i].pos[0][1] = features[l][i].pos[1] * height;
//					++nTracks;
////					cv::circle(videoFrame[0], cv::Point(features[l][i].pos[0] * videoFrame[0].cols,
////							(features[l][i].pos[1]) * videoFrame[0].rows), 3 * (l + 1), CV_RGB(255,255,255));
//				} else
//					tracks[l][i].clear();
//
//				//				printf("%lf,%lf\n",features[i].pos[0],features[i].pos[1]);
//			}
//			tracker[l]->advanceFrame();
//		}
////		cv::imshow("gray", videoFrame[0]);
////		cv::waitKey(-1);
//		cout << k++ << std::endl;
//	}
//	toc();
//	printf("number of frames:%d", videoLen);
//	return 0;
//}

//#include "SL_Tictoc.h"
//int main(int argc , char** argv) {
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
//	glutCreateWindow("test");
//	//open video file
//	cv::VideoCapture capture;
//	if (!capture.open("/media/工作学习/SLAM Video/newdynamic13/52/052_out.avi")) {
//		fprintf(stderr, "cannot open the video source");
//	}
//
//	width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
//	height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
//	int videoLen = capture.get(CV_CAP_PROP_FRAME_COUNT);
//
//	glewInit();
//	Cg_ProgramBase::initializeCg();
//	
//	//initialize the tracker
//	int nFeatures = 32 * 32;
//	KLT_TrackedFeature* features = new KLT_TrackedFeature[nFeatures];
//	PointTrack* tracks = new PointTrack[nFeatures];
//
//	KLT_SequenceTrackerConfig cfg;
//	cfg.nIterations = 5;
//	cfg.nLevels = 3;
//	cfg.levelSkip = 1;
//	cfg.trackBorderMargin = 5.0f;
//	cfg.convergenceThreshold = 0.1f;
//	cfg.SSD_Threshold = 7000.0f;
//	cfg.trackWithGain = false;
//
//	cfg.minDistance = 5;
//	cfg.minCornerness = 500.0f;
//	cfg.detectBorderMargin = cfg.trackBorderMargin;
//
//	KLT_SequenceTracker* tracker = new KLT_SequenceTracker(cfg);
//	tracker->allocate(width, height, cfg.nLevels, featuresWidth, featuresHeight);
//
//	cv::Mat gray(height, width, CV_8U);
//	tic();
//	int k = 0;
//	for (;;) {
//		if (!capture.grab()) {
//			printf("end of the file\n");
//			break;
//		}
//		cv::Mat rawFrame;
//		cv::Mat videoFrame;
//
//		bool bSucc = capture.retrieve(rawFrame);
//		if (!bSucc)
//			break;
//		cv::cvtColor(rawFrame, videoFrame, CV_RGB2GRAY);
//
//		int frameNo = k;
//		int nDetectedFeatures = 0;
//		int const relFrameNo = (frameNo % nTrackedFrames);
//
//		if (frameNo == 0) {
//			tracker->detect((V3D_GPU::uchar *) videoFrame.data, nDetectedFeatures, features);
//			cout << "nDetectedFeatures = " << nDetectedFeatures << endl;
//		} else if (relFrameNo == 0) {
//			tracker->redetect((V3D_GPU::uchar *) videoFrame.data, nDetectedFeatures, features);
//			cout << "nNewFeatures = " << nDetectedFeatures << endl;
//		} else {
//			tracker->track((V3D_GPU::uchar *) videoFrame.data, nDetectedFeatures, features);
//			cout << "nPresentFeatures = " << nDetectedFeatures << endl;
//		}
//
//		int nTracks = 0;
//		for (int i = 0; i < nFeatures; ++i) {
//			if (features[i].status == 0) {
//				tracks[i].add(features[i].pos[0] * width, features[i].pos[1] * height);
//				++nTracks;
////				cv::circle(videoFrame, cv::Point(features[i].pos[0] * videoFrame.cols, (features[i].pos[1])
////						* videoFrame.rows), 3, CV_RGB(255,255,255));
//			} else if (features[i].status > 0) {
//				tracks[i].len = 1;
//				tracks[i].pos[0][0] = features[i].pos[0] * width;
//				tracks[i].pos[0][1] = features[i].pos[1] * height;
//				++nTracks;
////				cv::circle(videoFrame, cv::Point(features[i].pos[0] * videoFrame.cols, (features[i].pos[1])
////						* videoFrame.rows), 6 , CV_RGB(0,0,255));
//			} else
//				tracks[i].clear();
//
//			//printf("%lf,%lf\n",features[i].pos[0],features[i].pos[1]);
//		}
//		tracker->advanceFrame();
////		cv::imshow("gray", videoFrame);
////		cv::waitKey(-1);
//		cout << k++ << std::endl;
//	}
//	toc();
//	printf("number of frames:%d", videoLen);
//	return 0;
//}
