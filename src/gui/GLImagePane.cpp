/*
 * GLImagePane.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */
#include "GLImagePane.h"
#include "GLHelper.h"

#include "app/SL_GlobParam.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"
#include "tools/SL_Print.h"
#include "tools/GUI_ImageViewer.h"

#include "slam/SL_CoSLAMHelper.h"

#include "MyApp.h"
#include "GLScenePane.h"
#include <cfloat>

int GLImagePane::s_clicked_camid = -1;
MapPoint* GLImagePane::s_clicked_mappt = 0;
FeaturePoint* GLImagePane::s_clicked_featPt = 0;
static void replaceMapPointFlag(MapPointList& mapPts, int oldFlag,
		int newFlag) {
	MapPoint* pHead = mapPts.getHead();
	if (!pHead)
		return;
	MapPoint* pEnd = mapPts.getTail()->next;
	MapPoint* p = pHead;
	while (p && p != pEnd) {
		if (p->flag == oldFlag) {
			p->flag = newFlag;
		}
		p = p->next;
	}
}
BEGIN_EVENT_TABLE(GLImagePane, wxGLCanvas) EVT_MOTION(GLImagePane::mouseMoved) EVT_LEFT_DOWN(GLImagePane::mouseDown)
EVT_LEFT_UP(GLImagePane::mouseReleased)
EVT_RIGHT_DOWN(GLImagePane::rightClick)
EVT_LEAVE_WINDOW(GLImagePane::mouseLeftWindow)
EVT_SIZE(GLImagePane::resized)
EVT_KEY_DOWN(GLImagePane::keyPressed)
EVT_KEY_UP(GLImagePane::keyReleased)
EVT_MOUSEWHEEL(GLImagePane::mouseWheelMoved)
EVT_PAINT(GLImagePane::render)
END_EVENT_TABLE()

// some useful events to use
void GLImagePane::mouseMoved(wxMouseEvent& event) {
}

void GLImagePane::mouseDown(wxMouseEvent& event) {
	int x = event.GetX();
	int y = event.GetY();

	double fm[2] = { imgWidth * x * 1.0 / getWidth(), imgHeight * y * 1.0
			/ getHeight() };
	FeaturePoint* pFeatPt = searchNearestFeatPt(
			m_pSLAM->slam[m_camId].m_featPts, m_pSLAM->curFrame, fm, 7.0);
	if (pFeatPt) {
		s_clicked_camid = m_camId;
		s_clicked_featPt = pFeatPt;
		s_clicked_mappt = pFeatPt->mpt;
		if (pFeatPt->mpt && pFeatPt->mpt->state == STATE_MAPPOINT_CURRENT) {
			replaceMapPointFlag(m_pSLAM->curMapPts, FLAG_MAPPOINT_TEST4,
					FLAG_MAPPOINT_NORMAL);
			pFeatPt->mpt->flag = FLAG_MAPPOINT_TEST4;
			//		//test
			pFeatPt->mpt->print();
			double M[3], cov[9];
			//		isDynamicPoint(2,pFeatPt->mpt,2.0,M,cov);

			//print reprojection errors
			MapPoint* p = pFeatPt->mpt;
			logInfo("newpt : %s\n", p->bNewPt ? "yes" : "no");
			for (int i = 0; i < SLAM_MAX_NUM; i++) {
				FeaturePoint* fp = p->pFeatures[i];
				if (fp) {
					CamPoseItem* cam = fp->cam;
					double err = reprojErrorSingle(m_pSLAM->slam[i].K.data,
							cam->R, cam->t, p->M, fp->m);
					logInfo("[view#%d]error:%lf  ", i, err);
					double rm[2], var[4], ivar[4];

					project(fp->K, fp->cam->R, fp->cam->t, p->M, rm);
					getProjectionCovMat(fp->K, fp->cam->R, fp->cam->t, p->M,
							p->cov, var, Const::PIXEL_ERR_VAR);
					mat22Inv(var, ivar);

					double err2 = mahaDist2(rm, fp->m, ivar);
					logInfo("maha error:%lf\n", err2);

					printMat(3, 3, cam->R);
					printMat(3, 1, cam->t);

					logInfo(
							"is at the cam#%d's back : '%s'\n----------------------------------------\n",
							i,
							isAtCameraBack(cam->R, cam->t, p->M) ?
									"yes" : "no");
					//get the number of previous frames
					//				int num = 0;
					//				while (fp != 0) {
					//					num++;
					//					fp = fp->preFrame;
					//				}
					//				logInfo("[%d]number :%d\n", i, num);
				}
			}
			if (isStaticPoint(m_pSLAM->numCams, p, 6.0, M, cov, 10)) {
				//test
				printf("static\n");
			}
			if (isDynamicPoint(m_pSLAM->numCams, p, 6.0, M, cov)) {
				//test
				printf("dynamic\n");
			}

			printf("static frame num:%d\n", p->staticFrameNum);
			//test map registration
			//			Mat_d camDist;
			//			m_pSLAM->getCurrentCamDist(camDist);
			//			MyApp::bDebug = true;
			//			int groupId = m_pSLAM->m_groupId[m_camId];
			//			bool bReg = m_pSLAM->currentStaticPointRegisterInGroup(m_pSLAM->m_groups[groupId], camDist, pFeatPt->mpt,
			//					10.0);
			//			logInfo("register:%d\n", bReg ? 1 : 0);
			MyApp::redrawViews();
		}
		MyApp::redrawViews();
	} else {
		s_clicked_camid = -1;
		s_clicked_featPt = 0;
		s_clicked_mappt = 0;
	}
}
void GLImagePane::mouseWheelMoved(wxMouseEvent& event) {
}
void GLImagePane::mouseReleased(wxMouseEvent& event) {
}
void GLImagePane::rightClick(wxMouseEvent& event) {
}
void GLImagePane::mouseLeftWindow(wxMouseEvent& event) {
}
void GLImagePane::keyPressed(wxKeyEvent& event) {
}
void GLImagePane::keyReleased(wxKeyEvent& event) {
}

GLImagePane::GLImagePane(wxFrame* parent, int* args) :
		wxGLCanvas(parent, wxID_ANY, args, wxDefaultPosition, wxDefaultSize,
				wxFULL_REPAINT_ON_RESIZE) {
	m_context = new wxGLContext(this);
	imgData = 0;
	imgWidth = 0;
	imgHeight = 0;

	m_camId = -1;
	m_pSLAM = 0;
	m_imgTexture = 0;
	b_haveImageData = false;
	b_firstRun = true;
	b_drawReprojectionError = false;
}

GLImagePane::~GLImagePane() {
	delete m_context;
	if (!imgData)
		delete[] imgData;
}

void GLImagePane::resized(wxSizeEvent& evt) {
	Refresh();
}

/** Inits the OpenGL viewport for drawing in 2D. */
void GLImagePane::prepare2DViewport(int topleft_x, int topleft_y,
		int bottomrigth_x, int bottomrigth_y) {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
	glEnable(GL_TEXTURE_2D); // textures
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glViewport(topleft_x, topleft_y, bottomrigth_x - topleft_x,
			bottomrigth_y - topleft_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(topleft_x, bottomrigth_x, bottomrigth_y, topleft_y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_LINE_SMOOTH);
}

int GLImagePane::getWidth() {
	return GetSize().x;
}

int GLImagePane::getHeight() {
	return GetSize().y;
}

void GLImagePane::draw() {
	MyApp::bBusyDrawingVideo[m_camId] = true;
	wxGLCanvas::SetCurrent(*m_context);
	wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// ------------- draw some 2D ----------------
	prepare2DViewport(0, 0, getWidth(), getHeight());
	glLoadIdentity();

	//draw video frame
	if (imgData) {
		glEnable(GL_TEXTURE_2D);
		if (b_firstRun) {
			glGenTextures(1, &m_imgTexture); // generate OpenGL texture object
			glBindTexture(GL_TEXTURE_2D, m_imgTexture); // use previously created texture object and set options
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			b_firstRun = false;
		}
		if (s_clicked_camid >= 0) {
			ImgRGB img(imgWidth, imgHeight);
			gray2rgb(imgWidth, imgHeight, imgData, img.data);
			drawClickedPoint(img);
			if (s_clicked_camid != m_camId)
				drawEpipolarLine(img);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0,
					GL_RGB, GL_UNSIGNED_BYTE, img.data);
		} else
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, imgWidth, imgHeight, 0,
					GL_LUMINANCE, GL_UNSIGNED_BYTE, imgData);

		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex3f(0, 0, 0);
		glTexCoord2f(1, 0);
		glVertex3f(getWidth(), 0, 0);
		glTexCoord2f(1, 1);
		glVertex3f(getWidth(), getHeight(), 0);
		glTexCoord2f(0, 1);
		glVertex3f(0, getHeight(), 0);
		glEnd();
		glDisable(GL_TEXTURE_2D);
	} else {
		// white background
		glColor4f(1, 1, 1, 1);
		glBegin(GL_QUADS);
		glVertex3f(0, 0, 0);
		glVertex3f(getWidth(), 0, 0);
		glVertex3f(getWidth(), getHeight(), 0);
		glVertex3f(0, getHeight(), 0);
		glEnd();
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	drawFeaturePoints();

	if (imgData) {
		drawConvexHulls();
		drawInfo(0, 0, getWidth(), getHeight());
	}

	SwapBuffers();
	MyApp::bBusyDrawingVideo[m_camId] = false;
}
void GLImagePane::render(wxPaintEvent& evt) {
	if (!IsShown())
		return;
	draw();
}
void GLImagePane::convert2ImageCoords(float x0, float y0, float& x, float& y) {
	x = x0 / imgWidth * getWidth();
	y = y0 / imgHeight * getHeight();
}
void GLImagePane::drawFeaturePoints() {
	const float COLOR_CORNER_ERR[3] = { 1.0f, 0.0f, 0.0f };
	const float COLOR_CORNER[3] = { 1.0f, 1.0f, 1.0f };

	//glLineWidth(3.0f);
	glLineWidth(2.0f);
	float radius = 1;
	for (size_t k = 0; k < m_pFeatPts.size(); k++) {
		FeaturePoint* p = m_pFeatPts[k];
		float x0 = static_cast<int>(p->x);
		float y0 = static_cast<int>(p->y);
		float x, y;
		//convert to image coordinate
		convert2ImageCoords(x0, y0, x, y);
		//draw projections without corresponding map points
		if (p->type != TYPE_FEATPOINT_DYNAMIC) {
			glColor3f(COLOR_CORNER[0], COLOR_CORNER[1], COLOR_CORNER[2]);
			drawCircle(x, y, radius, 5);
		} else {
			glColor3f(1.0f, 1.0f, 0.0f);
			drawCircle(x, y, radius, 10);
		}

		if (p->mpt && p->mpt->numVisCam >= 1) {
			//draw projections with corresponding map points
			//			if (p->mpt->flag == FLAG_MAPPOINT_NORMAL) {
			if (p->mpt->isCertainStatic())
				glColor3f(0.0f, 1.0f, 0.0f);
			else if (p->mpt->isFalse())
				glColor3f(1.0f, 0.0f, 0.0f);
			else if (p->mpt->isCertainDynamic())
				glColor3f(0.0f, 0.0f, 1.0f);
			else if (p->mpt->isUncertain())
				glColor3f(1.0f, 1.0f, 0.0f);
			//			} else if (p->mpt->flag == FLAG_MAPPOINT_TEST4) {
			//				glColor3f(0.0f, 1.0f, 1.0f);
			//				glLineWidth(2);
			//			}

			//			} else if (p->mpt->flag == FLAG_MAPPOINT_TEST1)
			//				glColor3f(1.0f, 0.0f, 0.0f);
			//			else if (p->mpt->flag == FLAG_MAPPOINT_TEST2)
			//				glColor3f(0.0f, 1.0f, 0.0f);
			//			else if (p->mpt->flag == FLAG_MAPPOINT_TEST3)
			//				glColor3f(0.0f, 0.0f, 1.0f);
			//			else if (p->mpt->flag == FLAG_MAPPOINT_TEST4)
			//				glColor3f(1.0f, 1.0f, 0.0f);
			//test
			if (p->mpt->flag >= FLAG_MAPPOINT_TEST1) {
				glColor3f(1.0f, 0.0f, 0.0f);
				GLUquadric* quad = gluNewQuadric();
				glTranslatef(x, y, 0);
				gluDisk(quad, 0, p->mpt->numVisCam * 1.5 * radius, 30, 10);
				glTranslatef(-x, -y, 0);
				gluDeleteQuadric(quad);

			}

			if (p->mpt->isCertainDynamic())
				glLineWidth(3.0f);
			else
				glLineWidth(2.0f);

			drawCircle(x, y, p->mpt->numVisCam * 1.5 * radius, 10);

			//drawCircle(x, y, radius, 10);

			if (b_drawReprojectionError) {
				//draw reprojection error
				double* M = p->mpt->M;
				double m[2];
				project(m_pSLAM->slam[m_camId].K, m_pCamPos->R, m_pCamPos->t, M,
						m);
				float x01 = static_cast<int>(m[0]);
				float y01 = static_cast<int>(m[1]);
				float x1, y1;
				convert2ImageCoords(x01, y01, x1, y1);
				glLineWidth(2);
				glColor3f(COLOR_CORNER_ERR[0], COLOR_CORNER_ERR[1],
						COLOR_CORNER_ERR[2]);
				drawLine(x, y, x1, y1);
				glLineWidth(1);
			}
		}
	}
}
void GLImagePane::drawConvexHulls() {
	if (!m_pSLAM)
		return;
	glLineWidth(3.0f);
	const int numCams = m_pSLAM->numCams;
	std::vector<double>* pConvexHull = 0;
	for (int j = 0; j < numCams; ++j) {
		//draw convex hulls related to other views
		if (j == m_camId || m_overlapCost[m_camId * numCams + j] < 0)
			continue;
		pConvexHull = &m_convexHulls[m_camId][j];
		glColor3f(GLScenePane::CAMERA_COLORS[j * 3],
				GLScenePane::CAMERA_COLORS[j * 3 + 1],
				GLScenePane::CAMERA_COLORS[j * 3 + 2]);
		//draw the convex hull
		size_t npts = pConvexHull->size() / 2;
		glBegin(GL_LINE_LOOP);
		for (size_t k = 0; k < npts; k++) {
			float x = pConvexHull->at(2 * k);
			float y = pConvexHull->at(2 * k + 1);
			float ix, iy;
			convert2ImageCoords(x, y, ix, iy);
			glVertex2f(ix, iy);
		}
		if (npts > 0) {
			float x = pConvexHull->at(0);
			float y = pConvexHull->at(1);
			float ix, iy;
			convert2ImageCoords(x, y, ix, iy);
			glVertex2f(ix, iy);
		}
		glEnd();
	}
}
#include "geometry/SL_5point.h"
#include "geometry/SL_FundamentalMatrix.h"
#include "tools/SL_DrawCorners.h"
#include "tools/SL_Print.h"
void GLImagePane::drawClickedPoint(ImgRGB& img) {
	assert(s_clicked_camid >= 0);
	if (s_clicked_mappt) {
		if (!s_clicked_mappt->pFeatures[s_clicked_camid])
			return;

		double m0[2];
		project(m_pSLAM->slam[m_camId].K.data,
				m_pSLAM->slam[m_camId].m_camPos.current()->R,
				m_pSLAM->slam[m_camId].m_camPos.current()->t,
				s_clicked_mappt->M, m0);

		//drawCircleToData(W, H, imgData, m0[0], m0[1], 15, 255, 0, 255);
		double var[4];
		getProjectionCovMat(m_pSLAM->slam[m_camId].K.data,
				m_pSLAM->slam[m_camId].m_camPos.current()->R,
				m_pSLAM->slam[m_camId].m_camPos.current()->t,
				s_clicked_mappt->M, s_clicked_mappt->cov, var, 4);

		drawEllipseToData(img, m0[0], m0[1], var, 255, 0, 255);
	} else {
		if (s_clicked_featPt)
			drawPoint(img, s_clicked_featPt->x, s_clicked_featPt->y, 255, 0, 0);
	}
}
void GLImagePane::drawEpipolarLine(ImgRGB& img) {
	assert(s_clicked_camid >= 0);
	const double* R0 = m_pSLAM->slam[s_clicked_camid].m_camPos.current()->R;
	const double* t0 = m_pSLAM->slam[s_clicked_camid].m_camPos.current()->t;
	const double* iK0 = m_pSLAM->slam[s_clicked_camid].iK.data;

	const double* R1 = m_pSLAM->slam[m_camId].m_camPos.current()->R;
	const double* t1 = m_pSLAM->slam[m_camId].m_camPos.current()->t;
	const double* iK1 = m_pSLAM->slam[m_camId].iK.data;

	double E[9], F[9];
	formEMat(R0, t0, R1, t1, E);
	getFMat(iK1, iK0, E, F);

	double l[3];
	double fx, fy;
	if (s_clicked_mappt) {
		if (!s_clicked_mappt->pFeatures[s_clicked_camid])
			return;
		fx = s_clicked_mappt->pFeatures[s_clicked_camid]->m[0];
		fy = s_clicked_mappt->pFeatures[s_clicked_camid]->m[1];
	} else {
		fx = s_clicked_featPt->x;
		fy = s_clicked_featPt->y;
	}
	computeEpipolarLine(F, fx, fy, l);
	drawLine(img, l, 255, 255, 255);
}
void GLImagePane::initImageData(int w, int h) {
	if (imgData)
		delete[] imgData;
	imgWidth = w;
	imgHeight = h;
	imgData = new unsigned char[w * h];
}
void GLImagePane::copyDisplayData() {
	pthread_mutex_lock(&MyApp::s_mutexBA);
	//copy the image
	memcpy(imgData, m_pSLAM->slam[m_camId].m_img.data,
			imgWidth * imgHeight * sizeof(unsigned char));
	b_haveImageData = true;
	int frame = m_pSLAM->curFrame;
	//copy pointers to feature points
	m_pFeatPts.clear();
	FeaturePoint* phead = m_pSLAM->slam[m_camId].m_featPts.getFrameHead(frame);
	if (phead) {
		FeaturePoint* pend = m_pSLAM->slam[m_camId].m_featPts.getFrameTail(
				frame)->next;
		for (FeaturePoint* p = phead; p != pend; p = p->next)
			m_pFeatPts.push_back(p);
	}

	m_pCamPos = m_pSLAM->slam[m_camId].m_camPos.current();
	for (int i = 0; i < m_pSLAM->numCams; i++) {
		for (int j = 0; j < m_pSLAM->numCams; j++) {
			if (i == j)
				continue;
			m_convexHulls[i][j].clear();
			m_convexHulls[i][j] = m_pSLAM->sharedConvexHull[i][j];
		}
	}

	memcpy(m_overlapCost, m_pSLAM->viewOverlapCost,
			sizeof(double) * SLAM_MAX_NUM * SLAM_MAX_NUM);

	pthread_mutex_unlock(&MyApp::s_mutexBA);
}
void GLImagePane::drawInfo(int topleft_x, int topleft_y, int bottomrigth_x,
		int bottomrigth_y) {
	glViewport(topleft_x, topleft_y, bottomrigth_x - topleft_x,
			bottomrigth_y - topleft_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(topleft_x, bottomrigth_x, bottomrigth_y, topleft_y);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glutPrint3D(0, 0, 0, "0,0,0", 255, 0, 0, 0.5);

//	if (m_pSLAM && m_pSLAM->m_intraCamPoseUpdateFail[m_camId] > 0)
//		glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
//	else
//		glColor4f(1.0f, 1.0f, 1.0f, 0.3f);
//
//	glTranslatef(topleft_x, topleft_y, 0);
//	glBegin(GL_QUADS);
//	glVertex2f(0, 0);
//	glVertex2f(getWidth(), 0);
//	glVertex2f(getWidth(), 35);
//	glVertex2f(0, 35);
//	glEnd();

	double camColor[3] = { GLScenePane::CAMERA_COLORS[m_camId * 3],
			GLScenePane::CAMERA_COLORS[m_camId * 3 + 1],
			GLScenePane::CAMERA_COLORS[m_camId * 3 + 2] };

	glColor3f(camColor[0], camColor[1], camColor[2]);
	glBegin(GL_QUADS);
	glVertex2f(5, 5);
	glVertex2f(30, 5);
	glVertex2f(30, 30);
	glVertex2f(5, 30);
	glEnd();
	if (m_pSLAM) {
//		char buf[256];
//		//		if (!m_pSLAM->m_intraCamPoseUpdateFail[m_camId]) {
//		sprintf(buf, "frame:%d,static:%d, dynamic:%d, group id:#%d\n",
//				m_pSLAM->curFrame, m_pSLAM->m_nStaticFeat[m_camId],
//				m_pSLAM->m_nDynamicFeat[m_camId], m_pSLAM->m_groupId[m_camId]);
//		//		} else if (m_pSLAM->m_intraCamPoseUpdateFail[m_camId] == 1)
//		//			sprintf(buf, "too few static points\n");
//		//		else if (m_pSLAM->m_intraCamPoseUpdateFail[m_camId] == 2)
//		//			sprintf(buf, "static points are not well distributed\n");
//		//glTranslatef(-topleft_x, -topleft_y, 0);
//		//glutPrint2D(40, 20, buf, 1.0f, 1.0f, 1.0f, 1.0f, false);
//		glutPrint2D(40, 20, buf, camColor[0], camColor[1], camColor[2], 1.0f,
//				true);

		int gid = m_pSLAM->m_groupId[m_camId];
		double groupColor[3] = { GLScenePane::CAMERA_COLORS[gid * 3],
				GLScenePane::CAMERA_COLORS[gid * 3 + 1],
				GLScenePane::CAMERA_COLORS[gid * 3 + 2] };

		glColor3f(groupColor[0], groupColor[1], groupColor[2]);
		glBegin(GL_QUADS);
		glVertex2f(5, getHeight() - 30);
		glVertex2f(90, getHeight() - 30);
		glVertex2f(90, getHeight() - 5);
		glVertex2f(5, getHeight() - 5);
		glEnd();

		char buf[256];
		sprintf(buf, "Group %d", m_pSLAM->m_groupId[m_camId]);
		glutPrint2D(7, getHeight() - 11, buf, 1.0f, 1.0f, 1.0f, 1.0f, true);

		sprintf(buf, "%02d", m_camId);
		glutPrint2D(5, 25, buf, 1.0f, 1.0f, 1.0f, 1.0f, true);
	}
}
void GLImagePane::saveScreen(const char* filePath) {
	wxGLCanvas::SetCurrent(*m_context);
	wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	int w = GetParent()->GetSize().x;
	int h = GetSize().y;
	ImgRGB imgData(w, h);
	glReadPixels(-1, 0, w - 1, h, GL_BGR, GL_UNSIGNED_BYTE, imgData.data);
	cv::Mat img(h, w, CV_8UC3, imgData);
	cv::line(img, cv::Point2i(0, 0), cv::Point2i(0, h - 1), cv::Scalar(0, 0, 0),
			2, CV_AA, 0);
	cv::line(img, cv::Point2i(0, h - 1), cv::Point2i(w - 1, h - 1),
			cv::Scalar(0, 0, 0), 2, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, h - 1), cv::Point2i(w - 1, 0),
			cv::Scalar(0, 0, 0), 2, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, 0), cv::Point2i(0, 0), cv::Scalar(0, 0, 0),
			2, CV_AA, 0);
	CvMat cvImg = img;
	cvFlip(&cvImg, 0);
	cv::imwrite(filePath, img);
}
