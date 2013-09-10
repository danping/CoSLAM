/*
 * GLScenePane.cpp
 *
 *  Created on: 2011-7-15
 *      Author: zou
 */
#include "GLScenePane.h"
#include "GLHelper.h"
#include "GLScenePaneHelper.h"
#include "MyApp.h"
#include "slam/SL_SLAMHelper.h"
#include <fstream>
BEGIN_EVENT_TABLE(GLScenePane, GLTrackballPane) EVT_SIZE(GLScenePane::resized)
EVT_RIGHT_DOWN(GLScenePane::rightClick)
EVT_KEY_DOWN(GLScenePane::keyPressed)
EVT_CHAR(GLScenePane::charPressed)
END_EVENT_TABLE()
///////////////////////////////////////////////////
void getDynTracks(const vector<vector<Point3dId> >& dynMapPts,
		vector<vector<Point3dId> >& dynTracks, int trjLen) {
	map<size_t, vector<Point3dId> > tracks;

	dynTracks.clear();
	if (dynMapPts.empty())
		return;

	int l = 0;
	for (vector<vector<Point3dId> >::const_reverse_iterator iter =
			dynMapPts.rbegin(); iter != dynMapPts.rend() && l < trjLen;
			iter++, l++) {
		const vector<Point3dId>& pts = *iter;
		if (l == 0) {
			for (size_t n = 0; n < pts.size(); n++) {
				size_t id = pts[n].id;
				tracks[id].push_back(pts[n]);
			}
		} else {
			for (size_t n = 0; n < pts.size(); n++) {
				size_t id = pts[n].id;
				if (tracks.count(id) == 0)
					continue;
				else
					tracks[id].push_back(pts[n]);
			}
		}
	}

	for (map<size_t, vector<Point3dId> >::iterator iter = tracks.begin();
			iter != tracks.end(); iter++) {
		dynTracks.push_back(iter->second);
	}
}
void GLScenePane::copyDispData() {
	pthread_mutex_lock(&MyApp::s_mutexBA);
	curMapPoints.clear();
	for (const MapPoint* p = m_pSLAM->curMapPts.getHead(); p; p = p->next) {
		curMapPoints.push_back(p);
	}
	actMapPoints.clear();
	for (const MapPoint* p = m_pSLAM->actMapPts.getHead(); p; p = p->next) {
		actMapPoints.push_back(p);
	}
	iactMapPoints.clear();
	for (const MapPoint* p = m_pSLAM->iactMapPts.getHead(); p; p = p->next) {
		iactMapPoints.push_back(p);
	}
	const int groupNum = m_pSLAM->m_groupNum;
	for (int i = 0; i < groupNum; i++) {
		m_groups[i].copy(m_pSLAM->m_groups[i]);
	}
	dynTracks.clear();
	getDynTracks(m_pSLAM->m_dynPts, dynTracks, m_nTrjLen);
	m_groupNum = groupNum;
	pthread_mutex_unlock(&MyApp::s_mutexBA);

	if (m_autoScale)
		getSceneScale();
}
void GLScenePane::getSceneScale() {
	m_center[0] = 0;
	m_center[1] = 0;
	m_center[2] = 0;
	int num = 0;

	double org[3], axisX[3], axisY[3], axisZ[3];
	for (int i = 0; i < m_pSLAM->numCams; i++) {
		const CamPoseItem* cam = m_pSLAM->slam[i].m_camPos.first();
		while (cam) {
			getCamCoords(cam, org, axisX, axisY, axisZ);
			m_center[0] += org[0];
			m_center[1] += org[1];
			m_center[2] += org[2];
			num++;
			cam = cam->next;
		}
	}
	m_center[0] /= num;
	m_center[1] /= num;
	m_center[2] /= num;

	m_scale = 0;
	for (int i = 0; i < m_pSLAM->numCams; i++) {
		const CamPoseItem* cam = m_pSLAM->slam[i].m_camPos.first();
		while (cam) {
			getCamCoords(cam, org, axisX, axisY, axisZ);
			double dx = m_center[0] - org[0];
			double dy = m_center[1] - org[1];
			double dz = m_center[2] - org[2];
			m_scale += sqrt(dx * dx + dy * dy + dz * dz);
			cam = cam->next;
		}
	}
	m_scale /= num;
	m_scale *= 0.5;
}
void GLScenePane::drawPoints() {
	//draw current map points
	for (size_t i = 0; i < curMapPoints.size(); i++) {
		const MapPoint* p = curMapPoints[i];
		drawCurMapPoint(p, m_scale, m_pointSize, true); //!MyApp::bStop);
	}
	glPointSize(3.0 * m_pointSize);
	//draw active map points
	glBegin(GL_POINTS);
	for (size_t i = 0; i < actMapPoints.size(); i++) {
		const MapPoint* p = actMapPoints[i];
		if (p->isCertainStatic()) {
			double r = p->color[0] / 255.0;
			double g = p->color[1] / 255.0;
			double b = p->color[2] / 255.0;
			glColor3d(r, g, b);
			glVertex3d(p->x, p->y, p->z);
		}
	}
	glEnd();
	glPointSize(2.0 * m_pointSize);
	//	//draw inactive map points
	//	for (size_t i = 0; i < iactMapPoints.size(); i++) {
	//		const MapPoint* p = iactMapPoints[i];
	//		if (p->type == TYPE_MAPPOINT_STATIC) {
	//			//glColor3f(PT_COLOR[0], PT_COLOR[1], PT_COLOR[2]);
	//			glColor3f(0.35f, 0.35f, 0.35f);
	//			glTranslatef(p->x, p->y, p->z);
	//			//glutSolidSphere(p->flag < FLAG_MAPPOINT_TEST1 ? m_range * 0.018 : m_range * 0.018 * 5, 12, 12);
	//			glutSolidSphere(0.004 * m_range * p->numVisCam, 12, 12);
	//			//			glutSolidSphere(p->flag < FLAG_MAPPOINT_TEST1 ? 0.004 * m_range * p->numVisCam : m_range * 0.004 * 5, 12,
	//			//					12);
	//			glTranslatef(-p->x, -p->y, -p->z);
	//		}
	//	}
	//glDisable(GL_LIGHT1);
	//glDisable(GL_LIGHTING);

	//draw inactive map points
	glBegin(GL_POINTS);
	for (size_t i = 0; i < iactMapPoints.size(); i++) {
		const MapPoint* p = iactMapPoints[i];
		if (p->isGlobalStatic()) {
			double r = p->color[0] / 255.0;
			double g = p->color[1] / 255.0;
			double b = p->color[2] / 255.0;
			glColor3d(r, g, b);

			glVertex3d(p->x, p->y, p->z);
		}
	}
	glEnd();

	//draw dynamic trajectories
	if (m_drawDynTrj) {
//		glPointSize(0.5);
//		glBegin(GL_POINTS);
//		int tf = dynMapPoints.size();
//		int f0 = tf - m_nTrjLen < 0 ? 0 : tf - m_nTrjLen;
//		int numFrms = tf - f0;
//		double dalpha = 0.3 / (numFrms + 1.0);
//		for (int f = f0; f < tf; f++) {
//			glColor4d(0, 0, 1, 0.0 + (f - f0) * dalpha);
//			for (int i = 0; i < dynMapPoints[f].m; i++) {
//				glVertex3d(dynMapPoints[f].data[3 * i], dynMapPoints[f].data[3 * i + 1], dynMapPoints[f].data[3 * i + 2]);
//			}
//		}
//		glEnd();

		for (size_t n = 0; n < dynTracks.size(); n++) {
			glLineWidth(0.5);
			glBegin(GL_LINE_STRIP);

			double alpha = 1.0 / dynTracks[n].size();
			for (size_t i = 0; i < dynTracks[n].size(); i++) {
				glColor4d(0, 0, 1, 1.0 - i * alpha);
				glVertex3d(dynTracks[n][i].x, dynTracks[n][i].y,
						dynTracks[n][i].z);
			}
			glEnd();
		}
	}
}
//float GLScenePane::CAMERA_COLORS[15] = { 1.0f, 0.0f, 0.0f, 0.2f, 0.8f, 0.0f,
//		0.0f, 0.0f, 1.0f, 0.7f, 0.7f, 0.0f, 0.0f, 1.0f, 1.0f };

float GLScenePane::CAMERA_COLORS[3 * SLAM_MAX_NUM] = { 0.0f, 0.0f, 1.0f, 0.0f,
		0.5f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.75f, 0.75f, 0.75f, 0.0f, 0.75f,
		0.75f, 0.75f, 0.0f, 0.25f, 0.25f, 0.25f, 0.0f, 0.0f, 1.0f, 0.0f, 0.5f,
		0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.75f, 0.75f, 0.75f, 0.0f, 0.75f };

void GLScenePane::drawCameras() {
	//draw camera trajectory
	for (int i = 0; i < m_pSLAM->numCams; i++) {
		const CamPoseItem* cam = m_pSLAM->slam[i].m_camPos.first();
		while (cam->f < m_pSLAM->m_lastFrmBundle)
			cam = cam->next;
		//glColor3f(CAMERA_COLORS[3 * i], CAMERA_COLORS[3 * i + 1], CAMERA_COLORS[3 * i + 2]);
		glLineWidth(2.0f);
		glColor3f(0.5f, 0.5f, 0.5f);
		glBegin(GL_LINE_STRIP);
		while (cam) {
			//			drawCamCenter(cam, PT_COLOR + 3 * i);
			//if (cam->f >= m_pSLAM->curFrame - 200) 
			{
				double org[3];
				getCamCenter(cam, org);
				glVertex3d(org[0], org[1], org[2]);
			}
			cam = cam->next;
		}
		glEnd();

		glPointSize(2.0);
		glBegin(GL_LINE_STRIP);
		cam = m_pSLAM->slam[i].m_camPos.first();
		while (cam) {
			//			drawCamCenter(cam, PT_COLOR + 3 * i);
			//if (cam->f >= m_pSLAM->curFrame - 200)
			{
				double org[3];
				getCamCenter(cam, org);
				glVertex3d(org[0], org[1], org[2]);
			}
			cam = cam->next;
		}
		glEnd();

		cam = m_pSLAM->slam[i].m_camPos.first();
		glColor3f(0.5f, 0.5f, 0.5f);
		glLineWidth(2.0f);
		glBegin(GL_LINE_STRIP);
		while (cam && cam->f <= m_pSLAM->m_lastFrmBundle) {
			//			drawCamCenter(cam, PT_COLOR + 3 * i);
			if (cam->f >= m_pSLAM->m_firstFrmBundle) {
				double org[3];
				getCamCenter(cam, org);
				glVertex3d(org[0], org[1], org[2]);
			}
			cam = cam->next;
		}
		glEnd();

		drawCamera(m_pSLAM->slam[i].videoReader->_w,
				m_pSLAM->slam[i].videoReader->_h, m_pSLAM->slam[i].K.data,
				m_pSLAM->slam[i].m_camPos.current(), m_camSize,
				CAMERA_COLORS + 3 * i, 100);
	}
	//draw key camera poses
	for (int i = 0; i < m_pSLAM->numCams; i++) {

		for (size_t n = 0; n < m_pSLAM->slam[i].m_selfKeyPose.size(); ++n) {
			const KeyPose* keyPose = m_pSLAM->slam[i].m_selfKeyPose[n];
			drawCameraPose(keyPose->cam, m_camSize, CAMERA_COLORS + 3 * i, 4);
		}

		const KeyPose* keyPose = m_pSLAM->slam[i].m_keyPose.first();
		int n = 0;
		while (keyPose) {
			drawCameraPose(keyPose->cam, m_camSize * 0.5,
					CAMERA_COLORS + 3 * i);
			n++;
			keyPose = keyPose->next;
		}
	}
}
void GLScenePane::drawInfo() {
	int bottomright_x = GetSize().x;
	int bottomright_y = GetSize().y;
	int topleft_x = 0;
	int topleft_y = bottomright_y - 120;

	glViewport(topleft_x, topleft_y, bottomright_x - topleft_x,
			bottomright_y - topleft_y);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(topleft_x, bottomright_x, bottomright_y, topleft_y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glutPrint3D(0, 0, 0, "0,0,0", 255, 0, 0, 0.5);
	if (MyApp::bStop)
		glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
	else
		glColor4f(0.5f, 0.5f, 0.5f, 0.5f);

	glTranslatef(topleft_x, topleft_y, 0);
	glBegin(GL_QUADS);
	glVertex2f(0, 0);
	glVertex2f(GetSize().x, 0);
	glVertex2f(GetSize().x, 60);
	glVertex2f(0, 60);
	glEnd();

	char buf[256];

	if (m_pSLAM) {
		sprintf(
				buf,
				"frame:%d,static(%d),dynamic(%d), stereo matching %d, bundle adjustment (%s) %d\n",
				m_pSLAM->curFrame, m_pSLAM->m_nStatic, m_pSLAM->m_nDynamic,
				m_pSLAM->m_lastFrmInterMapping,
				MyApp::bBusyBAing ? "busy" : "idle", m_pSLAM->m_lastFrmBundle);
		glutPrint2D(20, 15, buf, 1.0f, 0.0f, 0.0f, 0.5f, false);

		sprintf(
				buf,
				"fps:%3.2lf, inter-cam mapping:%3.3lf(ms), read frame :%3.0lf, feature tracking :%3.0lf, pose update:%3.0lf(ms)\n",
				1000.0 / m_pSLAM->m_tmPerStep * 0.7, m_pSLAM->m_tmNewMapPoints,
				m_pSLAM->m_tmReadFrame, m_pSLAM->m_tmFeatureTracking,
				m_pSLAM->m_tmPoseUpdate);
		glutPrint2D(20, 35, buf, 0.0f, 0.0f, 1.0f, 0.5f, false);

		sprintf(
				buf,
				"grouping :%3.0lf(ms) map classification:%3.0lf(ms), cur map registration:%lf(ms), act map registration:%lf(ms)",
				m_pSLAM->m_tmCameraGrouping, m_pSLAM->m_tmMapClassify,
				m_pSLAM->m_tmCurMapRegister, m_pSLAM->m_tmActMapRegister);

		glutPrint2D(20, 55, buf, 0.0f, 0.0f, 1.0f, 0.5f, false);

		sprintf(buf, "frame large merged:%d", m_pSLAM->m_lastFrmGroupMerge);
		
		glutPrint2D(20, 75, buf, 0.0f, 0.0f, 1.0f, 0.5f, false);
	}
	if (m_pSLAM) {
		char buf[256];
		sprintf(buf, "frame:%d", m_pSLAM->curFrame);
		glutPrint2D(20, 15, buf, 1.0f, 0.0f, 0.0f, 0.5f, false);
	}
}
void GLScenePane::drawCameraGroups() {
	int topleft_x = 0;
	int topleft_y = 0;
	int bottomright_x = GetSize().x;
	int bottomright_y = GetSize().y;

	glViewport(topleft_x, topleft_y, bottomright_x - topleft_x,
			bottomright_y - topleft_y);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(topleft_x, bottomright_x, bottomright_y, topleft_y);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	int base_top = -70;
	if (m_pSLAM) {
		char buf[256];
		glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
		sprintf(buf, "frame: %d", m_pSLAM->curFrame);
		glutPrint2D(15, 15, buf, 0.0f, 0.0f, 1.0f, 0.5f, false);
		for (int i = 0; i < m_groupNum; i++) {
			float groupColor[3] = { CAMERA_COLORS[3 * i], CAMERA_COLORS[3 * i
					+ 1], CAMERA_COLORS[3 * i + 2] };

			drawBlock(23, base_top + 90 + 30 * i, 50, 19, groupColor);
			sprintf(buf, "group %d:", i);
			glutPrint2D(25, base_top + 102 + 30 * i, buf, 1.0f, 1.0f, 1.0f,
					1.0f, false);

			int idNum = m_groups[i].num;
			for (int j = 0; j < idNum; j++) {
				int camId = m_groups[i].camIds[j];

				drawBlock(85 + 20 * j, base_top + 92 + 30 * i, 15, 15,
						CAMERA_COLORS + 3 * camId);

				sprintf(buf, "%02d", camId);
				glutPrint2D(86 + 20 * j, base_top + 103 + 30 * i, buf, 1.0f,
						1.0f, 1.0f, 1.0f, false);
			}
		}
	}
}
void GLScenePane::drawGLObjs() {
	glClearColor(1.0, 1.0, 1.0, 1.0);
	if (m_followCamId < 0) {
		glScaled(1. / m_scale, 1. / m_scale, 1. / m_scale);
		glTranslated(-m_center[0], -m_center[1], -m_center[2]);
	} else {
		if (m_pSLAM) {
			const CamPoseItem* cam =
					m_pSLAM->slam[m_followCamId].m_camPos.current();
			if (cam) {
				double mat[16];
				glGetDoublev(GL_MODELVIEW_MATRIX, mat);
				glMatrixMode(GL_MODELVIEW);
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						mat[j * 4 + i] = cam->R[i * 3 + j];
						mat[j * 4 + 3] = 0;
					}
				}
				mat[12] = cam->t[0];
				mat[13] = cam->t[1];
				mat[14] = cam->t[2];
				mat[15] = 1.0;
				glMultMatrixd(mat);
			}
		}
	}

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	if (m_pSLAM && m_pSLAM->curMapPts.getNum() > 0 && m_pSLAM->curFrame > 1) {
		MyApp::bBusyDrawingModel = true;
		drawPoints();
		drawCameras();
		//drawInfo();
		drawCameraGroups();
		MyApp::bBusyDrawingModel = false;
	}
}
#include "opencv2/opencv.hpp"
void GLScenePane::saveScreen(const char* filePath) {
	wxGLCanvas::SetCurrent(*m_context);
	wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	int w = GetParent()->GetScreenRect().width;
	int h = GetSize().y;
	unsigned char* imgData = new unsigned char[w * h * 3];
	glReadPixels(-1, 0, w - 1, h, GL_BGR, GL_UNSIGNED_BYTE, imgData);
	cv::Mat img(h, w, CV_8UC3, imgData);
	cv::line(img, cv::Point2i(0, 0), cv::Point2i(0, h - 1), cv::Scalar(0, 0, 0),
			1, CV_AA, 0);
	cv::line(img, cv::Point2i(0, h - 1), cv::Point2i(w - 1, h - 1),
			cv::Scalar(0, 0, 0), 1, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, h - 1), cv::Point2i(w - 1, 0),
			cv::Scalar(0, 0, 0), 1, CV_AA, 0);
	cv::line(img, cv::Point2i(w - 1, 0), cv::Point2i(0, 0), cv::Scalar(0, 0, 0),
			1, CV_AA, 0);
	CvMat cvImg = img;
	cvFlip(&cvImg, 0);
	cv::imwrite(filePath, img);
	delete[] imgData;
}

void GLScenePane::resized(wxSizeEvent& evt) {
	Refresh();
}
void GLScenePane::rightClick(wxMouseEvent& event) {
	MyApp::bStop = true;
//	//save the result
//	try {
//		m_pSLAM->saveResult(MyApp::timeStr);
//	} catch (std::exception& e) {
//		logInfo("%s\n", e.what());
//	}
	logInfo("save result OK!\n");
}

void GLScenePane::keyPressed(wxKeyEvent& event) {
//	if (event.GetKeyCode() == 'S') {
//		m_pSLAM->saveCurrentFrame(MyApp::timeStr);
//		MyApp::bStop = true;
//	} else 
	if (event.GetKeyCode() == 'A') {
		m_autoScale = !m_autoScale;
	} else if (event.GetKeyCode() == 'O') {
		m_camSize *= 1.5;
	} else if (event.GetKeyCode() == 'P') {
		m_camSize /= 1.5;
	} else if (event.GetKeyCode() == 'T') {
		MyApp::bSingleStep = !MyApp::bSingleStep;
	} else if (event.GetKeyCode() == 'H') {
		m_nTrjLen += 10;
	} else if (event.GetKeyCode() == 'J') {
		m_nTrjLen -= 10;
		m_nTrjLen = m_nTrjLen < 0 ? 0 : m_nTrjLen;
	} else {
		MyApp::bStop = !MyApp::bStop;
//		m_drawDynTrj = MyApp::bStop;
	}
	//test
	logInfo("'%d' key pressed!\n", event.GetKeyCode());
	logInfo("m_nTrjLen:%d\n", m_nTrjLen);
	Refresh();
}
void GLScenePane::charPressed(wxKeyEvent& event) {
}
void GLScenePane::save(const char* file_path) {
	ofstream file(file_path);
	if (!file)
		repErr("cannot open '%s' to write!", file_path);

	file << m_pointSize << endl;
	file << m_followCamId << endl;
	file << m_camView << endl;
	file << m_autoScale << endl;
	file << m_drawDynTrj << endl;
	file << m_nTrjLen << endl;
	file << m_camSize << endl;
	file << m_center[0] << " " << m_center[1] << " " << m_center[2] << endl;
	file << m_scale << endl;

	for (int i = 0; i < 16; ++i) {
		file << _matrix[i] << endl;
	}
	cout << "display parameters have been saved in '" << file_path << "'."
			<< endl;
}
bool GLScenePane::load(const char* file_path) {
	try {
		ifstream file(file_path);
		if (!file)
			repErr("cannot open '%s' to read!", file_path);

		file >> m_pointSize;
		file >> m_followCamId;
		file >> m_camView;
		file >> m_autoScale;
		file >> m_drawDynTrj;
		file >> m_nTrjLen;
		file >> m_camSize;
		file >> m_center[0] >> m_center[1] >> m_center[2];
		file >> m_scale;

		for (int i = 0; i < 16; ++i) {
			file >> _matrix[i];
		}
	} catch (SL_Exception& e) {
		cout << e.what();
		return false;
	}
	return true;
}
