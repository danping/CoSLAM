/*
 * GLModelPaneHelper.cpp
 *
 *  Created on: 2011-6-12
 *      Author: tsou
 */

#include "GLScenePaneHelper.h"
#include <GL/glu.h>
#include <GL/gl.h>
#include "GL/glut.h"

#include "math/SL_LinAlg.h"
#include "geometry/SL_Triangulate.h"
#include "slam/SL_SLAMHelper.h"

void drawAllCam(const CamPoseList& camlst, double cam_size) {
	CamPoseItem* cam = camlst.first();
	while (cam) {
		drawCameraPose(cam, cam_size, 0);
		cam = cam->next;
	}
}
void drawCamera(int IW, int IH, const double* K, const CamPoseItem* cam,
		double camSize, const float* color = 0, const double maxDist = 100) {
	double p0[3], xp[3], yp[3], zp[3]; //, tmp[3];
	getCamCoords(cam, p0, xp, yp, zp);
	double scale = camSize;
	//draw id
	//	char id_str[256];
	//	sprintf(id_str, "%d", cam->f);
	//	glutPrint(cam->t[0], cam->t[1], cam->t[2], GLUT_BITMAP_9_BY_15, id_str, 1, 1, 1, 0);

	//draw axes
	//	glBegin(GL_LINES);
	//	glColor3f(1, 0, 0);
	//	glquiver3(p0, xp, scale);
	//	glColor3f(0, 1, 0);
	//	glquiver3(p0, yp, scale);
	//	glColor3f(0, 0, 1);
	//	glquiver3(p0, zp, scale);
	//	glBegin(GL_LINES);
	//	glColor3f(color[0],color[1],color[2]);
	//	glquiver3(p0, xp, scale);
	//	glColor3f(color[0],color[1],color[2]);
	//	glquiver3(p0, yp, scale);
	//	glColor3f(color[0],color[1],color[2]);
	//	glquiver3(p0, zp, scale);
	//	glEnd();
	//	return;

	double quad[4][3];
	add(p0, zp, quad[0], 1.0, 1 * scale);
	add(quad[0], xp, quad[1], 1.0, 0.8 * scale);
	add(quad[1], yp, quad[0], 1.0, 0.8 * scale);

	if (color) {
		glColor3f(color[0], color[1], color[2]);
	}
	glLineWidth(2.0f);
	glBegin(GL_LINE_STRIP);
	glpt3(quad[0]);
	add(quad[0], xp, quad[1], 1.0, -1.6 * scale);
	glpt3(quad[1]);
	add(quad[1], yp, quad[2], 1.0, -1.6 * scale);
	glpt3(quad[2]);
	add(quad[2], xp, quad[3], 1.0, 1.6 * scale);
	glpt3(quad[3]);
	glpt3(quad[0]);
	glEnd();

	glBegin(GL_LINES);
	glline(p0, quad[0]);
	glline(p0, quad[1]);
	glline(p0, quad[2]);
	glline(p0, quad[3]);
	glEnd();

	//draw view angle
	if (maxDist > 0) {
		double Cx = K[2];
		double fx = K[0];
		double fy = K[4];
		double A1[3] = { -maxDist * Cx / fx, 0, maxDist };
		double A2[3] = { (IW - Cx) * maxDist / fx, 0, maxDist };
		double wA1[3];
		double wA2[3];

		CamCoord2WoldCoord(cam->R, cam->t, A1, wA1);
		CamCoord2WoldCoord(cam->R, cam->t, A2, wA2);
		//transform into world coordinates

		glColor4f(color[0], color[1], color[2], 0.2);
		glBegin(GL_TRIANGLES);
		glpt3(wA1);
		glpt3(p0);
		glpt3(wA2);
		glEnd();
	}
}

void drawCameraPose(const CamPoseItem* cam, double camSize, const float* color =
		0, float linewidth) {
	double p0[3], xp[3], yp[3], zp[3]; //, tmp[3];
	getCamCoords(cam, p0, xp, yp, zp);
	double scale = camSize;

	//	//draw id
	//	char id_str[256];
	//	sprintf(id_str, "%d", cam->f);
	//	glutPrint(cam->t[0], cam->t[1], cam->t[2], GLUT_BITMAP_9_BY_15, id_str, 1, 1, 1, 0);

	//draw axes
	//	glBegin(GL_LINES);
	//	glColor3f(1, 0, 0);
	//	glquiver3(p0, xp, scale);
	//	glColor3f(0, 1, 0);
	//	glquiver3(p0, yp, scale);
	//	glColor3f(0, 0, 1);
	//	glquiver3(p0, zp, scale);
	//	glEnd();

	//	glBegin(GL_LINE_STRIP);
	//	glpt3( quad[0]);
	//	add(quad[0], xp, quad[1], 1.0, -1.6 * scale);
	//	glpt3(quad[1]);
	//	add(quad[1], yp, quad[2], 1.0, -1.6 * scale);
	//	glpt3(quad[2]);
	//	add(quad[2], xp, quad[3], 1.0, 1.6 * scale);
	//	glpt3(quad[3]);
	//	glpt3(quad[0]);
	//	glEnd();

	glLineWidth(linewidth);
	glBegin(GL_LINES);
	glColor3f(color[0], color[1], color[2]);
	glquiver3(p0, zp, scale);
	glEnd();

	//	double quad[4][3];
	//	add(p0, zp, quad[0], 1.0, 1 * scale);
	//	add(quad[0], xp, quad[1], 1.0, 0.8 * scale);
	//	add(quad[1], yp, quad[0], 1.0, 0.8 * scale);
	//
	//	if (color) {
	//		glColor3f(color[0], color[1], color[2]);
	//	}
	//	glBegin(GL_LINE_STRIP);
	//	glpt3(quad[0]);
	//	add(quad[0], xp, quad[1], 1.0, -1.6 * scale);
	//	glpt3(quad[1]);
	//	add(quad[1], yp, quad[2], 1.0, -1.6 * scale);
	//	glpt3(quad[2]);
	//	add(quad[2], xp, quad[3], 1.0, 1.6 * scale);
	//	glpt3(quad[3]);
	//	glpt3(quad[0]);
	//	glEnd();
	//
	//	glBegin(GL_LINES);
	//	glline(p0, quad[0]);
	//	glline(p0, quad[1]);
	//	glline(p0, quad[2]);
	//	glline(p0, quad[3]);
	//	glEnd();

}

void drawCameraPose(const CamPoseItem* cam, double camSize, const float* color =
		0) {
	double p0[3], xp[3], yp[3], zp[3]; //, tmp[3];
	getCamCoords(cam, p0, xp, yp, zp);
	double scale = camSize;

	//	//draw id
	//	char id_str[256];
	//	sprintf(id_str, "%d", cam->f);
	//	glutPrint(cam->t[0], cam->t[1], cam->t[2], GLUT_BITMAP_9_BY_15, id_str, 1, 1, 1, 0);

	//draw axes
	//	glBegin(GL_LINES);
	//	glColor3f(1, 0, 0);
	//	glquiver3(p0, xp, scale);
	//	glColor3f(0, 1, 0);
	//	glquiver3(p0, yp, scale);
	//	glColor3f(0, 0, 1);
	//	glquiver3(p0, zp, scale);
	//	glEnd();

	double quad[4][3];
	add(p0, zp, quad[0], 1.0, 1 * scale);
	add(quad[0], xp, quad[1], 1.0, 0.8 * scale);
	add(quad[1], yp, quad[0], 1.0, 0.8 * scale);

	if (color) {
		glColor3f(color[0], color[1], color[2]);
	}
	glBegin(GL_LINE_STRIP);
	glpt3(quad[0]);
	add(quad[0], xp, quad[1], 1.0, -1.6 * scale);
	glpt3(quad[1]);
	add(quad[1], yp, quad[2], 1.0, -1.6 * scale);
	glpt3(quad[2]);
	add(quad[2], xp, quad[3], 1.0, 1.6 * scale);
	glpt3(quad[3]);
	glpt3(quad[0]);
	glEnd();

	glBegin(GL_LINES);
	glline(p0, quad[0]);
	glline(p0, quad[1]);
	glline(p0, quad[2]);
	glline(p0, quad[3]);
	glEnd();

}
void drawCamCenter(const CamPoseItem* cam, const float* color, int ptSz) {
	double org[3], ox[3], oy[3], oz[3];
	getCamCoords(cam, org, ox, oy, oz);
	glColor3f(color[0], color[1], color[2]);
	glLineWidth(ptSz);
	glBegin(GL_POINTS);
	glVertex3f(org[0], org[1], org[2]);
	glEnd();
}

void drawCurMapPoint(const MapPoint* p, double range, double pointSize,
		bool drawCov) {
	//GLfloat LightAmbient[] = { 0.5f, 1.0f, 0.5f, 1.0f };
	GLfloat LightAmbient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightAmbientInAct[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightDiffuse[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightPosition[] = { -range * 1.0f, -range * 1.0f, 0.0f, 1.0f };

	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbientInAct);
	glLightfv(GL_POSITION, GL_DIFFUSE, LightAmbientInAct);

	if (p->isCertainStatic())
		glColor3f(0.2f, 0.8f, 0.2f);
	else if (p->isCertainDynamic())
		glColor3f(0.0f, 0.0f, 1.0f);
	else if (p->isUncertain())
		glColor3f(0.8f, 0.8f, 0.0f);
	else
		return;

	//for debug
	if (p->flag == FLAG_MAPPOINT_TEST4)
		glColor3f(1.0f, 0.0f, 0.0f);

//	glEnable(GL_LIGHTING);
//	glEnable(GL_LIGHT1);
//
//	glPushMatrix();
//	glTranslatef(p->x, p->y, p->z);
//	glutSolidSphere(0.1 * range * p->numVisCam, 12, 12);
//	glPopMatrix();
//
//	glDisable(GL_LIGHT1);
//	glDisable(GL_LIGHTING);

	if (p->isCertainStatic())
		glPointSize(4.0 * pointSize);
	else
		glPointSize(8.0 * pointSize);

	if (p->flag == FLAG_MAPPOINT_TEST4)
		glPointSize(20.0 * pointSize);

	glBegin(GL_POINTS);
	glVertex3d(p->x, p->y, p->z);
	glEnd();

	if (drawCov && p->cov[0] > 0) {
		//skip the dynamic points
		if (p->isCertainDynamic())
			return;

		if (p->isCertainStatic())
			glColor4f(0.2f, 0.8f, 0.2f, 0.2f);
		else if (p->isCertainDynamic())
			glColor4f(0.0f, 0.0f, 1.0f, 0.2f);
		else if (p->isUncertain())
			glColor4f(0.8f, 0.8f, 0.0f, 0.2f);
		else
			return;

//		//for debug
//		if (p->flag == FLAG_MAPPOINT_TEST4
//		)
//			glColor3f(1.0f, 0.0f, 0.0f);

		double evec[9];
		double eval[3];
		dgeevFor(3, p->cov, evec, eval);
		glPushMatrix();
		glTranslatef(p->x, p->y, p->z);
		double rotMat[16];
		memset(rotMat, 0, sizeof(double) * 16);
		memcpy(rotMat, evec, sizeof(double) * 3);
		memcpy(rotMat + 4, evec + 3, sizeof(double) * 3);
		memcpy(rotMat + 8, evec + 3, sizeof(double) * 3);
		rotMat[15] = 1;
		glMultMatrixd(rotMat);
		//assert(eval[0] > 0 && eval[1] > 0 && eval[2] > 0);
		glScaled(sqrt(eval[0]), sqrt(eval[1]), sqrt(eval[2]));
		glutSolidSphere(1.0, 12, 12);
		glPopMatrix();
	}
}

void getCenterAndRange(const CamPoseList& camlst, double& xc, double& yc,
		double& zc, double& range) {

	CamPoseItem* cam = camlst.first();

	//get the center point of the camera positions
	double s[3] = { 0, 0, 0 };
	while (cam) {
		add(cam->t, s, s);
		cam = cam->next;
	}
	xc = s[0] / camlst.size();
	yc = s[1] / camlst.size();
	zc = s[2] / camlst.size();

	//get the maximum distance
	double ds_max = 0;
	cam = camlst.first();
	while (cam) {
		double dx = xc - cam->t[0];
		double dy = yc - cam->t[1];
		double dz = zc - cam->t[2];
		double ds = sqrt(dx * dx + dy * dy + dz * dz);
		if (ds > ds_max) {
			ds_max = ds;
		}
		cam = cam->next;
	}
	range = ds_max;
}

void getCenterAndRange(const MapPointList& pts, double& xc, double& yc,
		double& zc, double &range) {
	//get the center point of the points
	double c[3] = { 0, 0, 0 };
	for (const MapPoint* pt = pts.getHead(); pt; pt = pt->next) {
		c[0] += pt->x;
		c[1] += pt->y;
		c[2] += pt->z;
	}

	xc = c[0] / pts.getNum();
	yc = c[1] / pts.getNum();
	zc = c[2] / pts.getNum();

	double ds_max = 0;
	for (const MapPoint* pt = pts.getHead(); pt; pt = pt->next) {
		double dx = xc - pt->x;
		double dy = yc - pt->y;
		double dz = zc - pt->z;
		double ds = sqrt(dx * dx + dy * dy + dz * dz);
		if (ds > ds_max) {
			ds_max = ds;
		}
		pt = pt->next;
	}
	range = ds_max;
}

void getCenterAndRange(const CamPoseList& camlst, const MapPointList& pts,
		double &xc, double& yc, double& zc, double &range) {
	CamPoseItem* cam = camlst.first();

	//get the center point of all cameras and points
	double c[3] = { 0, 0, 0 };
	while (cam) {
		add(cam->t, c, c);
		cam = cam->next;
	}

	const MapPoint* pt = pts.getHead();

	while (pt) {
		c[0] += pt->x;
		c[1] += pt->y;
		c[2] += pt->z;
		pt = pt->next;
	}

	int num = camlst.size() + pts.getNum();
	xc = c[0] / num;
	yc = c[1] / num;
	zc = c[2] / num;

	//get the range of all cameras and points
	double ds_max = 0;
	cam = camlst.first();
	while (cam) {
		double dx = xc - cam->t[0];
		double dy = yc - cam->t[1];
		double dz = zc - cam->t[2];
		double ds = sqrt(dx * dx + dy * dy + dz * dz);
		if (ds > ds_max) {
			ds_max = ds;
		}
		cam = cam->next;
	}
	pt = pts.getHead();
	while (pt) {
		double dx = xc - pt->x;
		double dy = yc - pt->y;
		double dz = zc - pt->z;
		double ds = sqrt(dx * dx + dy * dy + dz * dz);
		if (ds > ds_max) {
			ds_max = ds;
		}
		pt = pt->next;
	}
	range = ds_max;
}
