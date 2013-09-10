/*
 * GLHelper.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef GLHELPER_H_
#define GLHELPER_H_
#include <GL/glew.h> // should be included before "wx/glcanvas.h"
#include <GL/glu.h>
#include <GL/gl.h>
#include "GL/glut.h"

inline void glpt3(double* p) {
	glVertex3f(p[0], p[1], p[2]);
}
inline void glquiver3(double* p, double *v, double alpha = 1.0) {
	glVertex3f(p[0], p[1], p[2]);
	glVertex3f(alpha * v[0] + p[0], alpha * v[1] + p[1], alpha * v[2] + p[2]);
}
inline void glline(double* p, double *q) {
	glVertex3f(p[0], p[1], p[2]);
	glVertex3f(q[0], q[1], q[2]);
}
/* c = alpha*a + beta*b */
inline void add(const double* a, const double* b, double * c, double alpha = 1.0, double beta = 1.0) {
	c[0] = alpha * a[0] + beta * b[0];
	c[1] = alpha * a[1] + beta * b[1];
	c[2] = alpha * a[2] + beta * b[2];
}
/* b = alpha*a */
inline void mul(const double* a, double* b, double alpha) {
	b[0] = alpha * a[0];
	b[1] = alpha * a[1];
	b[2] = alpha * a[2];
}
void drawCircle(float cx, float cy, float r, int num_segments);
void drawLine(float x0, float y0, float x1, float y1);
void drawBlock(float l, float t, float w, float h, const float* color);
void glutPrint2D(float x, float y, const char* text, float r, float g, float b, float a, bool large);
void glutPrint3D(float x, float y, float z, const char* text, float r, float g, float b, float a, bool large);

/* get row major model view matrix*/
void getModelViewMatrix(double M[16]);
void getModelViewMats(double R[9], double t[3]);
void setModelViewMatrix(const double M[16]);
void setModelViewMats(const double R[9], const double t[3]);
void getRotationTranslation(const double MT[16], double R[9], double t[3]);
void setRotationTranslation(const double R[9], const double t[3], double MT[16]);

/* draw camera pose*/
void drawCameraPose(int id,
		const double R[9],
		const double t[3],
		double scale,
		double cr = 0,
		double cg = 0,
		double cb = 0,
		double linewidth = 1.0);

/* draw covariance matrix*/
void drawCovariance(double x, double y, double z, const double cov[9], double cr, double cg, double cb);
#endif /* GLHELPER_H_ */
