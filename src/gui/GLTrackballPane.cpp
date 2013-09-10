/*
 * GLTrackBallPane.cpp
 *
 *  Created on: 2011-6-10
 *      Author: tsou
 */
#include "GLTrackballPane.h"

static void invertMatrix(const GLdouble *m, GLdouble *out);
/*****************************************************************
 * Utility functions
 *****************************************************************/

static double vlen(double x, double y, double z) {
	return sqrt(x * x + y * y + z * z);
}

void GLTrackballPane::pos(double *px, double *py, double *pz, const int x,
		const int y, const int *viewport) {
	/*
	 Use the ortho projection and viewport information
	 to map from mouse co-ordinates back into world
	 co-ordinates
	 */

	*px = (double) (x - viewport[0]) / (double) (viewport[2]);
	*py = (double) (y - viewport[1]) / (double) (viewport[3]);

	*px = _left + (*px) * (_right - _left);
	*py = _top + (*py) * (_bottom - _top);
	*pz = _zNear;
}
void GLTrackballPane::setMatrix() {
	glLoadMatrixd(_matrix);
}
void GLTrackballPane::getMatrix() {
	glGetDoublev(GL_MODELVIEW_MATRIX, _matrix);
	invertMatrix(_matrix, _matrixInverse);
}
/*
 * From Mesa-2.2\src\glu\project.c
 *
 * Compute the inverse of a 4x4 matrix.  Contributed by scotter@lafn.org
 */

static void invertMatrix(const GLdouble *m, GLdouble *out) {

	/* NB. OpenGL Matrices are COLUMN major. */
#define MAT(m,r,c) (m)[(c)*4+(r)]

	/* Here's some shorthand converting standard (row,column) to index. */
#define m11 MAT(m,0,0)
#define m12 MAT(m,0,1)
#define m13 MAT(m,0,2)
#define m14 MAT(m,0,3)
#define m21 MAT(m,1,0)
#define m22 MAT(m,1,1)
#define m23 MAT(m,1,2)
#define m24 MAT(m,1,3)
#define m31 MAT(m,2,0)
#define m32 MAT(m,2,1)
#define m33 MAT(m,2,2)
#define m34 MAT(m,2,3)
#define m41 MAT(m,3,0)
#define m42 MAT(m,3,1)
#define m43 MAT(m,3,2)
#define m44 MAT(m,3,3)

	GLdouble det;
	GLdouble d12, d13, d23, d24, d34, d41;
	GLdouble tmp[16]; /* Allow out == in. */

	/* Inverse = adjoint / det. (See linear algebra texts.)*/

	/* pre-compute 2x2 dets for last two rows when computing */
	/* cofactors of first two rows. */
	d12 = (m31 * m42 - m41 * m32);
	d13 = (m31 * m43 - m41 * m33);
	d23 = (m32 * m43 - m42 * m33);
	d24 = (m32 * m44 - m42 * m34);
	d34 = (m33 * m44 - m43 * m34);
	d41 = (m34 * m41 - m44 * m31);

	tmp[0] = (m22 * d34 - m23 * d24 + m24 * d23);
	tmp[1] = -(m21 * d34 + m23 * d41 + m24 * d13);
	tmp[2] = (m21 * d24 + m22 * d41 + m24 * d12);
	tmp[3] = -(m21 * d23 - m22 * d13 + m23 * d12);

	/* Compute determinant as early as possible using these cofactors. */
	det = m11 * tmp[0] + m12 * tmp[1] + m13 * tmp[2] + m14 * tmp[3];

	/* Run singularity test. */
	if (det == 0.0) {
		/* printf("invert_matrix: Warning: Singular matrix.\n"); */
		/*    memcpy(out,_identity,16*sizeof(double)); */
	} else {
		GLdouble invDet = 1.0 / det;
		/* Compute rest of inverse. */
		tmp[0] *= invDet;
		tmp[1] *= invDet;
		tmp[2] *= invDet;
		tmp[3] *= invDet;

		tmp[4] = -(m12 * d34 - m13 * d24 + m14 * d23) * invDet;
		tmp[5] = (m11 * d34 + m13 * d41 + m14 * d13) * invDet;
		tmp[6] = -(m11 * d24 + m12 * d41 + m14 * d12) * invDet;
		tmp[7] = (m11 * d23 - m12 * d13 + m13 * d12) * invDet;

		/* Pre-compute 2x2 dets for first two rows when computing */
		/* cofactors of last two rows. */
		d12 = m11 * m22 - m21 * m12;
		d13 = m11 * m23 - m21 * m13;
		d23 = m12 * m23 - m22 * m13;
		d24 = m12 * m24 - m22 * m14;
		d34 = m13 * m24 - m23 * m14;
		d41 = m14 * m21 - m24 * m11;

		tmp[8] = (m42 * d34 - m43 * d24 + m44 * d23) * invDet;
		tmp[9] = -(m41 * d34 + m43 * d41 + m44 * d13) * invDet;
		tmp[10] = (m41 * d24 + m42 * d41 + m44 * d12) * invDet;
		tmp[11] = -(m41 * d23 - m42 * d13 + m43 * d12) * invDet;
		tmp[12] = -(m32 * d34 - m33 * d24 + m34 * d23) * invDet;
		tmp[13] = (m31 * d34 + m33 * d41 + m34 * d13) * invDet;
		tmp[14] = -(m31 * d24 + m32 * d41 + m34 * d12) * invDet;
		tmp[15] = (m31 * d23 - m32 * d13 + m33 * d12) * invDet;

		memcpy(out, tmp, 16 * sizeof(GLdouble));
	}

#undef m11
#undef m12
#undef m13
#undef m14
#undef m21
#undef m22
#undef m23
#undef m24
#undef m31
#undef m32
#undef m33
#undef m34
#undef m41
#undef m42
#undef m43
#undef m44
#undef MAT
}
BEGIN_EVENT_TABLE(GLTrackballPane, wxGLCanvas) EVT_MOTION(GLTrackballPane::mouseMoved) EVT_LEFT_DOWN(GLTrackballPane::leftMouseDown)
EVT_LEFT_UP(GLTrackballPane::leftMouseUp)
EVT_MIDDLE_DOWN(GLTrackballPane::midMouseDown)
EVT_MIDDLE_UP(GLTrackballPane::midMouseUp)
EVT_RIGHT_DOWN(GLTrackballPane::rightClick)
EVT_LEAVE_WINDOW(GLTrackballPane::mouseLeftWindow)
EVT_SIZE(GLTrackballPane::resized)
EVT_KEY_DOWN(GLTrackballPane::keyPressed)
EVT_KEY_UP(GLTrackballPane::keyReleased)
EVT_MOUSEWHEEL(GLTrackballPane::mouseWheelMoved)
EVT_PAINT(GLTrackballPane::render)
END_EVENT_TABLE()

static void drawAxes(void) {
	static GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
	static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
	static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	static GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };

	static GLfloat mat_ambient[] = { 0.7, 0.7, 0.7, 1.0 };
	static GLfloat mat_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
	static GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	static GLfloat high_shininess[] = { 100.0 };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);

	/* Name-stack manipulation for the purpose of
	 selection hit processing when mouse button
	 is pressed.  Names are ignored in normal
	 OpenGL rendering mode.                    */

	/* No name for grey sphere */
	glScaled(0.1, 0.1, 0.1);
	glTranslated(1, 1, 1);
	glColor3f(0.3, 0.3, 0.3);
	glutSolidSphere(0.7, 20, 20);

	glPushMatrix();
	glPushName(1); /* Red cone is 1 */
	glColor3f(1, 0, 0);
	glRotatef(90, 0, 1, 0);
	glutSolidCone(0.6, 4.0, 20, 20);
	glPopName();
	glPopMatrix();

	glPushMatrix();
	glPushName(2); /* Green cone is 2 */
	glColor3f(0, 1, 0);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(0.6, 4.0, 20, 20);
	glPopName();
	glPopMatrix();

	glColor3f(0, 0, 1); /* Blue cone is 3 */
	glPushName(3);
	glutSolidCone(0.6, 4.0, 20, 20);
	glPopName();
}

GLTrackballPane::~GLTrackballPane() {
	delete m_context;
}

void GLTrackballPane::prepareViewport() {
	int w = GetSize().x;
	int h = GetSize().y;
	glViewport(0, 0, w, h);

	_top = 1.0;
	_bottom = -1.0;
	_left = -(double) w / (double) h;
	_right = -_left;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(_left, _right, _bottom, _top, _zNear, _zFar);

	glMatrixMode(GL_MODELVIEW);
	if (_firstRun && _matrix[0] > 0.0) {
		setMatrix();
		_firstRun = false;
	}
	getMatrix();
}
void GLTrackballPane::drawGLObjs() {
	drawAxes();
}
void GLTrackballPane::draw(bool inPaintEvts = false) {
	SetCurrent(*m_context);
	if (inPaintEvts)
		wxPaintDC(this);
	else
		wxClientDC(this);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_ACCUM_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	prepareViewport();

	glPushMatrix();
	drawGLObjs();
	glPopMatrix();
	glFlush();
	SwapBuffers();
}
void GLTrackballPane::render(wxPaintEvent& evt) {
	if (!IsShown())
		return;
	draw(true);
}
void GLTrackballPane::resized(wxSizeEvent& evt) {
	prepareViewport();
	Refresh();
}
void GLTrackballPane::mouseMoved(wxMouseEvent& event) {
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	if (event.Dragging() && event.LeftIsDown()) {
		int new_mousex = event.GetX();
		int new_mousey = event.GetY();

		const int dx = new_mousex - _mouseX;
		const int dy = new_mousey - _mouseY;

		double ax, ay, az;
		double bx, by, bz;
		double angle;

		ax = dy;
		ay = dx;
		az = 0.0;
		angle = vlen(ax, ay, az) / (double) (viewport[2]) * 180.0;

		/* Use inverse matrix to determine local axis of rotation */
		bx = _matrixInverse[0] * ax + _matrixInverse[4] * ay
				+ _matrixInverse[8] * az;
		by = _matrixInverse[1] * ax + _matrixInverse[5] * ay
				+ _matrixInverse[9] * az;
		bz = _matrixInverse[2] * ax + _matrixInverse[6] * ay
				+ _matrixInverse[10] * az;

		glTranslatef(_refPoint[0], _refPoint[1], _refPoint[2]);
		glRotatef(angle, bx, by, bz);
		glTranslatef(-_refPoint[0], -_refPoint[1], -_refPoint[2]);

		//Refresh(); // prevent flashing
		draw(false);
	}
	if (event.Dragging() && event.MiddleIsDown()) {
		int x = event.GetX();
		int y = event.GetY();
		double px, py, pz;
		pos(&px, &py, &pz, x, y, viewport);

		glLoadIdentity();
		glTranslatef(px - _dragPosX, py - _dragPosY, pz - _dragPosZ);
		glMultMatrixd(_matrix);

		_dragPosX = px;
		_dragPosY = py;
		_dragPosZ = pz;
		Refresh();
	}
	_mouseX = event.GetX();
	_mouseY = event.GetY();

}
void GLTrackballPane::leftMouseDown(wxMouseEvent& event) {
}
void GLTrackballPane::leftMouseUp(wxMouseEvent& event) {
}
void GLTrackballPane::midMouseDown(wxMouseEvent& event) {
	int x = event.GetX();
	int y = event.GetY();
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	pos(&_dragPosX, &_dragPosY, &_dragPosZ, x, y, viewport);
	Refresh();
}
void GLTrackballPane::midMouseUp(wxMouseEvent& event) {
}
void GLTrackballPane::mouseWheelMoved(wxMouseEvent& event) {
	int i = event.GetWheelRotation() / event.GetWheelDelta();
	double s = exp((double) i * 0.1);
	glTranslatef(_refPoint[0], _refPoint[1], _refPoint[2]);
	glScalef(s, s, s);
	glTranslatef(-_refPoint[0], -_refPoint[1], -_refPoint[2]);
	//Refresh();
	draw(false);
}
void GLTrackballPane::rightClick(wxMouseEvent& event) {
}
void GLTrackballPane::mouseLeftWindow(wxMouseEvent& event) {
}
void GLTrackballPane::keyPressed(wxKeyEvent& event) {
}
void GLTrackballPane::keyReleased(wxKeyEvent& event) {
}
