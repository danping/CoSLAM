/*
 * GLTrackballPane.h
 *
 *  Created on: 2011-6-10
 *      Author: tsou
 */

#ifndef GLTRACKBALL_H_
#define GLTRACKBALL_H_
#include "wx/wx.h"
#include "wx/glcanvas.h"
#include <GL/gl.h>
#include <GL/glut.h>
class GLTrackballPane: public wxGLCanvas {
protected:
	bool _firstRun;
	double _left;
	double _right;
	double _bottom;
	double _top;
	double _zNear;
	double _zFar;

	int _mouseX;
	int _mouseY;

	double _dragPosX;
	double _dragPosY;
	double _dragPosZ;

	double _matrix[16];
	double _matrixInverse[16];
	double _refPoint[3];

public:
	GLTrackballPane(wxWindow * parent, wxWindowID id, int args[]) :
			wxGLCanvas(parent, id, args, wxDefaultPosition, wxDefaultSize,
					wxFULL_REPAINT_ON_RESIZE) {
		m_context = new wxGLContext(this);
		_firstRun = true;
		_left = 0.0;
		_right = 0.0;
		_bottom = 0.0;
		_top = 0.0;
		_zNear = -10.0;
		_zFar = 10.0;

		_mouseX = 0;
		_mouseY = 0;

		_dragPosX = 0.0;
		_dragPosY = 0.0;
		_dragPosZ = 0.0;

		std::fill_n(_matrix, 16, 0.0);
		std::fill_n(_refPoint, 3, 0.0);
	}
	virtual ~GLTrackballPane();
	virtual void drawGLObjs();
	virtual void draw(bool inPaintEvts);
	virtual void render(wxPaintEvent& evt);
protected:
	void pos(double *px, double *py, double *pz, const int x, const int y,
			const int *viewport);
	void setMatrix();
	void getMatrix();
public:
	wxGLContext* m_context;
	// events
	void prepareViewport();
	void mouseMoved(wxMouseEvent& event);
	void leftMouseDown(wxMouseEvent& event);
	void leftMouseUp(wxMouseEvent& event);

	void midMouseDown(wxMouseEvent& event);
	void midMouseUp(wxMouseEvent& event);

	void mouseWheelMoved(wxMouseEvent& event);
	void mouseReleased(wxMouseEvent& event);
	void rightClick(wxMouseEvent& event);
	void mouseLeftWindow(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void keyReleased(wxKeyEvent& event);
	void resized(wxSizeEvent& evt);DECLARE_EVENT_TABLE()
	;
};

#endif /* GLPANE_H_ */
