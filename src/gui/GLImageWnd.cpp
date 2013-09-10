/*
 * GLImageWnd.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#include "GLImageWnd.h"

GLImageWnd::~GLImageWnd() {

}

GLImageWnd::GLImageWnd(const char* wndName , int x0 , int y0 , int W , int H) :
	wxFrame((wxFrame *) NULL, -1, wndName, wxPoint(x0, y0), wxSize(W, H)) {
	int args[] = { WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0 };
	glPane = new GLImagePane((wxFrame*) this, args);
}

