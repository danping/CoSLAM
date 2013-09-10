/*
 * GLSceneWnd.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef GLSCENEWND_H_
#define GLSCENEWND_H_
#include "GLScenePane.h"
#include "wx/event.h"
#include "wx/wx.h"
class GLSceneWnd: public wxFrame {
public:
	GLScenePane* glPane;
	virtual ~GLSceneWnd();
	GLSceneWnd(const char* wndName, int x0, int y0, int W, int H);
	void setSLAMData(CoSLAM* pSLAM) {
		glPane->setSLAMData(pSLAM);
	}
	void copyDispData() {
		glPane->copyDispData();
	}
	void redraw() {
		glPane->draw(false);
	}
	void save(const char* filePath) {
		glPane->saveScreen(filePath);
	}
};

#endif /* GLSCENEWND_H_ */
