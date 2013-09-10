/*
 * GLImageWnd.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef GLIMAGEWND_H_
#define GLIMAGEWND_H_
#include "GLImagePane.h"
class GLImageWnd : public wxFrame {
protected:
	GLImagePane* glPane;
public:
	virtual ~GLImageWnd();
	GLImageWnd(const char* wndName , int x0 , int y0 , int W , int H);
public:
	void initVideoFrame(int W , int H) {
		glPane->initImageData(W, H);
	}
	void copyDataForDisplay() {
		glPane->copyDisplayData();
	}
	void setSLAMData(int id , CoSLAM* pSLAM) {
		glPane->setSLAMData(id, pSLAM);
	}
	void redraw() {
		glPane->draw();
	}
	void save(const char* filePath) {
		glPane->saveScreen(filePath);
	}
};

#endif /* GLIMAGEWND_H_ */
