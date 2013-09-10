/*
 * GLImagePane.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef GLIMAGEPANE_H_
#define GLIMAGEPANE_H_

#include <GL/glew.h> // should be included before "wx/glcanvas.h"
#include "wx/wx.h"
#include "wx/glcanvas.h"

#include "app/SL_CoSLAM.h"
class GLImagePane: public wxGLCanvas {
	wxGLContext* m_context;
	unsigned char* imgData;
	int imgWidth, imgHeight;
	int m_camId;
	CoSLAM* m_pSLAM;

	//store pointers to the feature points
	std::vector<FeaturePoint*> m_pFeatPts;
	//store current camera pose
	CamPoseItem* m_pCamPos;
	//store the convex hulls
	std::vector<double> m_convexHulls[SLAM_MAX_NUM][SLAM_MAX_NUM];
	double m_overlapCost[SLAM_MAX_NUM * SLAM_MAX_NUM];

	GLuint m_imgTexture;
	bool b_haveImageData;
	bool b_firstRun;
	bool b_drawReprojectionError;
	//clicked feature points
	static int s_clicked_camid;
	static FeaturePoint* s_clicked_featPt;
	static MapPoint* s_clicked_mappt;
public:
	GLImagePane(wxFrame* parent, int* args);
	virtual ~GLImagePane();

	void resized(wxSizeEvent& evt);

	int getWidth();
	int getHeight();

	void draw();
	void render(wxPaintEvent& evt);
	void prepare2DViewport(int topleft_x, int topleft_y, int bottomrigth_x,
			int bottomrigth_y);

	// events
DECLARE_EVENT_TABLE()
	void mouseMoved(wxMouseEvent& event);
	void mouseDown(wxMouseEvent& event);
	void mouseWheelMoved(wxMouseEvent& event);
	void mouseReleased(wxMouseEvent& event);
	void rightClick(wxMouseEvent& event);
	void mouseLeftWindow(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void keyReleased(wxKeyEvent& event);
public:
	void convert2ImageCoords(float x0, float y0, float& x, float& y);
	void drawFeaturePoints();
	void drawConvexHulls();
	void drawClickedPoint(ImgRGB& img);
	void drawEpipolarLine(ImgRGB& img);
	void drawBoundBox();
	void initImageData(int w, int h);
	void copyDisplayData();
	void setSLAMData(int id, CoSLAM* pSLAM) {
		m_camId = id;
		m_pSLAM = pSLAM;
	}
	void saveScreen(const char* filePath);
	void drawInfo(int topleft_x, int topleft_y, int bottomrigth_x,
			int bottomrigth_y);
};
#endif /* GLIMAGEPANE_H_ */
