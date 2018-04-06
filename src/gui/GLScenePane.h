/*
 * GLScenePane.h
 *
 *  Created on: 2011-7-15
 *      Author: zou
 */

#ifndef GLSCENEPANE_H_
#define GLSCENEPANE_H_
#include "app/SL_CoSLAM.h"
#include "GL/glew.h"
#include "GLTrackballPane.h"

using namespace std;
class GLScenePane: public GLTrackballPane {
public:
	GLScenePane(wxWindow * parent, wxWindowID id, int args[]) :
			GLTrackballPane(parent, id, args) {
		//memset(m_center, 0, sizeof(double) * 3);
		fill_n(m_center, 3, 0.0);
		m_pSLAM = 0;
		m_scale = 1.0;
		m_camSize = 0.05;
		m_autoScale = true;
		m_drawDynTrj = true;
		m_nTrjLen = 150;
		m_followCamId = -1;
		m_camView = false;
		m_pointSize = 1.0;
	}
	virtual ~GLScenePane() {
	}
public:
	double m_pointSize;
	int m_followCamId;
	bool m_camView;
	bool m_autoScale;
	bool m_drawDynTrj;
	int m_nTrjLen;
	double m_camSize;
	double m_center[3];
	double m_scale;

	/* slam data*/
	const CoSLAM* m_pSLAM;
	std::vector<const MapPoint*> curMapPoints;
	std::vector<const MapPoint*> actMapPoints;
	std::vector<const MapPoint*> iactMapPoints;
	std::vector<std::vector<Point3dId> > dynTracks;

	CameraGroup m_groups[SLAM_MAX_NUM];
	int m_groupNum;
	/* copy SLAM data*/
	void setSLAMData(const CoSLAM* pSLAM) {
		m_pSLAM = pSLAM;
	}
	void copyDispData();
	void getSceneScale();
	static float CAMERA_COLORS[3 * SLAM_MAX_NUM];
public:
	void drawPoints();
	void drawCameras();
	void drawInfo();
	void drawCameraGroups();
	virtual void drawGLObjs();
	void saveScreen(const char* filePath);
public:
	void resized(wxSizeEvent& evt);
	void rightClick(wxMouseEvent& event);
	void keyPressed(wxKeyEvent& event);
	void charPressed(wxKeyEvent& event);DECLARE_EVENT_TABLE()

public:
	void save(const char* file_path);
	bool load(const char* file_path);
};

#endif /* GLSCENEPANE_H_ */
