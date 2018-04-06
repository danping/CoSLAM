/*
 * MyApp.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#ifndef MYAPP_H_
#define MYAPP_H_
#undef Status
#include <opencv2/opencv.hpp>

#include "app/SL_CoSLAM.h"
#include "app/SL_GlobParam.h"

#include "GLImageWnd.h"
#include "GLSceneWnd.h"
#include "CoSLAMThread.h"

#include "tools/SL_AVIReader.h"

class MyApp: public wxApp {
protected:
	void initSyncVars();
	virtual int OnExit();
	virtual bool OnInit();
public:
	static GLImageWnd* videoWnd[SLAM_MAX_NUM];
	static GLSceneWnd* modelWnd1;
	static GLSceneWnd* modelWnd2;
	static double videoWndScale;

	static AVIReader aviReader[SLAM_MAX_NUM];
	static CoSLAM coSLAM;

	static pthread_mutex_t s_mutexCreateGUI;
	static pthread_cond_t s_condCreateGUI;
	static pthread_mutex_t s_mutexBA;

	static bool bInitSucc;
	static bool bBusyDrawingVideo[SLAM_MAX_NUM];
	static bool bBusyDrawingModel;

	/*flag for busy running bundle adjustment*/
	static bool bBusyBAing;
	static bool bCancelBA;	//when group merge happens 
	static MyApp* app;

	static CoSLAMThread *coSlamThread;

	static void preWaitCreateGUI();
	static void waitCreateGUI();
	static void broadcastCreateGUI();
	DECLARE_EVENT_TABLE();
public:
	static void redrawViews();
	void onUpdateViews(wxCommandEvent& evt);
    void parseInput();
public:
	static bool bStop;
	static bool bSingleStep;
	static char timeStr[256];
};

void getCurTimeString(char* timeStr);
#endif /* MYAPP_H_ */
