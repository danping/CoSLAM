/*
 * CoSLAMThread.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#include "CoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_CoSLAM.h"
#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"

#include "gui/MyApp.h"

#ifdef WIN32
#include <io.h>  
#include <process.h>  
#else
#include <unistd.h>
#endif

DEFINE_EVENT_TYPE(EventUpdateViews);

CoSLAMThread::CoSLAMThread() {
}
CoSLAMThread::~CoSLAMThread() {
}
void updateDisplayData() {
	CoSLAM& coSLAM = MyApp::coSLAM;
	for (int i = 0; i < coSLAM.numCams; i++) {
		if (!MyApp::bBusyDrawingVideo[i])
			MyApp::videoWnd[i]->copyDataForDisplay();
	}
	//	MyApp::s_mutexUpdateDispData.Unlock();
	if (!MyApp::bBusyDrawingModel) {
		MyApp::modelWnd1->copyDispData();
		MyApp::modelWnd2->copyDispData();
	}
}
void redrawAllViews() {
	MyApp::redrawViews();
}
CoSLAMThread::ExitCode CoSLAMThread::Entry() {
	CoSLAM& coSLAM = MyApp::coSLAM;
	/////////////////////////1.GPU initilization/////////////////////////
	//initialization for CG;
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(" ");
	glutHideWindow();

	glewInit();

	V3D_GPU::Cg_ProgramBase::initializeCg();

	//////////////////////////2.read video information//////////////////
	try {
		for(int c = 0; c < coSLAM.numCams; c++){
			coSLAM.slam[c].videoReader = &MyApp::aviReader[c];
		}
		
		coSLAM.init();
		MyApp::bInitSucc = true;
		logInfo("Loading video sequences.. OK!\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		return 0;
	}

	//notify the GUI thread to create GUIs
	MyApp::broadcastCreateGUI();

	//wait for the accomplishment of creating GUIs
	MyApp::waitCreateGUI();

	for (int i = 0; i < coSLAM.numCams; i++)
		MyApp::videoWnd[i]->setSLAMData(i, &coSLAM);

	MyApp::modelWnd1->setSLAMData(&coSLAM);
	MyApp::modelWnd2->setSLAMData(&coSLAM);

	/* start the SLAM process*/
	try {
		coSLAM.readFrame();
		//copy the data to buffers for display
		updateDisplayData();
		//initialise the map points
		tic();
		coSLAM.initMap();
		toc();

        for( int i = 1; i < Param::nTotalFrame; i++){
			while (MyApp::bStop) {/*stop*/
			}

			TimeMeasurer tmPerStep;
			tmPerStep.tic();

			coSLAM.grabReadFrame();
			coSLAM.featureTracking();
			coSLAM.poseUpdate();
			coSLAM.cameraGrouping();
            
			//existing 3D to 2D points robust
			coSLAM.activeMapPointsRegister(Const::PIXEL_ERR_VAR);

			TimeMeasurer tmNewMapPoints;
			tmNewMapPoints.tic();

			coSLAM.genNewMapPoints();
			coSLAM.m_tmNewMapPoints = tmNewMapPoints.toc();

			//point registration
			coSLAM.currentMapPointsRegister(Const::PIXEL_ERR_VAR,
					i % 50 == 0 ? true : false);

			coSLAM.storeDynamicPoints();

			updateDisplayData();
			redrawAllViews();

			coSLAM.m_tmPerStep = tmPerStep.toc();
			Sleep(50);					
            
            cout << "f:" << coSLAM.curFrame << "/" << Param::nTotalFrame << endl;
		}
		cout << " the result is saved at " << MyApp::timeStr << endl;
		coSLAM.exportResults(MyApp::timeStr);
		logInfo("slam finished\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
	} catch (std::exception& e) {
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		logInfo("%s\n", e.what());
		logInfo("slam failed!\n");
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	}
	logInfo("\nslam stopped!\n");
	return 0;
}
