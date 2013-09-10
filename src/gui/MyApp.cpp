/*
 * main.cpp
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */

#include "MyApp.h"
#include "tracking/CGKLT/v3d_gpuklt.h"
#include "CoSLAMThread.h"
#include "tools/SL_Timing.h"
#include "tools/SL_Tictoc.h"
#include <fstream>

GLImageWnd* MyApp::videoWnd[SLAM_MAX_NUM];
GLSceneWnd* MyApp::modelWnd1;
GLSceneWnd* MyApp::modelWnd2;

double MyApp::videoWndScale = 0.5;
AVIReader MyApp::aviReader[SLAM_MAX_NUM];
CoSLAM MyApp::coSLAM;

pthread_mutex_t MyApp::s_mutexCreateGUI;
pthread_cond_t MyApp::s_condCreateGUI;
pthread_mutex_t MyApp::s_mutexBA;

bool MyApp::bInitSucc = false;
bool MyApp::bBusyDrawingModel = false;
bool MyApp::bBusyDrawingVideo[SLAM_MAX_NUM];
bool MyApp::bBusyBAing = false;
bool MyApp::bCancelBA = false;

MyApp* MyApp::app = 0;

bool MyApp::bStop = false;
bool MyApp::bSingleStep = false;
char MyApp::timeStr[256];

CoSLAMThread* MyApp::coSlamThread;
BEGIN_EVENT_TABLE(MyApp, wxApp) EVT_COMMAND(wxID_ANY, EventUpdateViews, MyApp::onUpdateViews) END_EVENT_TABLE()
IMPLEMENT_APP(MyApp)
void MyApp::initSyncVars() {
	pthread_mutex_init(&s_mutexCreateGUI, 0);
	pthread_mutex_init(&s_mutexBA, 0);
	pthread_cond_init(&s_condCreateGUI, 0);
}
int MyApp::OnExit() {
	pthread_mutex_destroy(&s_mutexCreateGUI);
	pthread_mutex_destroy(&s_mutexBA);
	pthread_cond_destroy(&s_condCreateGUI);
	return 0;
}
void MyApp::preWaitCreateGUI() {
	pthread_mutex_lock(&s_mutexCreateGUI);
}
void MyApp::waitCreateGUI() {
	pthread_cond_wait(&s_condCreateGUI, &s_mutexCreateGUI);
#ifndef WIN32
	pthread_mutex_unlock(&s_mutexCreateGUI);
#endif
}
void MyApp::broadcastCreateGUI() {
#ifndef WIN32
	pthread_mutex_lock(&s_mutexCreateGUI);
#endif
	pthread_cond_signal(&s_condCreateGUI);
#ifndef WIN32
	pthread_mutex_unlock(&s_mutexCreateGUI);
#endif
}
///////////////////////////////////////////////////////////////////////////////////
void MyApp::redrawViews() {
	wxCommandEvent evt(EventUpdateViews, wxID_ANY);
	evt.SetInt(0);
	MyApp::app->AddPendingEvent(evt);
}
#define SAVE_VIEW_IMAGE
void MyApp::onUpdateViews(wxCommandEvent& evt) {
	for (int i = 0; i < coSLAM.numCams; i++) {
		videoWnd[i]->redraw();
	}
	modelWnd1->redraw();
	modelWnd2->redraw();
#ifdef SAVE_VIEW_IMAGE
	static bool bFirst = true;
	char dirPath[1024];
	//sprintf(dirPath, "/home/tsou/slam_results_video/%s", MyApp::timeStr);
	sprintf(dirPath,"/media/VIDEO_DATA_/slam_result_video2/%s",MyApp::timeStr);
#ifdef WIN32
	mkdir(dirPath);
#else
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

	if (coSLAM.curFrame > 0) {
		static int frame = coSLAM.curFrame;
		char filePath[256];
		for (int i = 0; i < coSLAM.numCams; i++) {
			sprintf(filePath, "%s/%dcam_%04d.ppm", dirPath, i, frame);
			videoWnd[i]->save(filePath);
		}
		sprintf(filePath, "%s/1map_%04d.ppm", dirPath, frame);
		modelWnd1->save(filePath);
		sprintf(filePath, "%s/2map_%04d.ppm", dirPath, frame);
		modelWnd2->save(filePath);
		frame++;
	}
#endif
}
void getCurTimeString(char* timeStr) {
	time_t rawtime;
	time(&rawtime);

	struct tm * timeinfo;
	timeinfo = localtime(&rawtime);
	strftime(timeStr, 256, "%y-%m-%d=%H-%M", timeinfo);
}

inline void tokenize(const std::string& str, std::vector<std::string>& tokens,
		const std::string delim = " ';\t") {
	using namespace std;
	size_t prev = 0, pos;
	tokens.clear();
	while ((pos = str.find_first_of(delim, prev)) != string::npos) {
		if (pos > prev)
			tokens.push_back(str.substr(prev, pos - prev));
		prev = pos + 1;
	}
	if (prev < str.length())
		tokens.push_back(str.substr(prev, string::npos));
}

template<class T>
inline void str2val(const string& str, T& val){
    stringstream ss(str);
    ss >> val;
}

void MyApp::parseInput(){
    if( argc != 2) {
        cout << "Usage:" << endl;
        cout << "\t CoSLAM <Input file>" << endl;
        Exit();
    }
    else{       
        std::string filePath = argv[1].ToStdString();
        
        ifstream file(filePath.c_str());
        if( !file)
            repErr("Cannot open '%s' to read!", filePath.c_str());
        
        int numView;
        
        string str;
        vector<string> tokens;
        std::getline(file, str);
        
        tokenize(str, tokens);        
        str2val(tokens[0], numView);
        
        if( numView < 1)
            repErr("Error in parsing the input file!");
        
        //parse video files
        int k = 0;
        while(getline(file,str)){
            tokenize(str,tokens);
            if( !tokens[0].empty()){
                if( k < numView){
                    if( tokens.size() != 2)
                        repErr("Error in parsing the input file!");
                    
                    int nStartFrm = 0;
                    int nInitFrm = 0;
        
                    str2val(tokens[0],nStartFrm);
                    str2val(tokens[1],nInitFrm);
                    
                    Param::nSkipFrms.push_back(nStartFrm);
                    Param::nInitFrms.push_back(nInitFrm);
                }
                else if( k < 2*numView){
                    Param::videoFilePath.push_back(tokens[0]);
                }
                else{
                    Param::camFilePath.push_back(tokens[0]);
                }
                k++;
            }
        }
        
        if( Param::videoFilePath.size() != numView || Param::videoFilePath.size() != numView)
            repErr("Error in parsing the input file!");
       
    }
}

bool MyApp::OnInit() {
	initSyncVars();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glEnable(GL_MULTISAMPLE);
	app = this;

	getCurTimeString(timeStr);
	   
    parseInput();
    
	
	Param::SSD_Threshold = 20000;
	Param::minCornerness = 3000;
	

    cout << "nViews:" << Param::videoFilePath.size() << endl;
	for (size_t i = 0; i < Param::videoFilePath.size(); ++i) {
		coSLAM.addInput(Param::videoFilePath[i].c_str(),
                        Param::camFilePath[i].c_str(), Param::nSkipFrms[i],Param::nInitFrms[i]);
        
        cout << "Skipped frames:" << Param::nSkipFrms[i] << endl;
        cout << "Init frames:" << Param::nInitFrms[i] << endl;
        cout << "Video input:" << Param::videoFilePath[i] << endl;
        cout << "Camera parameters:" << Param::camFilePath[i] << endl;
	}
  

	preWaitCreateGUI();

	//create main thread
	coSlamThread = new CoSLAMThread();
	coSlamThread->Create();
	coSlamThread->Run();

	waitCreateGUI();

	if (!bInitSucc) {
		wxMessageBox("Initialization failed!");
		return false;
	}
	modelWnd1 = new GLSceneWnd("model window1", 0, 0, 800, 450);
	modelWnd1->glPane->m_autoScale = true;
	modelWnd1->glPane->m_pointSize = 0.5;
	modelWnd1->Show();

	modelWnd2 = new GLSceneWnd("model window2", 600, 0, 800, 450);
	modelWnd2->glPane->m_autoScale = false;
	modelWnd2->glPane->m_camView = true;
	modelWnd2->glPane->m_followCamId = 0;
	modelWnd2->Show();

	char wndName[256];
	for (int i = 0; i < coSLAM.numCams; i++) {
		int W = coSLAM.slam[i].videoReader->_w;
		int H = coSLAM.slam[i].videoReader->_h;
		int Ws = W * videoWndScale;
		int Hs = H * videoWndScale;
		sprintf(wndName, "video %d", i);
		videoWnd[i] = new GLImageWnd(wndName, 300 + i * Ws, 600, Ws, Hs);
		videoWnd[i]->initVideoFrame(W, H);
		videoWnd[i]->Show();
	}
	//finish creating GUIs, broadcast the CoSLAMThread to start SLAM
	broadcastCreateGUI();
	modelWnd1->SetFocus();
	return true;
}
