/*
 * CoSLAMThread.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */
#ifndef COSLAMTHREAD_H_
#define COSLAMTHREAD_H_
#include "wx/wx.h"
#include "wx/event.h"
#include "wx/thread.h"
#include "pthread.h"

DECLARE_EVENT_TYPE(EventUpdateViews,-1)
;

class CoSLAMThread: public wxThread {
public:
	CoSLAMThread();
	~CoSLAMThread();
protected:
	virtual ExitCode Entry();
};

void updateDisplayData();
void redrawAllViews();

#endif /* COSLAMTHREAD_H_ */
