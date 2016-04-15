/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2012 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#pragma once
#include "util_pipeline_stackable.h"
#include "pxcemotion.h"

class UtilPipelineEmotion:protected UtilPipelineStackable {
public:

	UtilPipelineEmotion(UtilPipelineStackable *next=0);

	virtual void  EnableEmotion(pxcUID iuid=0);
	virtual void  EnableEmotion(pxcCHAR *name);

    virtual void  PauseEmotion(bool pause) { m_emotion_pause=pause; }

	virtual void  OnEmotionSetup(PXCEmotion::ProfileInfo * /*finfo*/) {}
	virtual PXCEmotion* QueryEmotion(void) { return m_emotion; }

protected:

	PXCEmotion*						m_emotion;
	PXCEmotion::ProfileInfo			m_emotion_pinfo;
	bool							m_emotion_enabled;

	int								m_emotion_stream_index;
	PXCSession::ImplDesc			m_emotion_mdesc;
    bool                            m_emotion_pause;

	virtual bool      StackableCreate(PXCSession *session);
	virtual pxcStatus StackableSearchProfiles(UtilCapture *capture, std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs, int vidx, std::vector<PXCCapture::AudioStream::DataDesc*> &ainputs, int aidx);
	virtual bool      StackableSetProfile(UtilCapture *capture);
	virtual bool      StackableReadSample(UtilCapture *capture,PXCSmartArray<PXCImage> &images,PXCSmartSPArray &sps,pxcU32 isps);
	virtual void      StackableCleanUp(void);
};

