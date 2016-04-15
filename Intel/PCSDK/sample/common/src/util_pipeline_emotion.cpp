/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2012 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include "util_pipeline_emotion.h"

bool UtilPipelineEmotion::StackableCreate(PXCSession *session) {
    if (m_emotion_enabled) {
        pxcStatus sts=session->CreateImpl<PXCEmotion>(&m_emotion_mdesc,&m_emotion);
        if (sts<PXC_STATUS_NO_ERROR) return false;
    }
	return UtilPipelineStackable::StackableCreate(session);
}

bool UtilPipelineEmotion::StackableSetProfile(UtilCapture *capture) {
	if (m_emotion_enabled) {
		if (!m_emotion) return false;
		OnEmotionSetup(&m_emotion_pinfo);
		pxcStatus sts=m_emotion->SetProfile(&m_emotion_pinfo);
		if (sts<PXC_STATUS_NO_ERROR) return false;

		for (pxcU32 i=0;;i++) {
			PXCEmotion::ProfileInfo pinfo;
			sts=m_emotion->QueryProfile(i,&pinfo);
			if (sts<PXC_STATUS_NO_ERROR) return false;
			OnEmotionSetup(&pinfo);
			sts=m_emotion->SetProfile(&pinfo);
			if (sts>=PXC_STATUS_NO_ERROR) break;
		}
		m_emotion_pause=false;
	}

	return UtilPipelineStackable::StackableSetProfile(capture);
}

bool UtilPipelineEmotion::StackableReadSample(UtilCapture *capture,PXCSmartArray<PXCImage> &images,PXCSmartSPArray &sps,pxcU32 isps) {
    if (m_emotion && (!m_emotion_pause)) {
        PXCCapture::VideoStream::Images images2;
        capture->MapImages(m_emotion_stream_index,images,images2);
        pxcStatus sts=m_emotion->ProcessImageAsync(images2,sps.ReleaseRef(isps));
		if (sts<PXC_STATUS_NO_ERROR) return false;
    }
    return UtilPipelineStackable::StackableReadSample(capture,images,sps,isps);
}

void UtilPipelineEmotion::StackableCleanUp(void) {
	if (m_emotion)	{ m_emotion->Release(); m_emotion=0; }
	UtilPipelineStackable::StackableCleanUp();
}

pxcStatus UtilPipelineEmotion::StackableSearchProfiles(UtilCapture *uc, std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs, int vidx, std::vector<PXCCapture::AudioStream::DataDesc*> &ainputs, int aidx) {
	if (!m_emotion) return UtilPipelineStackable::StackableSearchProfiles(uc,vinputs,vidx,ainputs,aidx);

	m_emotion_stream_index=vidx;
	for (pxcU32 p=0;;p++) {
		pxcStatus sts=m_emotion->QueryProfile(p,&m_emotion_pinfo);
		if (sts<PXC_STATUS_NO_ERROR) break;

		if (vidx>=(int)vinputs.size()) vinputs.push_back(&m_emotion_pinfo.inputs);
		else vinputs[vidx]=&m_emotion_pinfo.inputs;

		sts=UtilPipelineStackable::StackableSearchProfiles(uc,vinputs,vidx+1,ainputs,aidx);
		if (sts>=PXC_STATUS_NO_ERROR) return sts;
		if (sts!=PXC_STATUS_ITEM_UNAVAILABLE) continue;

		sts=uc->LocateStreams(vinputs,ainputs);
		if (sts>=PXC_STATUS_NO_ERROR) return PXC_STATUS_NO_ERROR;
	}
	return PXC_STATUS_PARAM_UNSUPPORTED;
}

void UtilPipelineEmotion::EnableEmotion(pxcUID iuid) {
	m_emotion_enabled=true;
	m_emotion_mdesc.iuid=iuid;
}

void UtilPipelineEmotion::EnableEmotion(pxcCHAR *name) {
	m_emotion_enabled=true;
	wcscpy(m_emotion_mdesc.friendlyName,name);
	//wcscpy_s<sizeof(m_emotion_mdesc.friendlyName)/sizeof(pxcCHAR)>(m_emotion_mdesc.friendlyName,name);
}

UtilPipelineEmotion::UtilPipelineEmotion(UtilPipelineStackable *next):UtilPipelineStackable(next) {
	memset(&m_emotion_mdesc,0,sizeof(m_emotion_mdesc));
    m_emotion_enabled=false;
	m_emotion=0;
}
