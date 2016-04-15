/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include <Windows.h>
#include "util_pipeline_segmentation.h"

bool UtilPipelineSegmentation::StackableCreate(PXCSession *session) {
    if (m_segmentation_enabled) {
		m_session = session;
        pxcStatus sts=session->CreateImpl<PXCSegmentation>(&m_segmentation_mdesc,&m_segmentation);
        if (sts<PXC_STATUS_NO_ERROR) return false;
    }
	return UtilPipelineStackable::StackableCreate(session);
}

bool UtilPipelineSegmentation::StackableSetProfile(UtilCapture *capture) {
    if (m_segmentation) {
		OnSegmentationSetup(&m_segmentation_pinfo);
        pxcStatus sts=m_segmentation->SetProfile(&m_segmentation_pinfo);
        if (sts<PXC_STATUS_NO_ERROR) return false;
 
        m_pause=false;
    }
    return UtilPipelineStackable::StackableSetProfile(capture);
}


bool UtilPipelineSegmentation::StackableReadSample(UtilCapture *capture,PXCSmartArray<PXCImage> &images, PXCSmartSPArray &sps,pxcU32 isps) {
    if (m_segmentation && !m_pause) {
		pxcStatus sts;
        PXCCapture::VideoStream::Images images2;

		if (m_segImage) m_segImage->Release(); 
		if (m_enableBlend && m_blendedImage)  m_blendedImage->Release(); 

        capture->MapImages (m_segmentation_stream_index,images,images2);
	    if (!m_enableBlend) {
            sts=m_segmentation->SegmentImageAsync(images2, &m_segImage, sps.ReleaseRef(isps));
            if (sts<PXC_STATUS_NO_ERROR) return false;
        } else {
			PXCSmartSP  seg_sp;
			sts = m_segmentation->SegmentImageAsync(images2, &m_segImage, &seg_sp);
			if (sts<PXC_STATUS_NO_ERROR) return false;

			sts = m_segmentation->BlendImageAsync (capture->QueryImage(images, PXCImage::IMAGE_TYPE_COLOR),m_segImage, m_bgImage, &m_blendedImage, sps.ReleaseRef(isps));
			if (sts<PXC_STATUS_NO_ERROR) return false;
		}
    }
    return UtilPipelineStackable::StackableReadSample(capture,images,sps,isps);   
}

PXCImage* UtilPipelineSegmentation::QuerySegmentationImage() {
    return m_segImage;
}

PXCImage* UtilPipelineSegmentation::QuerySegmentationBlendedImage() {
    return m_blendedImage; 
}

void UtilPipelineSegmentation::StackableCleanUp(void) {
	if (m_segmentation)	{ 
		m_segmentation->Release(); m_segmentation=0; 
	}

	UtilPipelineStackable::StackableCleanUp();
}

pxcStatus UtilPipelineSegmentation::StackableSearchProfiles(UtilCapture *uc, std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs, int vidx, std::vector<PXCCapture::AudioStream::DataDesc*> &ainputs, int aidx) {
	if (!m_segmentation) return UtilPipelineStackable::StackableSearchProfiles(uc,vinputs,vidx,ainputs,aidx);

	m_segmentation_stream_index=vidx;
	for (int p=0;;p++) {
		pxcStatus sts=m_segmentation->QueryProfile(p,&m_segmentation_pinfo);
		if (sts<PXC_STATUS_NO_ERROR) break;

		if (vidx>=(int)vinputs.size()) vinputs.push_back(&m_segmentation_pinfo.inputs);
		else vinputs[vidx]=&m_segmentation_pinfo.inputs;

		sts=UtilPipelineStackable::StackableSearchProfiles(uc,vinputs,vidx+1,ainputs,aidx);
		if (sts>=PXC_STATUS_NO_ERROR) return sts;
		if (sts!=PXC_STATUS_ITEM_UNAVAILABLE) continue;

		sts=uc->LocateStreams(vinputs,ainputs);
		if (sts>=PXC_STATUS_NO_ERROR) return PXC_STATUS_NO_ERROR;
	}
	return PXC_STATUS_PARAM_UNSUPPORTED;
}

void UtilPipelineSegmentation::EnableSegmentation(pxcUID iuid) {
	m_segmentation_enabled=true;
	m_enableBlend = false; 
	m_segmentation_mdesc.iuid=iuid;
}

void UtilPipelineSegmentation::EnableSegmentation(pxcCHAR *name) {
	m_segmentation_enabled=true;
	m_enableBlend = false; 
	//wcscpy_s<sizeof(m_segmentation_mdesc.friendlyName)/sizeof(pxcCHAR)>(m_segmentation_mdesc.friendlyName,name);
	wcscpy(m_segmentation_mdesc.friendlyName,name);
}

void UtilPipelineSegmentation::SetSegmentationBlendMode(PXCSegmentation::BlendMode mode) {
	if (!m_segmentation) return;

	if (mode != PXCSegmentation::BLEND_ANY)
		m_enableBlend = true; 
	m_mode = mode;
	return;
}

pxcStatus UtilPipelineSegmentation::SetSegmentationBGImage(PXCImage* bgImage)
{
	if (!m_segmentation || !bgImage) return PXC_STATUS_HANDLE_INVALID;

	if (!m_bgImage && m_session) {
		PXCSmartPtr<PXCAccelerator> accelerator;
		m_session->CreateAccelerator(&accelerator);
		if (!accelerator.IsValid()) return PXC_STATUS_HANDLE_INVALID;
		
		pxcStatus sts;
		PXCImage::ImageInfo info;
		bgImage->QueryInfo(&info); 
		sts=accelerator->CreateImage(&info, 0, 0, &m_bgImage);
		if (sts < PXC_STATUS_NO_ERROR) return sts;
	}

	if (m_bgImage) m_bgImage->CopyData(bgImage); 

	return PXC_STATUS_NO_ERROR;
}

UtilPipelineSegmentation::UtilPipelineSegmentation(UtilPipelineStackable *next):UtilPipelineStackable(next) {
	memset(&m_segmentation_mdesc,0,sizeof(m_segmentation_mdesc));
	m_segmentation_enabled=false;
	m_enableBlend = false;
	m_mode = PXCSegmentation::BLEND_BG_BLUE;
	m_segmentation=0;
	m_session=0;
}
