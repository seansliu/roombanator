/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2012-2013 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include <set>
#include "util_captureimpl.h"
#include "util_capture_file.h"
#include "pxcmetadata.h"
#include <memory.h>

#pragma warning(disable:4482) /* enum used in qualified name */

/*  File Format:
        struct {
            pxcU32   ID = PXC_UID('P','X','C','F');
            pxcU32   fileVersion;
            pxcU32   reserved[24];
            pxcU32   firstFrameOffset;
            pxcU32   PXCCapture::CUID;
            pxcU32   sizeOfDeviceInfo;
            pxcU32   sizeOfStreamInfo;
            pxcU32   sizeOfVideoStreamProfileInfo;
            pxcU32   sizeOfAudioStreamProfileInfo;
        } header;
        PXCCapture::DeviceInfo  deviceInfo;
        int                     nproperties;
        struct {
            Property property;
            pxcF32   value;
        } properties[nproperties];
        int                     nvstreams;
        struct {
            PXCCapture::Device::StreamInfo          sinfo;
            PXCCapture::VideoStream::ProfileInfo    pinfo;
        } vstreams[nvstreams];
        int                     nastreams;
        struct {
            PXCCapture::Device::StreamInfo          sinfo;
            PXCCapture::AudioStream::ProfileInfo    pinfo;
        } astreams[nastreams];
        int                     nserializables; // for file version 2 and high (since 2012,December,15)
        struct {                                // for file version 2 and high
            pxcU32              dataSize;       // for file version 2 and high
            pxcBYTE             data[dataSize]; // for file version 2 and high
        } serializables[nserializables];        // for file version 2 and high
        struct {    // data frame starting point
            pxcU32  sidx;
            pxcU32  frame_bytes;    // timeStamp+video_frame|audio_frame
            pxcU64  timeStamp;
            union {
                pxcBYTE audio_frame[frame_bytes];
                struct {
                    union {
                        pxcBYTE color_image[pitch*height];
                        struct {
                            union {
                                pxcBYTE depth_plane[pitch*height];
                                pxcBYTE vertice_plane[pitch*height];
                            };
                            pxcBYTE irmap[pitch*height];            // if NO_IR_MAP is absent
                            pxcBYTE uvmap[pitch*height];            // if NO_UVMAP is absent
                        } depth_image;
                    };
                } video_frame;
            }
        } frames[];
*/

class UtilCaptureFile::DiskIO {
public:
    struct Header {
        pxcU32   id;
        pxcU32   fileVersion;
        pxcU32   reserved[24];
        pxcU32   firstFrameOffset;
        pxcUID   captureCuid;
        pxcU32   sizeOfDeviceInfo;
        pxcU32   sizeOfStreamInfo;
        pxcU32   sizeOfVideoStreamProfileInfo;
        pxcU32   sizeOfAudioStreamProfileInfo;
    };

	DiskIO(pxcCHAR *filename, pxcBool recording, pxcBool &sts);
	~DiskIO(void);

	/* seek operations */
	pxcU64 SeekSET(pxcU64 pos)	{ this->pos=pos; return pos; }
	pxcU64 SeekCUR(int nbytes)	{ pos+=nbytes; return pos; }

	void   NextFrame(void);
	void   SetFrameNo(int ifn);
	void   SetFrameTimeStamp(pxcU64 ts);
	int    GetFrameNo(void);
	pxcU64 GetFrameTimeStamp(void);
	void   SetPause(pxcBool pause);
	void   SelectStream(pxcI32 sidx) { selected.insert(sidx); }

	/* write operations */
	void   WriteHeader(void);
	void   WriteDeviceInfo(PXCCapture::DeviceInfo *dinfo);
	void   WriteDeviceProperties(std::map<PXCCapture::Device::Property,pxcF32> &properties);
	void   WriteVideoStreamInfo(std::vector<PXCCapture::Device::StreamInfo> &streams, std::vector<PXCCapture::VideoStream::ProfileInfo> &profiles);
	void   WriteAudioStreamInfo(PXCCapture::Device::StreamInfo *sinfo, PXCCapture::AudioStream::ProfileInfo *pinfo);
	void   WriteSerializables(pxcU32 nitems, pxcU32 nbytes, pxcBYTE *data);
	void   WriteFirstFrameOffset(void);
	void   WriteFrame(pxcU32 sidx, PXCImage *image);
	void   WriteFrame(pxcU32 sidx, PXCAudio *audio);

	/* read operations */
	void   ReadHeader(Header *header) { SeekSET(0); Read(header,sizeof(*header)); }
	void   ReadDeviceInfo(Header *header, PXCCapture::DeviceInfo *dinfo) { ReadStruct(dinfo,sizeof(*dinfo),header->sizeOfDeviceInfo); }
	void   ReadDeviceProperties(std::map<PXCCapture::Device::Property,pxcF32> &properties);
	void   ReadVideoStreamInfo(Header *header, std::vector<PXCCapture::Device::StreamInfo> &streams, std::vector<PXCCapture::VideoStream::ProfileInfo> &profiles);
	void   ReadAudioStreamInfo(Header *header, std::vector<PXCCapture::Device::StreamInfo> &sinfos, std::vector<PXCCapture::AudioStream::ProfileInfo> &pinfos);
	pxcU32 ReadSerializeables(Header *header, pxcBYTE *data, pxcU32 nbytesMax);
	void   SeekFirstFrame(Header *header);

	DWORD  Read(void *buffer, DWORD nbytes);
	DWORD  Read(PXCAudio *audio, DWORD nbytes);
	DWORD  Read(PXCImage *image, DWORD nbytes);

protected:

	pxcU64  pos, length, pos_next;
	HANDLE  gowrite, file, file2, thread;
	std::vector<std::pair<pxcU64,pxcU64> > indexed;
	std::set<pxcI32> selected;
	CRITICAL_SECTION cs;
	pxcBool pause;

	struct Block {
		Block	 *next;
		PXCImage *sample;
		pxcU32    sidx;
		pxcU32    nbytes;
	} *header, **tail;

	void   IndexFrames(void);
	static DWORD WINAPI ThreadProc(LPVOID arg);
	void   ReadStruct(void *buffer, int size_of_buffer, int size_in_header);
};

UtilCaptureFile::DiskIO::DiskIO(pxcCHAR *filename, pxcBool recording, pxcBool &sts) {
	length=0; header=0; tail=&header; pause=false; pos=pos_next=0;
	file=file2=thread=INVALID_HANDLE_VALUE;
	indexed.push_back(std::pair<pxcU64,pxcU64>(0,1));

	gowrite=CreateEvent(0,TRUE,FALSE,0);
	InitializeCriticalSection(&cs);

	if (filename) file=CreateFileW(filename,
		recording?GENERIC_WRITE:GENERIC_READ,
		recording?0:FILE_SHARE_READ,0,
		recording?CREATE_ALWAYS:OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL|FILE_FLAG_SEQUENTIAL_SCAN,
		0);
	sts=(file!=INVALID_HANDLE_VALUE);
	if (!recording && sts) {
		file2=CreateFileW(filename,GENERIC_READ,FILE_SHARE_READ,0,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,0);
		LARGE_INTEGER tmp;
		GetFileSizeEx(file2,&tmp);
		length=(pxcU64)tmp.QuadPart;
	}
	if (recording && sts)
		thread=CreateThread(0,0,ThreadProc,this,0,0);
}

UtilCaptureFile::DiskIO::~DiskIO(void) {
	if (thread!=INVALID_HANDLE_VALUE) {
        do { SetEvent(gowrite); } 
            while (WaitForSingleObject(thread,50)==WAIT_TIMEOUT);
		CloseHandle(thread);
	}
	CloseHandle(gowrite);
	DeleteCriticalSection(&cs);
	if (file!=INVALID_HANDLE_VALUE) CloseHandle(file);
	if (file2!=INVALID_HANDLE_VALUE) CloseHandle(file2);
}

void UtilCaptureFile::DiskIO::WriteHeader(void) {
    Header header;
    memset(&header, 0, sizeof(header));

    header.id=PXC_UID('P','X','C','F');
    header.fileVersion=3; // file format version
    header.captureCuid=(pxcU32)PXCCapture::CUID;
    header.sizeOfDeviceInfo=sizeof(PXCCapture::DeviceInfo);
    header.sizeOfStreamInfo=sizeof(PXCCapture::Device::StreamInfo);
    header.sizeOfVideoStreamProfileInfo=sizeof(PXCCapture::VideoStream::ProfileInfo);
    header.sizeOfAudioStreamProfileInfo=sizeof(PXCCapture::AudioStream::ProfileInfo);

	SetFilePointer(file,0,0,FILE_BEGIN);
	DWORD nbytesWrite;
	WriteFile(file,&header,sizeof(header),&nbytesWrite,0);
}

void UtilCaptureFile::DiskIO::WriteDeviceInfo(PXCCapture::DeviceInfo *dinfo) {
	DWORD nbytesWrite;
	WriteFile(file,dinfo,sizeof(*dinfo),&nbytesWrite,0); 
}

void UtilCaptureFile::DiskIO::WriteDeviceProperties(std::map<PXCCapture::Device::Property,pxcF32> &properties) {
	DWORD nbytesWrite;

    int nproperties=(int)properties.size();
	WriteFile(file,&nproperties,sizeof(int),&nbytesWrite,0);
    for (std::map<PXCCapture::Device::Property,pxcF32>::iterator itr=properties.begin();itr!=properties.end();itr++) {
		WriteFile(file,(void*)&itr->first,sizeof(PXCCapture::Device::Property),&nbytesWrite,0);
		WriteFile(file,&itr->second,sizeof(pxcF32),&nbytesWrite,0);
    }
}

void UtilCaptureFile::DiskIO::WriteVideoStreamInfo(std::vector<PXCCapture::Device::StreamInfo> &streams, std::vector<PXCCapture::VideoStream::ProfileInfo> &profiles) {
	DWORD nbytesWrite;

    int nstreams=(int)streams.size();
	WriteFile(file,&nstreams,sizeof(nstreams),&nbytesWrite,0);
	for (int i=0;i<nstreams;i++) {
		WriteFile(file,&streams[i],sizeof(streams[i]),&nbytesWrite,0);
		WriteFile(file,&profiles[i],sizeof(profiles[i]),&nbytesWrite,0);
	}
}

void UtilCaptureFile::DiskIO::WriteAudioStreamInfo(PXCCapture::Device::StreamInfo *sinfo, PXCCapture::AudioStream::ProfileInfo *pinfo) {
	DWORD nbytesWrite;

    int nastreams=sinfo?1:0;
	WriteFile(file,&nastreams,sizeof(nastreams),&nbytesWrite,0);
    if (nastreams<=0) return;
	WriteFile(file,sinfo,sizeof(*sinfo),&nbytesWrite,0);
	WriteFile(file,pinfo,sizeof(*pinfo),&nbytesWrite,0);
}

void UtilCaptureFile::DiskIO::WriteSerializables(pxcU32 nitems, pxcU32 nbytes, pxcBYTE *data) {
	DWORD nbytesWrite;
    WriteFile(file,&nitems,sizeof(nitems),&nbytesWrite,0);
    if (nitems) WriteFile(file,data,nbytes,&nbytesWrite,0);
}

void UtilCaptureFile::DiskIO::WriteFirstFrameOffset(void) {
    pxcU32 tmp=SetFilePointer(file,0,0,FILE_CURRENT);
    SetFilePointer(file,26*sizeof(pxcU32),0,FILE_BEGIN);
	DWORD nbytesWrite;
    WriteFile(file,&tmp,sizeof(tmp),&nbytesWrite,0);
	SetFilePointer(file,tmp,0,FILE_BEGIN);
}

class PXCImageAddRef: public PXCBase {
public:
	PXC_CUID_OVERWRITE(PXC_UID('I','M','G','S'));
	virtual pxcStatus PXCAPI XXXXX(void) = 0; // place holder
	virtual pxcU32    PXCAPI AddRef(void) = 0;
};

void UtilCaptureFile::DiskIO::WriteFrame(pxcU32 sidx, PXCImage *image) {
	Block *block=(Block*)new pxcBYTE[sizeof(Block)];
	if (!block) return;

	block->next=0;
	block->sidx=sidx;
	block->sample=image;
	image->DynamicCast<PXCImageAddRef>()->AddRef(); // preserve image

	EnterCriticalSection(&cs);
	*tail=block;
	tail=&(block->next);
	SetEvent(gowrite);
	LeaveCriticalSection(&cs);
}

void UtilCaptureFile::DiskIO::WriteFrame(pxcU32 sidx, PXCAudio* audio) {
    PXCAudio::AudioData data;
    pxcStatus sts=audio->AcquireAccess(PXCAudio::ACCESS_READ,&data);
    if (sts>=PXC_STATUS_NO_ERROR) {
		pxcU32 nbytes=(data.format&PXCAudio::AUDIO_FORMAT_SIZE_MASK)/8*data.dataSize;
		Block *block=(Block*)new pxcBYTE[sizeof(Block)+sizeof(pxcU64)+nbytes];
		if (block) {
			block->next=0;
			block->sidx=sidx;
			block->sample=0;
			block->nbytes=nbytes+sizeof(pxcU64);

			/* copy audio buffer */
			*((pxcU64*)(block+1))=audio->QueryTimeStamp();
			memcpy(((BYTE*)block)+sizeof(Block)+sizeof(pxcU64),data.dataPtr,nbytes);

			EnterCriticalSection(&cs);
			*tail=block;
			tail=&(block->next);
			SetEvent(gowrite);
			LeaveCriticalSection(&cs);
		}
        audio->ReleaseAccess(&data);
    }
}

DWORD UtilCaptureFile::DiskIO::ThreadProc(LPVOID arg) {
	UtilCaptureFile::DiskIO *pthis=(UtilCaptureFile::DiskIO*)arg;
	for (;;) {
		WaitForSingleObject(pthis->gowrite,INFINITE);

		EnterCriticalSection(&pthis->cs);
        Block *block=pthis->header;
        pthis->header=0; pthis->tail=&pthis->header;
		ResetEvent(pthis->gowrite);
		LeaveCriticalSection(&pthis->cs);
		if (!block) break;

        for (Block *tmp;block;block=tmp) {
		    DWORD nbytesWrite;
		    if (block->sample) {
			    PXCImage::ImageData data;
			    pxcStatus sts=block->sample->AcquireAccess(PXCImage::ACCESS_READ,&data);
			    if (sts>=PXC_STATUS_NO_ERROR) {
				    /* sample information */
				    PXCImage::ImageOption options=block->sample->QueryOption();
				    PXCImage::ImageInfo info;
				    block->sample->QueryInfo(&info);

				    /* write sidx */
				    WriteFile(pthis->file,&block->sidx,sizeof(pxcU32),&nbytesWrite,0);

				    /* write nbytes */
				    pxcU32 nbytes=sizeof(pxcU64)+data.pitches[0]*info.height;
				    switch (data.format&PXCImage::IMAGE_TYPE_MASK) {
				    case PXCImage::IMAGE_TYPE_DEPTH:
					    if (!(options&PXCImage::IMAGE_OPTION_NO_IR_MAP)) 
						    nbytes+=data.pitches[1]*info.height;
					    if (!(options&PXCImage::IMAGE_OPTION_NO_UV_MAP)) 
						    nbytes+=data.pitches[2]*info.height;
					    break;
				    }
				    WriteFile(pthis->file,&nbytes,sizeof(nbytes),&nbytesWrite,0);

				    /* write time stamp */
				    pxcU64 ts=block->sample->QueryTimeStamp();
				    WriteFile(pthis->file,&ts,sizeof(ts),&nbytesWrite,0);

				    /* write planes */
				    WriteFile(pthis->file,data.planes[0],data.pitches[0]*info.height,&nbytesWrite,0);
				    switch (data.format&PXCImage::IMAGE_TYPE_MASK) {
				    case PXCImage::IMAGE_TYPE_DEPTH:
					    if (!(options&PXCImage::IMAGE_OPTION_NO_IR_MAP)) 
						    WriteFile(pthis->file,data.planes[1],data.pitches[1]*info.height,&nbytesWrite,0);
					    if (!(options&PXCImage::IMAGE_OPTION_NO_UV_MAP)) 
						    WriteFile(pthis->file,data.planes[2],data.pitches[2]*info.height,&nbytesWrite,0);
					    break;
				    }
				    block->sample->ReleaseAccess(&data);
			    }
			    block->sample->Release();
		    } else {
			    WriteFile(pthis->file,&block->sidx,sizeof(pxcU32),&nbytesWrite,0);
			    WriteFile(pthis->file,&block->nbytes,sizeof(pxcU32),&nbytesWrite,0);
			    WriteFile(pthis->file,(BYTE*)(block+1),block->nbytes,&nbytesWrite,0);
		    }
            tmp=block->next;
            delete [] block;
        }
	}
	return 0;
}

void UtilCaptureFile::DiskIO::ReadStruct(void *buffer, int size_of_buffer, int size_in_header) {
    if (size_in_header<=0) size_in_header=size_of_buffer;
    memset(buffer,0,size_of_buffer);
	Read(buffer,size_of_buffer<size_in_header?size_of_buffer:size_in_header);
    if (size_of_buffer<size_in_header) SeekCUR(size_in_header-size_of_buffer);
}

void UtilCaptureFile::DiskIO::ReadDeviceProperties(std::map<PXCCapture::Device::Property,pxcF32> &properties) {
	properties.clear();
    int nproperties=0;
	Read(&nproperties,sizeof(nproperties));
    for (int i=0;i<nproperties;i++) {
        PXCCapture::Device::Property pty; pxcF32 value;
		Read(&pty,sizeof(pty));
		Read(&value,sizeof(value));
        properties[pty]=value;
    }
}

void UtilCaptureFile::DiskIO::ReadVideoStreamInfo(Header *header, std::vector<PXCCapture::Device::StreamInfo> &sinfos, std::vector<PXCCapture::VideoStream::ProfileInfo> &pinfos) {
    int nvstreams=0;
	Read(&nvstreams,sizeof(nvstreams));
    for (int i=0;i<nvstreams;i++) {
        PXCCapture::Device::StreamInfo sinfo;
        ReadStruct(&sinfo,sizeof(sinfo),header->sizeOfStreamInfo);
        sinfos.push_back(sinfo);
        PXCCapture::VideoStream::ProfileInfo pinfo;
        ReadStruct(&pinfo,sizeof(pinfo),header->sizeOfVideoStreamProfileInfo);
        pinfos.push_back(pinfo);
    }
}

void UtilCaptureFile::DiskIO::ReadAudioStreamInfo(Header *header, std::vector<PXCCapture::Device::StreamInfo> &sinfos, std::vector<PXCCapture::AudioStream::ProfileInfo> &pinfos) {
    int nastreams=0;
	Read(&nastreams,sizeof(nastreams));
    for (int j=0;j<nastreams;j++) {
        PXCCapture::Device::StreamInfo sinfo;
        ReadStruct(&sinfo,sizeof(sinfo),header->sizeOfStreamInfo);
        sinfos.push_back(sinfo);
        PXCCapture::AudioStream::ProfileInfo pinfo;
        ReadStruct(&pinfo,sizeof(pinfo),header->sizeOfAudioStreamProfileInfo);
        pinfos.push_back(pinfo);
    }
}

pxcU32 UtilCaptureFile::DiskIO::ReadSerializeables(Header *header, pxcBYTE *data, pxcU32 nbytesMax) {
	pxcU32 nitems=0;
    if (header->fileVersion>1 || header->firstFrameOffset>0) { // header->firstFrameOffset==0 for version 1
        Read(&nitems,sizeof(nitems));
		for (pxcU32 i=0;i<nitems;i++) {
			pxcU32 nbytes=0;
			Read(&nbytes,sizeof(nbytes));
			if (nbytes+sizeof(pxcU32)<=nbytesMax) {
				*((pxcU32*)data)=nbytes;
				nbytesMax-=Read(data+sizeof(pxcU32),nbytes)+sizeof(pxcU32);
				data+=sizeof(pxcU32)+nbytes;
			} else {
				SeekCUR(nbytes);
				nbytesMax=0;
			}
		}
	}
	return nitems;
}

void UtilCaptureFile::DiskIO::SeekFirstFrame(Header *header) {
	pxcU64 tmp=SeekCUR(0);
    SeekSET((pxcU64)header->firstFrameOffset>tmp?(pxcU64)header->firstFrameOffset:tmp);
	SetFrameNo(0);
}

DWORD UtilCaptureFile::DiskIO::Read(void *buffer, DWORD nbytes) {
	if (file==INVALID_HANDLE_VALUE) return 0;

	LARGE_INTEGER tmp;
	tmp.QuadPart=pos;
	SetFilePointerEx(file,tmp,&tmp,FILE_BEGIN);

	if (pos+nbytes>length) nbytes=(DWORD)(length-pos);
	DWORD nbytesRead=0;
	ReadFile(file,buffer,nbytes,&nbytesRead,0);

	pos+=nbytesRead;
	return nbytesRead;
}

DWORD UtilCaptureFile::DiskIO::Read(PXCImage *sample, DWORD nbytes) {
    PXCImage::ImageData data;
    pxcStatus sts=sample->AcquireAccess(PXCImage::ACCESS_WRITE,&data);
    if (sts>=PXC_STATUS_NO_ERROR) {
        pxcU64  timeStamp=0;
		Read(&timeStamp,sizeof(timeStamp));
        sample->SetTimeStamp(timeStamp);

		PXCImage::ImageInfo info;
		sample->QueryInfo(&info);

		PXCImage::ImageOption options;
		options=sample->QueryOption();

        switch (info.format&PXCImage::IMAGE_TYPE_MASK) {
        case PXCImage::IMAGE_TYPE_COLOR:
			Read(data.planes[0],data.pitches[0]*info.height);
            break;
        case PXCImage::IMAGE_TYPE_DEPTH:
			Read(data.planes[0],data.pitches[0]*info.height);
            if (!(options&PXCImage::IMAGE_OPTION_NO_IR_MAP) && data.planes[1]) 
				Read(data.planes[1],data.pitches[1]*info.height);
            if (!(options&PXCImage::IMAGE_OPTION_NO_UV_MAP) && data.planes[2])
				Read(data.planes[2],data.pitches[2]*info.height);
            break;
        }
        sample->ReleaseAccess(&data);
	} else {
		SeekCUR(nbytes);
	}
	return nbytes;
}

DWORD UtilCaptureFile::DiskIO::Read(PXCAudio *sample, DWORD nbytes) {
    PXCAudio::AudioData data;
    pxcStatus sts=sample->AcquireAccess(PXCAudio::ACCESS_WRITE,PXCAudio::AUDIO_FORMAT_IEEE_FLOAT,&data);
    if (sts>=PXC_STATUS_NO_ERROR) {
	    PXCAudio::AudioInfo info;
		sts=sample->QueryInfo(&info);

        pxcU64 timeStamp=0;
		Read(&timeStamp,sizeof(timeStamp));
        sample->SetTimeStamp(timeStamp);

        data.dataSize=(nbytes-sizeof(timeStamp))/((data.format&PXCAudio::AUDIO_FORMAT_SIZE_MASK)/8);
		Read(data.dataPtr,((data.format&PXCAudio::AUDIO_FORMAT_SIZE_MASK)/8)*data.dataSize);
        sample->ReleaseAccess(&data);
    } else {
		SeekCUR(nbytes);
    }
	return nbytes;
}

void UtilCaptureFile::DiskIO::IndexFrames(void) {
	if (indexed[0].second>1 || !file2) return;
	pxcU64 pos=indexed[0].first, timeStamp=0;
	indexed.clear();
	for (DWORD nbytes=0, nbytesRead=0, sidx=0;;pos+=nbytes+sizeof(pxcU32)*2) {
		LARGE_INTEGER tmp;
		tmp.QuadPart=pos;
		SetFilePointerEx(file2,tmp,&tmp,FILE_BEGIN);
		ReadFile(file2,&sidx,sizeof(sidx),&nbytesRead,0);
		ReadFile(file2,&nbytes,sizeof(nbytes),&nbytesRead,0);
		if (!ReadFile(file2,&timeStamp,sizeof(timeStamp),&nbytesRead,0)) break;
		if (nbytesRead<sizeof(timeStamp)) break;
		if (selected.find(sidx)!=selected.end()) indexed.push_back(std::pair<pxcU64,pxcU64>(pos,timeStamp));
	}
}

void UtilCaptureFile::DiskIO::SetFrameNo(int fn) {
	if (fn==0 && !indexed[0].first) indexed[0]=std::pair<pxcU64,pxcU64>(SeekCUR(0),1);
	if (fn>0) IndexFrames();
	pos_next=(fn<(int)indexed.size())?indexed[fn].first:length;
}

void UtilCaptureFile::DiskIO::SetFrameTimeStamp(pxcU64 ts) {
	int i;
	IndexFrames();
	if (ts>0) ts+=indexed[0].second;
	for (i=(int)(indexed.size()-1);i>=0;i--)
		if (ts>indexed[i].second) break;
	pos_next=(i+1<(int)(indexed.size()))?indexed[i+1].first:length;
}

int UtilCaptureFile::DiskIO::GetFrameNo(void) {
	pxcU64 pos=(pos_next>0)?pos_next:SeekCUR(0);
	IndexFrames();
	for (pxcU32 i=0;i<indexed.size()-1;i++)
		if (pos>=indexed[i].first && pos<indexed[i+1].first) return i;
	if (pos<indexed[0].first) return 0; // pos_next may not be correct before IndexFrames.
	if (pos>=indexed[indexed.size()-1].first && pos<length)
		return (int)(indexed.size()-1);
	return -1;
}

pxcU64 UtilCaptureFile::DiskIO::GetFrameTimeStamp(void) {
	int no=GetFrameNo();
	return (no>=0)?indexed[no].second:0;
}

void UtilCaptureFile::DiskIO::NextFrame(void) {
	if (pos_next>0) {
		SeekSET(pos_next);
		if (!pause) pos_next=0;
	}
	if (pause && !pos_next) pos_next=SeekCUR(0);
}

void UtilCaptureFile::DiskIO::SetPause(pxcBool pause) {
	this->pause=pause;
	if (!pause) pos_next=0;
}

#ifndef UTIL_CAPTURE_FILE_DISKIO_ONLY

class CaptureRecording: public PXCBaseImpl<PXCCapture> {
public:
    class VideoStreamRecording;
    class AudioStreamRecording;
    class DeviceRecording: public PXCBaseImpl<PXCCapture::Device> {
        friend class VideoStreamRecording;
        friend class AudioStreamRecording;
    public:
        DeviceRecording(PXCCapture::Device *d, PXCScheduler *s, PXCImage::ImageType t);
        virtual ~DeviceRecording(void);
        virtual pxcStatus PXCAPI QueryDevice(DeviceInfo *dinfo) { return device->QueryDevice(dinfo); }
        virtual pxcStatus PXCAPI QueryStream(pxcU32 sidx, StreamInfo *sinfo) { return device->QueryStream(sidx,sinfo); }
        virtual pxcStatus PXCAPI CreateStream(pxcU32 sidx, pxcUID cuid, void **stream);
        virtual pxcStatus PXCAPI QueryPropertyInfo(Property label, PXCRangeF32 *range, pxcF32 *step, pxcF32 *def, pxcBool *isAuto) { return device->QueryPropertyInfo(label,range,step,def,isAuto); }
        virtual pxcStatus PXCAPI QueryProperty(Property label, pxcF32 *value);
        virtual pxcStatus PXCAPI SetPropertyAuto(Property pty, pxcBool ifauto) { return device->SetPropertyAuto(pty,ifauto); }
        virtual pxcStatus PXCAPI SetProperty(Property pty, pxcF32 value) { return device->SetProperty(pty,value); }
        virtual void SaveDeviceInfo(UtilCaptureFile::DiskIO *file);
    protected:
        PXCSmartPtr<PXCCapture::Device> device;
        std::map<Property,pxcF32>       properties;
        DeviceInfo                      dinfo;
        UtilCaptureFile::DiskIO       *file;
        PXCScheduler                    *scheduler;
        PXCImage::ImageType             types;
        CRITICAL_SECTION                cs;
    };

    class VideoStreamRecording: public PXCBaseImpl<PXCCapture::VideoStream> {
		friend class UtilCaptureFile;
    public:
        VideoStreamRecording(DeviceRecording *d, PXCCapture::VideoStream *s) { stream=s; device=d; QueryStream(&sinfo); }
        virtual pxcStatus PXCAPI QueryStream(Device::StreamInfo *sinfo) { return stream->QueryStream(sinfo); }
        virtual pxcStatus PXCAPI QueryProfile(pxcU32 pidx, ProfileInfo *pinfo) { return stream->QueryProfile(pidx,pinfo); }
        virtual pxcStatus PXCAPI SetProfile(ProfileInfo *pinfo) { this->pinfo=(*pinfo); return stream->SetProfile(pinfo); }
        virtual pxcStatus PXCAPI ReadStreamAsync(PXCImage **image, PXCScheduler::SyncPoint **sp);
		virtual pxcStatus PXCAPI PassOnStatus(pxcStatus sts) { return sts; }
        virtual pxcStatus PXCAPI SaveImage(PXCImage *image);
    protected:
        PXCSmartPtr<PXCCapture::VideoStream> stream;
        Device::StreamInfo      sinfo;
        ProfileInfo             pinfo;
        DeviceRecording         *device;
    };

    class AudioStreamRecording: public PXCBaseImpl<PXCCapture::AudioStream> {
		friend class UtilCaptureFile;
    public:
        AudioStreamRecording(DeviceRecording *d, PXCCapture::AudioStream *s) { stream=s; device=d; QueryStream(&sinfo); }
        virtual pxcStatus PXCAPI QueryStream(Device::StreamInfo *sinfo) { return stream->QueryStream(sinfo); }
        virtual pxcStatus PXCAPI QueryProfile(pxcU32 pidx, ProfileInfo *pinfo) { return stream->QueryProfile(pidx,pinfo); }
        virtual pxcStatus PXCAPI SetProfile(ProfileInfo *pinfo) { this->pinfo=(*pinfo); return stream->SetProfile(pinfo); }
        virtual pxcStatus PXCAPI ReadStreamAsync(PXCAudio **audio, PXCScheduler::SyncPoint **sp);
		virtual pxcStatus PXCAPI PassOnStatus(pxcStatus sts) { return sts; }
        virtual pxcStatus PXCAPI SaveAudio(PXCAudio *audio);
    protected:
        PXCSmartPtr<PXCCapture::AudioStream> stream;
        Device::StreamInfo      sinfo;
        ProfileInfo             pinfo;
        DeviceRecording         *device;
    };
    CaptureRecording(PXCCapture *c, PXCScheduler *s, PXCImage::ImageType t) { capture=c; scheduler=s; types=t; }
    virtual pxcStatus PXCAPI QueryDevice(pxcU32 didx, DeviceInfo *dinfo) { return capture->QueryDevice(didx,dinfo); }
    virtual pxcStatus PXCAPI CreateDevice(pxcU32 didx, Device **device);
protected:
    PXCSmartPtr<PXCCapture> capture;
    PXCScheduler            *scheduler;
    PXCImage::ImageType     types;
};

class RealtimeSync {
public:
    RealtimeSync(void);
    void Wait(pxcU64 ts);
protected:
    pxcU64 t2;
    LARGE_INTEGER freq, t1;
};

RealtimeSync::RealtimeSync() {
	QueryPerformanceFrequency(&freq);
    t2=(pxcU64)-1;
}

void RealtimeSync::Wait(pxcU64 ts) {
	if (ts>t2) {
		LONGLONG delta=(LONGLONG)((double)(ts-t2)/10000000*freq.QuadPart);
		if (delta>freq.QuadPart) delta=freq.QuadPart;
		for (LONGLONG t3=t1.QuadPart+delta;t1.QuadPart<t3;Sleep(0))
			QueryPerformanceCounter(&t1);
	} else {  // reverted? reset
		QueryPerformanceCounter(&t1);
	}
	t2=ts;
}

class CapturePlayback: public UtilCaptureImpl {
public:
    class VideoStreamPlayback;
    class AudioStreamPlayback;
    class DevicePlayback: public UtilCaptureImpl::DeviceImpl {
        friend class VideoStreamPlayback;
        friend class AudioStreamPlayback;
    public:
        enum { PLAYBACK_REALTIME=PROPERTY_CUSTOMIZED };
        DevicePlayback(CapturePlayback *capture, int didx, UtilCaptureFile::DiskIO *file);
        virtual pxcStatus PXCAPI CreateStream(pxcU32 sidx, pxcUID cuid, void **stream);
        virtual pxcStatus PXCAPI QueryProperty(Property label, pxcF32 *value);
        virtual pxcStatus PXCAPI SetProperty(Property pty, pxcF32 value);
    protected:
        std::map<Property,pxcF32>               properties;
        std::map<pxcU32,pxcU32>                 smap;
        std::vector<VideoStream::ProfileInfo>   vprofiles;
        std::vector<AudioStream::ProfileInfo>   aprofiles;
        UtilCaptureFile::DiskIO                *file;
        int                                     fileVersion;
        pxcBool									realtime;
        RealtimeSync							realtimeSync;

        virtual void UpdateSample(std::deque<pxcU32> &updates);
        virtual pxcStatus ProcessSample(pxcU32 sidx, PXCBase *storage);
    };

    class VideoStreamPlayback: public UtilCaptureImpl::VideoStreamImpl {
    public:
        VideoStreamPlayback(DevicePlayback *device, int sidx);
    };

    class AudioStreamPlayback: public UtilCaptureImpl::AudioStreamImpl {
    public:
        AudioStreamPlayback(DevicePlayback *device, int sidx);
    };
    CapturePlayback(PXCSession *session, PXCScheduler *sch,UtilCaptureFile::DiskIO *file, PXCImage::ImageType types);
    virtual pxcStatus PXCAPI CreateDevice(pxcU32 didx,PXCCapture::Device **instance);
protected:
    UtilCaptureFile::DiskIO *file;
	PXCImage::ImageType types;
};

class WaveFilePlayback: public UtilCaptureImpl {
public:
    class AudioStreamPlayback;
    class DevicePlayback: public UtilCaptureImpl::DeviceImpl {
        friend class AudioStreamPlayback;
    public:
        enum { PLAYBACK_REALTIME=PROPERTY_CUSTOMIZED };
        DevicePlayback(WaveFilePlayback *capture, int didx, UtilCaptureFile::DiskIO *file);
		virtual ~DevicePlayback(void) { CloseHandle(eRead); }
        virtual pxcStatus PXCAPI CreateStream(pxcU32 sidx, pxcUID cuid, void **stream);
        virtual pxcStatus PXCAPI SetProperty(Property pty, pxcF32 value);
    protected:
		virtual void UpdateSample(std::deque<pxcU32> &updates);        
        virtual pxcStatus ProcessSample(pxcU32 sidx, PXCBase *storage);
        virtual void PushContext(pxcU32 sidx, SampleContext &context);
		virtual void PopContext(pxcU32 sidx, SampleContext &context);
        virtual void StopThread(void);
        UtilCaptureFile::DiskIO *file;
		pxcU64	        timeStamp;
        pxcBool         realtime;
        RealtimeSync    realtimeSync;
        HANDLE          eRead;
    };

    class AudioStreamPlayback: public UtilCaptureImpl::AudioStreamImpl {
    public:
        AudioStreamPlayback(DevicePlayback *device, int sidx);
    };

    WaveFilePlayback(PXCSession *session, PXCScheduler *sch,UtilCaptureFile::DiskIO *file, pxcCHAR *filename);
    virtual pxcStatus PXCAPI CreateDevice(pxcU32 didx,PXCCapture::Device **instance);
protected:
    UtilCaptureFile::DiskIO *file;
};

CaptureRecording::DeviceRecording::DeviceRecording(PXCCapture::Device *d, PXCScheduler *s, PXCImage::ImageType t) { 
    device=d; scheduler=s; types=t; file=0;
    QueryDevice(&dinfo);
    InitializeCriticalSection(&cs);
}

CaptureRecording::DeviceRecording::~DeviceRecording(void) {
    DeleteCriticalSection(&cs);
}

pxcStatus CaptureRecording::DeviceRecording::QueryProperty(Property label, pxcF32 *value) {
    pxcStatus sts=device->QueryProperty(label,value);
    if (sts>=PXC_STATUS_NO_ERROR) properties[label]=(*value);
    return sts;
}

void CaptureRecording::DeviceRecording::SaveDeviceInfo(UtilCaptureFile::DiskIO *file) {
    static int knowns[]={  /* from */							/* to */
					PROPERTY_COLOR_EXPOSURE,            PROPERTY_COLOR_GAIN,
					PROPERTY_AUDIO_MIX_LEVEL,           PROPERTY_AUDIO_MIX_LEVEL,
					PROPERTY_DEPTH_SATURATION_VALUE,    PROPERTY_DEPTH_UNIT,
					PROPERTY_CAMERA_MODEL,              PROPERTY_CAMERA_MODEL,
					PROPERTY_COLOR_FIELD_OF_VIEW,       PROPERTY_COLOR_PRINCIPAL_POINT+1,
					PROPERTY_DEPTH_FIELD_OF_VIEW,       PROPERTY_DEPTH_PRINCIPAL_POINT+1,
					PROPERTY_ACCELEROMETER_READING,     PROPERTY_ACCELEROMETER_READING+2,
    };

    this->file=file;
	file->WriteHeader();
	file->WriteDeviceInfo(&dinfo);

    pxcF32 value; 	/* Scan common properties */
    for (int i=0;i<sizeof(knowns)/sizeof(knowns[0]);i+=2)
        for (int j=knowns[i];j<=knowns[i+1];j++)
            QueryProperty((PXCCapture::Device::Property)j,&value);
	file->WriteDeviceProperties(properties);
}

pxcStatus CaptureRecording::DeviceRecording::CreateStream(pxcU32 sidx, pxcUID cuid, void **stream) {
    PXCBase *stream2=0;
    pxcStatus sts=device->CreateStream(sidx,cuid,(void**)&stream2);
    if (sts>=PXC_STATUS_NO_ERROR) {
        switch (cuid) {
        case PXCCapture::VideoStream::CUID:
            *stream=new CaptureRecording::VideoStreamRecording(this,(PXCCapture::VideoStream*)stream2->DynamicCast(cuid));
            break;
        case PXCCapture::AudioStream::CUID:
            *stream=new CaptureRecording::AudioStreamRecording(this,(PXCCapture::AudioStream*)stream2->DynamicCast(cuid));
            break;
        }
    }
    return sts;
}

pxcStatus CaptureRecording::VideoStreamRecording::ReadStreamAsync(PXCImage **image, PXCScheduler::SyncPoint **sp) {
    PXCSmartSP sp2;
    pxcStatus sts=stream->ReadStreamAsync(image,&sp2);
	if (sts==PXC_STATUS_DEVICE_LOST) return sts;
    if (sts>=PXC_STATUS_NO_ERROR && device->file && (*image)) {
        PXCImage::ImageInfo info;
        (*image)->QueryInfo(&info);
        if (info.format&PXCImage::IMAGE_TYPE_MASK&device->types)
            return PXCSmartAsyncImplI1<CaptureRecording::VideoStreamRecording,PXCImage>::SubmitTask(*image,sp,this,device->scheduler,&VideoStreamRecording::SaveImage,&VideoStreamRecording::PassOnStatus,L"VideoStreamRecording::SaveImage");
    }
    *sp=sp2.ReleasePtr();
    return sts;
}

pxcStatus CaptureRecording::VideoStreamRecording::SaveImage(PXCImage *image) {
    EnterCriticalSection(&device->cs);
	device->file->WriteFrame(sinfo.sidx,image);
	LeaveCriticalSection(&device->cs);
    return PXC_STATUS_NO_ERROR;
}

pxcStatus CaptureRecording::AudioStreamRecording::ReadStreamAsync(PXCAudio **audio, PXCScheduler::SyncPoint **sp) {
    PXCSmartSP sp2;
    pxcStatus sts=stream->ReadStreamAsync(audio,&sp2);
	if (sts==PXC_STATUS_DEVICE_LOST) return sts;
    if (sts>=PXC_STATUS_NO_ERROR && device->file)
        return PXCSmartAsyncImplI1<CaptureRecording::AudioStreamRecording,PXCAudio>::SubmitTask(*audio,sp,this,device->scheduler,&AudioStreamRecording::SaveAudio,&AudioStreamRecording::PassOnStatus,L"AudioStreamRecording::SaveAudio");
    *sp=sp2.ReleasePtr();
    return sts;
}

pxcStatus CaptureRecording::AudioStreamRecording::SaveAudio(PXCAudio *audio) {
    EnterCriticalSection(&device->cs);
	device->file->WriteFrame(sinfo.sidx,audio);
    LeaveCriticalSection(&device->cs);
    return PXC_STATUS_NO_ERROR;
}

pxcStatus CaptureRecording::CreateDevice(pxcU32 didx, Device **device) {
    PXCCapture::Device *device2=0;
    pxcStatus sts=capture->CreateDevice(didx,&device2);
    if (sts>=PXC_STATUS_NO_ERROR) {
        *device=new CaptureRecording::DeviceRecording(device2,scheduler,types);
        if (!(*device)) sts=PXC_STATUS_ALLOC_FAILED;
    }
    return sts;
}

CapturePlayback::DevicePlayback::DevicePlayback(CapturePlayback *capture, int didx, UtilCaptureFile::DiskIO *file):DeviceImpl(capture,didx) {
    this->file=file; realtime=TRUE;

	/* Read file header */
    UtilCaptureFile::DiskIO::Header header;
	file->ReadHeader(&header);
    fileVersion=header.fileVersion;

    PXCCapture::DeviceInfo dinfo;
    file->ReadDeviceInfo(&header,&dinfo);
	file->ReadDeviceProperties(properties);

	/* Read video streams. Remap the stream index */
	file->ReadVideoStreamInfo(&header,streams,vprofiles);
	int i;
	for (i=0;i<(int)streams.size();i++) {
		smap[streams[i].sidx]=i;
		if (streams[i].imageType&capture->types) file->SelectStream(streams[i].sidx);
        streams[i].sidx=i;
    }

	/* Read audio streams. Remap the stream index */
	file->ReadAudioStreamInfo(&header,streams,aprofiles);
	for (int j=i;j<(int)streams.size();j++) {
		smap[streams[j].sidx]=j;
        streams[j].sidx=j;
    }

	/* Read serializables */
	pxcBYTE sdata[256*sizeof(pxcF32)];
	pxcU32 nitems=file->ReadSerializeables(&header,sdata,sizeof(sdata));
	if (nitems>0) { // Assume the first serializable is the projection interface
		PXCMetadata *md=capture->session->DynamicCast<PXCMetadata>();
        pxcUID uid=md->QueryUID();
        md->AttachBuffer(uid,sdata+sizeof(pxcU32),((pxcU32*)sdata)[0]);
        ((pxcUID*)&(properties[PROPERTY_PROJECTION_SERIALIZABLE]))[0]= uid;
    }

	/* Seek to first frame position */
	file->SeekFirstFrame(&header);
}

pxcStatus CapturePlayback::DevicePlayback::CreateStream(pxcU32 sidx, pxcUID, void **stream) {
    *stream=0;
    if (sidx<vprofiles.size()) {
        *stream=new CapturePlayback::VideoStreamPlayback(this,sidx);
    } else if (sidx<vprofiles.size()+aprofiles.size()) {
        *stream=new CapturePlayback::AudioStreamPlayback(this,sidx);
    }
    return (*stream)?PXC_STATUS_NO_ERROR:PXC_STATUS_ITEM_UNAVAILABLE;
}

pxcStatus CapturePlayback::DevicePlayback::QueryProperty(Property label, pxcF32 *value) {
    std::map<Property,pxcF32>::iterator itr=properties.find(label);
    if (itr==properties.end()) {
        if ((fileVersion<=2) && (label&PROPERTY_CUSTOMIZED)) return PXC_STATUS_NO_ERROR; 
        return PXC_STATUS_ITEM_UNAVAILABLE;
    }
    *value=itr->second;
    return PXC_STATUS_NO_ERROR;
}

void CapturePlayback::DevicePlayback::UpdateSample(std::deque<pxcU32> &updates) {
	file->NextFrame();
	pxcU64 pos=file->SeekCUR(0);
	for (;;) {
        pxcU32 sidx=0;
        if (file->Read(&sidx,sizeof(sidx))<sizeof(sidx)) break; // EOF

		/* map the disk stream index to the device stream list */
        std::map<pxcU32,pxcU32>::iterator itr=smap.find(sidx);
        if (itr==smap.end()) break; // EOF: The stream not in the header stream list?
        sidx=itr->second;

		switch (streams[sidx].cuid) {
		case PXCCapture::VideoStream::CUID:
			if (!profiles[sidx].video.imageInfo.format) break;
		    updates.push_back(sidx); // This stream is selected
			file->SeekSET(pos);
			return;
		case PXCCapture::AudioStream::CUID:
			if (!profiles[sidx].audio.audioInfo.format) break;
			updates.push_back(sidx); // This stream is selected
			file->SeekSET(pos);
			return;
		}

		// skip unused streams
		int nbytes=0;
        if (file->Read(&nbytes,sizeof(nbytes))<sizeof(nbytes)) break; // EOF
		pos=file->SeekCUR(nbytes);
    }

	// Updates for EOF signalling
	file->SeekSET(pos);
    for (std::map<pxcU32,pxcU32>::iterator itr=smap.begin();itr!=smap.end();itr++)
		updates.push_back(itr->second);
}

pxcStatus CapturePlayback::DevicePlayback::ProcessSample(pxcU32 sidx, PXCBase *storage) {
    if (!storage) return PXC_STATUS_NO_ERROR; // the application did not submit a request fast enough

	pxcU32  sidx2=0;
	if (file->Read(&sidx2,sizeof(sidx2))<sizeof(sidx2)) return PXC_STATUS_ITEM_UNAVAILABLE; // EOF
    std::map<pxcU32,pxcU32>::iterator itr=smap.find(sidx2);
    if (itr==smap.end()) return PXC_STATUS_ITEM_UNAVAILABLE; // EOF
    if (sidx!=itr->second) return PXC_STATUS_ITEM_UNAVAILABLE; // EOF

    pxcU32	nbytes=0;
    if (file->Read(&nbytes,sizeof(nbytes))<sizeof(nbytes)) return PXC_STATUS_ITEM_UNAVAILABLE; // EOF

    if (streams[sidx].cuid==PXCCapture::VideoStream::CUID) {
        PXCImage *image=storage->DynamicCast<PXCImage>();
		file->Read(image, nbytes);
        if (realtime) realtimeSync.Wait(image->QueryTimeStamp());
    } else {
		file->Read(storage->DynamicCast<PXCAudio>(), nbytes);
    }

    return PXC_STATUS_NO_ERROR;
}

CapturePlayback::VideoStreamPlayback::VideoStreamPlayback(DevicePlayback *device, int sidx):VideoStreamImpl(device,sidx) {
    profiles.push_back(device->vprofiles[sidx]);
}

CapturePlayback::AudioStreamPlayback::AudioStreamPlayback(DevicePlayback *device, int sidx):AudioStreamImpl(device,sidx) {
    profiles.push_back(device->aprofiles[sidx-device->vprofiles.size()]);
}

CapturePlayback::CapturePlayback(PXCSession *session, PXCScheduler *sch,UtilCaptureFile::DiskIO *file, PXCImage::ImageType types):UtilCaptureImpl(session,sch) {
	this->types=types; this->file=file;

	UtilCaptureFile::DiskIO::Header header={0};
	file->ReadHeader(&header);
	if (header.id==PXC_UID('P','X','C','F') && header.captureCuid==PXCCapture::CUID) {
		PXCCapture::DeviceInfo dinfo;
		file->ReadDeviceInfo(&header,&dinfo);
		dinfo.didx=0;
		devices.push_back(dinfo);
	}
}

pxcStatus CapturePlayback::CreateDevice(pxcU32 didx,PXCCapture::Device **instance) {
    if (didx>=devices.size()) return PXC_STATUS_ITEM_UNAVAILABLE;
    *instance=new CapturePlayback::DevicePlayback(this,didx,file);
    return (*instance)?PXC_STATUS_NO_ERROR:PXC_STATUS_ALLOC_FAILED;
}

pxcStatus CapturePlayback::DevicePlayback::SetProperty(Property pty, pxcF32 value) {
    if ((int)pty==PLAYBACK_REALTIME) realtime=(pxcBool)value;
    return DeviceImpl::SetProperty(pty,value);
}

WaveFilePlayback::DevicePlayback::DevicePlayback(WaveFilePlayback *capture, int didx, UtilCaptureFile::DiskIO *file):DeviceImpl(capture,didx) {
	this->file=file;
    eRead=CreateEvent(0,TRUE,FALSE,0);
	Device::StreamInfo sinfo;
	memset(&sinfo,0,sizeof(sinfo));
	sinfo.sidx=0;
	sinfo.cuid=PXCCapture::AudioStream::CUID;
	streams.push_back(sinfo);
	timeStamp=0;
    realtime=0;
}

pxcStatus WaveFilePlayback::DevicePlayback::CreateStream(pxcU32 sidx, pxcUID, void **stream) {
    *stream=(sidx<streams.size())?new WaveFilePlayback::AudioStreamPlayback(this,sidx):0;
    return (*stream)?PXC_STATUS_NO_ERROR:PXC_STATUS_ITEM_UNAVAILABLE;
}

void WaveFilePlayback::DevicePlayback::PushContext(pxcU32 sidx, SampleContext &context) {
    DeviceImpl::PushContext(sidx,context);
    SetEvent(eRead);
}

void WaveFilePlayback::DevicePlayback::PopContext(pxcU32 sidx, SampleContext &context) {
    DeviceImpl::PopContext(sidx,context);
    EnterCriticalSection(&cs);
    if (!queues[sidx].size()) ResetEvent(eRead);
    LeaveCriticalSection(&cs);
}

void WaveFilePlayback::DevicePlayback::StopThread(void) {
    SetEvent(eRead);
    DeviceImpl::StopThread();
}

void WaveFilePlayback::DevicePlayback::UpdateSample(std::deque<pxcU32> &updates) {
    WaitForSingleObject(eRead,500);
    updates.push_back(0);
}

pxcStatus WaveFilePlayback::DevicePlayback::ProcessSample(pxcU32, PXCBase *storage) {
    if (!storage) return PXC_STATUS_NO_ERROR; // the application did not submit a request fast enough
    PXCAudio *audio=storage->DynamicCast<PXCAudio>();
    PXCAudio::AudioInfo info;
    audio->QueryInfo(&info);

    PXCAudio::AudioData data;
    pxcStatus sts=audio->AcquireAccess(PXCAudio::ACCESS_WRITE,PXCAudio::AUDIO_FORMAT_PCM,&data);
    if (sts>=PXC_STATUS_NO_ERROR) {
		sts=PXC_STATUS_ITEM_UNAVAILABLE;
		DWORD sampleSize=((PXCAudio::AUDIO_FORMAT_PCM&PXCAudio::AUDIO_FORMAT_SIZE_MASK)/8);
		DWORD nbytesRead=(int)file->Read(data.dataPtr,sampleSize*info.bufferSize);
		if (nbytesRead>=sampleSize) {
			data.dataSize=(pxcU32)nbytesRead/sampleSize;
			timeStamp+=(pxcU64)10000000*(pxcU64)data.dataSize/(pxcU64)info.sampleRate/info.nchannels;
			//Sleep((DWORD)1000*(DWORD)data.dataSize/(DWORD)info.sampleRate);
			audio->SetTimeStamp(timeStamp);
			sts=PXC_STATUS_NO_ERROR;
		}
		audio->ReleaseAccess(&data);
    }

    if (realtime) realtimeSync.Wait(timeStamp);
    return sts;
}

WaveFilePlayback::AudioStreamPlayback::AudioStreamPlayback(DevicePlayback *device, int sidx):AudioStreamImpl(device,sidx) {
	device->file->SeekSET(12);
	ProfileInfo pinfo;
	memset(&pinfo,0,sizeof(pinfo));
	for (pxcU64 tmp;;) {
		struct RiffChunkHeader {
			int chunkId;
			int	chunkDataSize;
		} chunkHeader;
		struct WaveFormatChunk {
			short	compressionCode;
			short	numberofChannels;
			int		sampleRate;
			int		bytesPerSample;
			short	blockAlign;
			short	significantBitsPerSample;
		} waveFormatChunk;

		if (device->file->Read(&chunkHeader,sizeof(chunkHeader))<sizeof(chunkHeader)) break;
		tmp=device->file->SeekCUR(0);
		switch (chunkHeader.chunkId) {
		case PXC_UID('d','a','t','a'):
		    if (pinfo.audioInfo.bufferSize>0) profiles.push_back(pinfo);
			return;
		case PXC_UID('f','m','t',' '):
			if (device->file->Read(&waveFormatChunk,sizeof(waveFormatChunk))<sizeof(waveFormatChunk)) break;
			if (waveFormatChunk.compressionCode!=1) break; // skip if the format is compressed.
			pinfo.audioInfo.format=PXCAudio::AUDIO_FORMAT_PCM;
			pinfo.audioInfo.nchannels=waveFormatChunk.numberofChannels;
			pinfo.audioInfo.sampleRate=waveFormatChunk.sampleRate;
			pinfo.audioInfo.channelMask=(pinfo.audioInfo.nchannels==1)?PXCAudio::CHANNEL_MASK_FRONT_CENTER:(pinfo.audioInfo.nchannels==2)?PXCAudio::CHANNEL_MASK_FRONT_LEFT|PXCAudio::CHANNEL_MASK_FRONT_RIGHT:0;
			pinfo.audioInfo.bufferSize=(pxcU32)((pinfo.audioInfo.sampleRate/8)*pinfo.audioInfo.nchannels); // 1/8 second buffer
			break;
		}
		device->file->SeekSET(tmp+chunkHeader.chunkDataSize);
	}
}

WaveFilePlayback::WaveFilePlayback(PXCSession *session, PXCScheduler *sch, UtilCaptureFile::DiskIO *file,pxcCHAR *filename):UtilCaptureImpl(session,sch) {
    this->file=file;
    SetFilePointer(file,0,0,FILE_BEGIN);
	struct WaveFileHeader {
		int chunkId;
		int	chunkDataSize;
		int	riffType;
	} header;
	if (file->Read(&header,sizeof(header))>=sizeof(header)) {
		if (header.chunkId==PXC_UID('R','I','F','F') && header.riffType==PXC_UID('W','A','V','E')) {
			PXCCapture::DeviceInfo dinfo;
			memset(&dinfo,0,sizeof(dinfo));
			wcscpy(dinfo.name,filename);
			wcscpy(dinfo.did,filename);
			//wcscpy_s<sizeof(dinfo.name)/sizeof(pxcCHAR)>(dinfo.name,filename);
			//wcscpy_s<sizeof(dinfo.did)/sizeof(pxcCHAR)>(dinfo.did,filename);
			devices.push_back(dinfo);
		}
	}
}

pxcStatus WaveFilePlayback::CreateDevice(pxcU32 didx,PXCCapture::Device **instance) {
    if (didx>=devices.size()) return PXC_STATUS_ITEM_UNAVAILABLE;
    *instance=new WaveFilePlayback::DevicePlayback(this,didx,file);
    return (*instance)?PXC_STATUS_NO_ERROR:PXC_STATUS_ALLOC_FAILED;
}

pxcStatus WaveFilePlayback::DevicePlayback::SetProperty(Property pty, pxcF32 value) {
    if ((int)pty==PLAYBACK_REALTIME) realtime=(pxcBool)value;
    return DeviceImpl::SetProperty(pty,value);
}

void UtilCaptureFile::SaveStreamInfo(void) {
	CaptureRecording::DeviceRecording *device=m_device->DynamicCast<CaptureRecording::DeviceRecording>();
	device->SaveDeviceInfo(file);

    /* Save Video Stream Info */
	std::vector<PXCCapture::Device::StreamInfo> sinfos;
	std::vector<PXCCapture::VideoStream::ProfileInfo> pinfos;
    for (int i=0;i<(int)m_vstreams.size();i++) {
		CaptureRecording::VideoStreamRecording *vsr=m_vstreams[i]->DynamicCast<CaptureRecording::VideoStreamRecording>();
		sinfos.push_back(vsr->sinfo);
		pinfos.push_back(vsr->pinfo);
	}
	file->WriteVideoStreamInfo(sinfos,pinfos);
		
    /* Save Audio Stream Info */
	CaptureRecording::AudioStreamRecording *asr=m_astream?m_astream->DynamicCast<CaptureRecording::AudioStreamRecording>():0;
	file->WriteAudioStreamInfo(asr?&asr->sinfo:0,asr?&asr->pinfo:0);

    /* Save any serializables */
    pxcUID    uid=0;
    pxcU32    serializableDataSize=0;
    pxcBYTE  *serializableData=0;
    pxcStatus sts=device->QueryPropertyAsUID(PXCCapture::Device::PROPERTY_PROJECTION_SERIALIZABLE,&uid);
    if (sts>=PXC_STATUS_NO_ERROR) {
        sts=m_session->DynamicCast<PXCMetadata>()->QueryBuffer(uid,0,&serializableDataSize);
        if (sts>=PXC_STATUS_NO_ERROR) {
			serializableData=new pxcBYTE[serializableDataSize+sizeof(pxcU32)];
            if (serializableData) {
				*((pxcU32*)serializableData)=serializableDataSize;
                sts=m_session->DynamicCast<PXCMetadata>()->QueryBuffer(uid,serializableData+sizeof(pxcU32),&serializableDataSize);
            } else sts=PXC_STATUS_ALLOC_FAILED;
        }
    }
    file->WriteSerializables(sts>=PXC_STATUS_NO_ERROR?1:0,serializableDataSize+sizeof(pxcU32),serializableData);
	if (serializableData) delete [] serializableData;

    /* Save first frame offset */
	file->WriteFirstFrameOffset();
}

pxcStatus UtilCaptureFile::LocateStreams(std::vector<PXCCapture::VideoStream::DataDesc*> &vinputs,std::vector<PXCCapture::AudioStream::DataDesc*> &ainputs) {
    pxcStatus sts=UtilCapture::LocateStreams(vinputs,ainputs);
    if (sts>=PXC_STATUS_NO_ERROR && recording) SaveStreamInfo();
    return sts;
}

pxcStatus UtilCaptureFile::CreateCapture(pxcU32 index, PXCCapture **capture) {
    if (filename && !file) return PXC_STATUS_ITEM_UNAVAILABLE; // failed to open file
    if (!file) return UtilCapture::CreateCapture(index,capture);
    *capture=0;
    if (recording) {
        PXCCapture *capture2=0;
        pxcStatus sts=UtilCapture::CreateCapture(index,&capture2);
        if (sts<PXC_STATUS_NO_ERROR) return sts;
        *capture=new CaptureRecording(capture2,m_scheduler, types);
    } else {
        if (index>0) return PXC_STATUS_ITEM_UNAVAILABLE;
		if (wcsstr(filename,L".WAV") || wcsstr(filename,L".wav")) {
	        *capture=new WaveFilePlayback(m_session,m_scheduler,file,filename);
		} else {
		    *capture=new CapturePlayback(m_session,m_scheduler,file, types);
		}
    }
    return (*capture)?PXC_STATUS_NO_ERROR:PXC_STATUS_ITEM_UNAVAILABLE;
}

UtilCaptureFile::UtilCaptureFile(PXCSession *session,pxcCHAR *filename,pxcBool recording):UtilCapture(session) {
    this->filename=filename;
    this->recording=filename?recording:false;
    file=0;
    if (filename) {
		pxcBool sts=false;
		file=new UtilCaptureFile::DiskIO(filename,recording,sts);
		if (!sts) delete file, file=0;
	}
    types=(PXCImage::ImageType)-1;
}

UtilCaptureFile::~UtilCaptureFile(void) {
	DeleteStreams();
	m_device.ReleaseRef(); // ensure no one is using the file any more
    if (file!=NULL) delete file;
}

void UtilCaptureFile::SetPause(pxcBool pause) {
	if (file) file->SetPause(pause!=0);
	if (pause) SetRealtime(FALSE);
}

void UtilCaptureFile::SetRealtime(pxcBool realtime) {
    QueryDevice()->SetProperty((PXCCapture::Device::Property)CapturePlayback::DevicePlayback::PLAYBACK_REALTIME, (pxcF32)realtime);
}

void UtilCaptureFile::SetPosition(pxcI32 frame) {
	if (file) file->SetFrameNo(frame);
}

void UtilCaptureFile::SetTimePosition(pxcU64 ts) {
	if (file) file->SetFrameTimeStamp(ts);
}

pxcI32 UtilCaptureFile::QueryPosition(void) {
	return file?file->GetFrameNo():-1;
}

pxcU64 UtilCaptureFile::QueryTimePosition(void) {
	return file?file->GetFrameTimeStamp():0;
}

#endif