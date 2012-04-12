/**************************************************************************
###########################################################################
 This file is part of LImA, a Library for Image Acquisition

 Copyright (C) : 2009-2011
 European Synchrotron Radiation Facility
 BP 220, Grenoble 38043
 FRANCE

 This is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 This software is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, see <http://www.gnu.org/licenses/>.
###########################################################################
**************************************************************************/
#include <cstdlib>
#include <process.h>

#include <sys/timeb.h>
#include <time.h>

#include "Exceptions.h"

#include "PcoCamera.h"
#include "PcoSyncCtrlObj.h"
#include "PcoBufferCtrlObj.h"

using namespace lima;
using namespace lima::Pco;

static char *timebaseUnits[] = {"ns", "us", "ms"};

#define BUFF_INFO_SIZE 5000


char * _timestamp_pcocamera();
char * _timestamp_pcocamerautils();
char * _timestamp_pcosyncctrlobj();
char * _timestamp_pcointerface();
char * _timestamp_pcobufferctrlobj();
char * _timestamp_pcodetinfoctrlobj();
void print_hex_dump_buff(void *ptr_buff, size_t len);
	
//=========================================================================================================
char* _timestamp_pcocamerautils() {return "$Id: " __TIMESTAMP__ " (" __FILE__ ") $";}
//=========================================================================================================
//=========================================================================================================
//=========================================================================================================
enum timestampFmt {Iso, FnFull, FnDate};

static char *getTimestamp(timestampFmt fmtIdx) {
   static char timeline[128];
   errno_t err;
	time_t ltime;
	struct tm today;
	char *fmt;

  switch(fmtIdx) {
    case Iso: fmt = "%Y/%m/%d %H:%M:%S"; break;
    case FnDate: fmt = "%Y-%m-%d"; break;

    default:
    case FnFull: fmt = "%Y-%m-%d-%H%M%S"; break;
  }

	time( &ltime );
	err = localtime_s( &today, &ltime );
	strftime(timeline, 128, fmt, &today );
      
	return timeline;
}

//====================================================================
//====================================================================

char *str_trim_left(char *s) {
	char c;
	if(s == NULL) return NULL;
	while((c = *s) != 0) {
		if(!isspace(c)) break;
		s++;		
	}
	return s;
}

char *str_trim_right(char *s) {
	char *ptr;
	if(s == NULL) return NULL;
	ptr = s + strlen(s) - 1;
	while(s <= ptr) {
		if(!isspace(*ptr)) break;
		*ptr-- = 0;
	}
	return s;
}
char *str_trim(char *s) {
	return str_trim_left(str_trim_right(s));
}

//=========================================================================================================
//=========================================================================================================

char *Camera::talk(char *cmd){
	static char buff[BUFF_INFO_SIZE +1];
	return _talk(cmd, buff, BUFF_INFO_SIZE);
}

#define NRTOK 5
#define NRCMDS 50
char *Camera::_talk(char *_cmd, char *output, int lg){
	DEB_MEMBER_FUNCT();
		char cmdBuff[BUFF_INFO_SIZE +1];
		char *cmd, *key, *keys[NRCMDS];
		int ikey = 0;
		char *tok[NRTOK];
		int tokNr;
		char *ptr, *ptrMax;
		int segmentPco = m_pcoData->activeRamSegment;
		int segmentArr = segmentPco -1;
		
		ptr = output; *ptr = 0;
		ptrMax = ptr + lg;

		int width = +20;
		
		strncpy_s(cmdBuff, BUFF_INFO_SIZE, _cmd, BUFF_INFO_SIZE);
		cmd = str_trim(cmdBuff);

		if(*cmd){
			char *tokContext;
			for(int i=0; i < NRTOK; i++) {
				if( (tok[i] = strtok_s(cmd, " ", &tokContext)) == NULL) break;
				cmd = NULL;
				tokNr = i;
			}
			cmd = tok[0];
		}

		if(*cmd == 0) {
			ptr += sprintf_s(ptr, ptrMax - ptr,"**** %s [begin]\n", __FUNCTION__);

			ptr += sprintf_s(ptr, ptrMax - ptr,"**** PCO log\n");
			ptr += sprintf_s(ptr, ptrMax - ptr,"%s\n", m_log.c_str());

			ptr += sprintf_s(ptr, ptrMax - ptr,"**** PCO Info\n");
			ptr += sprintf_s(ptr, ptrMax - ptr, "* timestamp[%s]\n", getTimestamp(Iso));
			ptr += sprintf_s(ptr, ptrMax - ptr, "* cam_name[%s]\n", m_pcoData->camera_name);

			ptr += sprintf_s(ptr, ptrMax - ptr, "* roi X(%d,%d) Y(%d,%d) size(%d,%d)\n",  
					m_roi.x[0], m_roi.x[1],
					m_roi.y[0], m_roi.y[1],
					m_roi.x[1] - m_roi.x[0] + 1, m_roi.y[1] - m_roi.y[0] + 1);

			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_cocRunTime=[%g s]\n",  m_pcoData->cocRunTime);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_frameRate=[%g fps]\n", m_pcoData->frameRate);
			
			ptr += sprintf_s(ptr, ptrMax - ptr, "* Acq (ms) Tnow=[%ld] Tout=[%ld] Rec=[%ld] Xfer=[%ld]\n", 
						m_pcoData->msAcqTnow, m_pcoData->msAcqTout, m_pcoData->msAcqRec, m_pcoData->msAcqXfer);

			double _exposure, _delay;
			m_sync->getExpTime(_exposure);
			m_sync->getLatTime(_delay);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->storage_mode=[%d]\n", m_pcoData->storage_mode);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->recorder_submode=[%d]\n", m_pcoData->recorder_submode);
			
			ptr += sprintf_s(ptr, ptrMax - ptr, "* _exposure=[%g s]\n", _exposure);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* _delay=[%g s]\n", _delay);
			

			ptr += sprintf_s(ptr, ptrMax - ptr, "* wXResActual=[%d] wYResActual=[%d] \n",  m_pcoData->wXResActual,  m_pcoData->wYResActual);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* wXResMax=[%d] wYResMax=[%d] \n",  m_pcoData->wXResMax,  m_pcoData->wYResMax);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* wMetaDataSize=[%d] wMetaDataVersion=[%d] \n",  m_pcoData->wMetaDataSize,  m_pcoData->wMetaDataVersion);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* dwPixelRate=[%ld](%g MHz) dwPixelRateRequested=[%ld](%g MHz) \n",  
				m_pcoData->dwPixelRate, m_pcoData->dwPixelRate/1000000.,
				m_pcoData->dwPixelRateRequested, m_pcoData->dwPixelRateRequested/1000000.);

			ptr += sprintf_s(ptr, ptrMax - ptr, "* Valid dwPixelRate=[%ld][%ld][%ld][%ld] \n",  
				m_pcoData->pcoInfo.dwPixelRateDESC[0],m_pcoData->pcoInfo.dwPixelRateDESC[1],
				m_pcoData->pcoInfo.dwPixelRateDESC[2],m_pcoData->pcoInfo.dwPixelRateDESC[3]);

			ptr += sprintf_s(ptr, ptrMax - ptr, "* wLUT_Identifier[x%04x] wLUT_Parameter [x%04x]\n",
				m_pcoData->wLUT_Identifier, m_pcoData->wLUT_Parameter);

			int iFrames;
			m_sync->getNbFrames(iFrames);
			ptr += sprintf_s(ptr, ptrMax - ptr, "* m_sync->getNbFrames=[%d frames]\n", iFrames);

			if(_getCameraType()==CAMERATYPE_PCO_DIMAX_STD){
				ptr += sprintf_s(ptr, ptrMax - ptr, "* DIMAX info\n");
				ptr += sprintf_s(ptr, ptrMax - ptr, "* PcoActiveSegment=[%d]\n", segmentArr+1);
				ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwMaxFramesInSegment[%d]=[%d frames]\n", segmentArr, m_pcoData->dwMaxFramesInSegment[segmentArr]);
				ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwSegmentSize[%d]=[%d pages]\n", segmentArr, m_pcoData->dwSegmentSize[segmentArr]);
				ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->wPixPerPage[%d pix]\n", m_pcoData->wPixPerPage);
				ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwValidImageCnt[%d]=[%ld]\n", segmentArr, m_pcoData->dwValidImageCnt[segmentArr]);
				ptr += sprintf_s(ptr, ptrMax - ptr, "* m_pcoData->dwMaxImageCnt[%d]=[%ld]\n", segmentArr, m_pcoData->dwMaxImageCnt[segmentArr]);
			}

			ptr += sprintf_s(ptr, ptrMax - ptr,"**** %s [end]\n", __FUNCTION__);
			return output;
		}
		
		key = keys[ikey++] = "cocRunTime";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%g",  m_pcoData->cocRunTime);
			return output;
		}

		key = keys[ikey++] = "frameRate";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%g", m_pcoData->frameRate);
			return output;
		}

		key = keys[ikey++] = "timestamp";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcocamera());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcosyncctrlobj());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcointerface());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcobufferctrlobj());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcodetinfoctrlobj());
			ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", _timestamp_pcocamerautils());
			
			return output;
		}


		key = keys[ikey++] = "clTransferParam";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "      baudrate=[%u] %g Kbps\n", m_pcoData->clTransferParam.baudrate, m_pcoData->clTransferParam.baudrate/1000.);
			ptr += sprintf_s(ptr, ptrMax - ptr, "ClockFrequency=[%u] %g MHz\n", m_pcoData->clTransferParam.ClockFrequency, m_pcoData->clTransferParam.ClockFrequency/1000000.);
			ptr += sprintf_s(ptr, ptrMax - ptr, "        CCline=[%u]\n", m_pcoData->clTransferParam.CCline);
			ptr += sprintf_s(ptr, ptrMax - ptr, "    DataFormat=[%u]\n", m_pcoData->clTransferParam.DataFormat);
			ptr += sprintf_s(ptr, ptrMax - ptr, "      Transmit=[%u]\n", m_pcoData->clTransferParam.Transmit);
			
			return output;
		}

		key = keys[ikey++] = "maxNbImages";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "%ld", m_pcoData->dwMaxImageCnt[segmentArr]);
			return output;
		}

		key = keys[ikey++] = "acqTime";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "* Acq (ms) Tnow=[%ld] Tout=[%ld] Rec=[%ld] Xfer=[%ld]\n", 
						m_pcoData->msAcqTnow, m_pcoData->msAcqTout, m_pcoData->msAcqRec, m_pcoData->msAcqXfer);
			return output;
		}

		key = keys[ikey++] = "allocatedBuffer";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "AllocatedBuffer: Done=[%d] Nr=[%d] Size=[%ld][%g MB]\n", 
				m_pcoData->bAllocatedBufferDone, 
				m_pcoData->iAllocatedBufferNumber, 
				m_pcoData->dwAllocatedBufferSize, m_pcoData->dwAllocatedBufferSize/1000000.);
			
			return output;
		}

		key = keys[ikey++] = "testCmd";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){

			if((tokNr == 2) &&  (_stricmp(tok[1], "time")==0)){
				ptr += sprintf_s(ptr, ptrMax - ptr, "sleeping\n"); 
				::Sleep(atoi(tok[2])*1000);
				ptr += sprintf_s(ptr, ptrMax - ptr, "sleeping\n"); 
			}

			if(tokNr == 0) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "tokNr [%d] cmd [%s] No parameters", tokNr, cmd); 
			} else {
				ptr += sprintf_s(ptr, ptrMax - ptr, "tokNr [%d] cmd [%s]\n", tokNr, cmd); 
				for(int i = 1; i<= tokNr; i++) {
					ptr += sprintf_s(ptr, ptrMax - ptr, "tok [%d] [%s]\n", i, tok[i]); 
				}
			}
			return output;
		}


		key = keys[ikey++] = "rollingShutter";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			DWORD dwSetup; int error;

			if(_getCameraType() != CAMERATYPE_PCO_EDGE) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "invalid cmd / only for EDGE");
				return output;
			}
			
			if(tokNr == 0) {
				_pco_GetCameraSetup(dwSetup, error);
				ptr += sprintf_s(ptr, ptrMax - ptr, "%d", dwSetup);
				return output;
			}

			if((tokNr != 1) || ((strcmp(tok[1],"0") != 0) && (strcmp(tok[1],"1") != 0))){
				ptr += sprintf_s(ptr, ptrMax - ptr, "syntax ERROR - %s <0 | 1>", cmd);
				return output;
			}
			
			dwSetup = atoi(tok[1]);

			_pco_SetCameraSetup(dwSetup, error);
			ptr += sprintf_s(ptr, ptrMax - ptr, "%d", dwSetup);
			return output;
		}


		key = keys[ikey++] = "pixelRate";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			DWORD _dwPixelRateRequested;

			if(_getCameraType() != CAMERATYPE_PCO_EDGE) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "invalid cmd / only for EDGE");
				return output;
			}
			
			if(tokNr == 0) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "%ld", m_pcoData->dwPixelRate);
				return output;
			}

			if((tokNr != 1)){
				ptr += sprintf_s(ptr, ptrMax - ptr, "syntax ERROR - %s <value Hz>", cmd);
				return output;
			}
			
			_dwPixelRateRequested = atoi(tok[1]);

			if((_dwPixelRateRequested != 0) && (!_isValid_pixelRate(_dwPixelRateRequested))){
				ptr += sprintf_s(ptr, ptrMax - ptr, "value out of range");
				return output;
			}

			m_pcoData->dwPixelRateRequested = _dwPixelRateRequested;		
			ptr += sprintf_s(ptr, ptrMax - ptr, "%ld", m_pcoData->dwPixelRateRequested);
			return output;
		}




		key = keys[ikey++] = "roi";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){

			if((tokNr != 0) && (tokNr != 4)){
					ptr += sprintf_s(ptr, ptrMax - ptr, "syntax ERROR - %s [x0 y0 x1 y1]", cmd);
					return output;
			}
				
			if(tokNr == 4){
				struct stcRoi roi;
				roi.x[0]= atoi(tok[1]);
				roi.y[0]= atoi(tok[2]);
				roi.x[1]= atoi(tok[3]);
				roi.y[1]= atoi(tok[4]);

				if(!_isValid_Roi(&roi)){
					ptr += sprintf_s(ptr, ptrMax - ptr, "value out of range");
					return output;
				}

				m_roi.x[0]= roi.x[0];
				m_roi.x[1]= roi.x[1];
				m_roi.y[0]= roi.y[0];
				m_roi.y[1]= roi.y[1];
				m_roi.changed = Changed;
			} 

			ptr += sprintf_s(ptr, ptrMax - ptr, "* roi X(%d,%d) Y(%d,%d) size(%d,%d)\n",  
					m_roi.x[0], m_roi.x[1],
					m_roi.y[0], m_roi.y[1],
					m_roi.x[1] - m_roi.x[0] + 1, m_roi.y[1] - m_roi.y[0] + 1);
				return output;

		}




		key = keys[ikey++] = "cameraType";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			ptr += sprintf_s(ptr, ptrMax - ptr, "sn[%ld] type: cam[%x][%x]if[%x] ver: hw[%lx]fw[%lx]\n", 
				m_pcoData->stcCamType.dwSerialNumber, 
				m_pcoData->stcCamType.wCamType, 
				m_pcoData->stcCamType.wCamSubType, 
				m_pcoData->stcCamType.wInterfaceType,
				m_pcoData->stcCamType.dwHWVersion, 
				m_pcoData->stcCamType.dwFWVersion

				);
			
			return output;
		}


		key = keys[ikey++] = "lastError";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			m_pcoData->pcoErrorMsg[ERR_SIZE] = 0;
			ptr += sprintf_s(ptr, ptrMax - ptr, "[%d] [%s]\n", 
				m_pcoData->pcoError, m_pcoData->pcoErrorMsg
				);
			
			return output;
		}

		key = keys[ikey++] = "lastError";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			m_pcoData->pcoErrorMsg[ERR_SIZE] = 0;
			ptr += sprintf_s(ptr, ptrMax - ptr, "[%d] [%s]\n", 
				m_pcoData->pcoError, m_pcoData->pcoErrorMsg
				);
			
			return output;
		}

		key = keys[ikey++] = "dumpData";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){

			print_hex_dump_buff(m_pcoData, sizeof(stcPcoData));
			ptr += sprintf_s(ptr, ptrMax - ptr, "dumped\n");
			
			return output;
		}

		key = keys[ikey++] = "?";     //----------------------------------------------------------------
		if(_stricmp(cmd, key) == 0){
			for(int i = 0; i < ikey; i++) {
				ptr += sprintf_s(ptr, ptrMax - ptr, "%s\n", keys[i]);
			}
			ptr += sprintf_s(ptr, ptrMax - ptr, "--- nrCmds[%d][%d]\n", ikey, NRCMDS);
			return output;
		}


		sprintf_s(ptr, ptrMax - ptr, "ERROR unknown cmd [%s]", cmd);
		return output;
}


//====================================================================
// utils
//====================================================================

#define LEN_LINE_BUFF	128
#define BYTES_PER_LINE		16
#define OFFSET_COL_HEX1	(4 + 1 + 3)
#define OFFSET_COL_HEX2	(OFFSET_COL_HEX1 + (8 * 3) + 2)
#define OFFSET_COL_ASC1	(OFFSET_COL_HEX2 + (8 * 3) + 3)
#define OFFSET_COL_ASC2	(OFFSET_COL_ASC1 + 8 + 1)

//--------------------------------------------------------------------
//--------------------------------------------------------------------
char *nibble_to_hex(char *s, BYTE nibble)
{
  nibble &= 0x0f;
  *s = (nibble <= 9) ?  '0' + nibble : 'a' + nibble - 10;
  return s+1;
}



//--------------------------------------------------------------------
//--------------------------------------------------------------------
char *byte_to_hex(char *s, BYTE byte)
{
  s = nibble_to_hex(s, byte >> 4);
  return nibble_to_hex(s, byte);;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
char *word_to_hex(char *s, WORD word)
{
  s = byte_to_hex(s, word >> 8);
  return byte_to_hex(s, (BYTE) word);
  
}


//--------------------------------------------------------------------
//--------------------------------------------------------------------
char *dword_to_hex(char *s, DWORD dword)
{
  s = word_to_hex(s, dword >> 16);
  return word_to_hex(s, (WORD) dword);
  
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
const char *hex_dump_line(void *buff, size_t len, size_t *nr, WORD *offset) {

	static char line_buff[LEN_LINE_BUFF];

	char *s;
	BYTE *ptr_buff;
	BYTE *ptr_buff0;
	int i, nr1, nr2;

	ptr_buff = (BYTE *) buff;
	ptr_buff0 = ptr_buff;

	s = line_buff;


	if(len <= 0) {
		strcpy_s(s,LEN_LINE_BUFF,"<empty>");
		return line_buff;
	}

	memset(line_buff, ' ', LEN_LINE_BUFF);

	*nr = (len < BYTES_PER_LINE) ? len : BYTES_PER_LINE;

	s = word_to_hex(s, *offset);
	*offset += *nr;
	*s = ':';
	s = line_buff + OFFSET_COL_HEX1;

	if(*nr <= BYTES_PER_LINE / 2) {
		nr1 = *nr;
		nr2 = -1;
	} else {
		nr1 = BYTES_PER_LINE / 2;
		nr2 = *nr - nr1;
	}

	for(i = 0; i < nr1; i++) {
		s = byte_to_hex(s, *ptr_buff++);
		s++;
	}

	if(nr2 >0 ){
		*s = '-';
		s = line_buff + OFFSET_COL_HEX2;
		for(i = 0; i < nr2; i++) {
			s = byte_to_hex(s, *ptr_buff++);
			s++;
		}
	}	

	ptr_buff = ptr_buff0;
	s = line_buff + OFFSET_COL_ASC1;
	for(i = 0; i < nr1; i++) {
		*s++ = (isprint(*ptr_buff)) ? *ptr_buff : '.';
		ptr_buff++;
	}

	if(nr2 >0 ){
		s = line_buff + OFFSET_COL_ASC2;
		for(i = 0; i < nr2; i++) {
			*s++ = (isprint(*ptr_buff)) ? *ptr_buff : '.';
			ptr_buff++;
		}
	}	
	
	*s = 0;

	return line_buff;

}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
void print_hex_dump_buff(void *ptr_buff, size_t len) {
	WORD offset = 0;
	size_t nr = 0;
	BYTE * ptr = (BYTE *) ptr_buff;
	;
	
	printf("dump buff / len: %d\n", len);
	
	while(len > 0) {
		printf("%s\n", hex_dump_line(ptr, len, &nr, &offset));
		len -= nr;
		ptr += nr;
	}

}





