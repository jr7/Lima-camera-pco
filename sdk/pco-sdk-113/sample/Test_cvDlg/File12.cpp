#include "stdafx.h"
#ifndef FILE12_H
#include "file12.h"

#define FILEVERSION200 200
#define FILEVERSION300 300
#define FILEVERSION301 301
#define FILEVERSION302 302             // eingeführt 05.2005 / ab CamWare 2.13
#define FILEVERSION303 303             // eingeführt 04.2008 / ab CamWare 2.21


#endif

void Shift(WORD* data, unsigned long k, bool bup, unsigned long bits)
{
  if(bup)
  {
    for(unsigned int i = 0; i < k; i++)
      data[i] <<= bits;
  }
  else
  {
    for(unsigned int i = 0; i < k; i++)
      data[i] >>= bits;
  }
}



/********************************************************************************************************/
/* added TIFF - Reader... Franz Reitner/24.06.1999                                                        */
/********************************************************************************************************/
const short ImageWidth      = 256;     // IFD - Einträge (Image File Descriptor)
const short ImageHeight     = 257;
const short BitsPerPixel    = 258;
const short Compression     = 259;
const short PhotoInterp     = 262;
const short StripOffsets    = 273;
const short SamplesPerPixel = 277;
const short RowsPerStrip    = 278;
const short StripByteCnt    = 279;
const short XResolution     = 282;
const short YResolution     = 283;
const short PlanarConfig    = 284;
const short ResolutionUnits = 296;
const short ColorMap        = 320;

/* Aufbau TIF - File: Header 0x49 0x49 0x2A 0x2A(Kennung mit Version)
XXXXYYYY(Offset für IFD)
.... DATEN(entweder vor oder nach IFD)
bei Adresse XXXXYYYY: ZZZZ Anzahl Einträge in IFD
.... IFDs
.... DATEN(entweder vor oder nach IFD) */

typedef struct                         // IFD - Struktur
{
  unsigned short TagField;             // IFD ID
  unsigned short ftype;                // Type (Byte,String,Short,Long,Float)
  unsigned long length;                // Anzahl Einträge
  unsigned long Offset;                // Datum, falls Anzahl Einträge=1; ansonsten Offset im File
}TE;

TE TIFFEntry;

int IsTiffFile(char *filename, HANDLE file)
{
  char bfr[4];
  DWORD read, currPos;
  int ok;
  bool bl = FALSE;


  if ((file == NULL) &&(filename == NULL))
    return false;

  if ((file == NULL) &&(filename != NULL))
  {
    file = CreateFile(filename,
      GENERIC_READ,
      0,
      0,
      OPEN_EXISTING,
      0,
      0);
    bl = TRUE;
  }
  if (file == INVALID_HANDLE_VALUE)
    return false;

  currPos = SetFilePointer(file, 0, 0, FILE_CURRENT);
  SetFilePointer(file, 0, 0, FILE_BEGIN);
  if (!ReadFile(file, bfr, 4, &read, 0))
  {
    ok = FALSE; 
  }
  else
  {
    if ((bfr[0] == 'I') &&(bfr[1] == 'I') &&(bfr[2] == 0x2A) &&(bfr[3] == 0))
      ok = FILEISOK;                   // nur INTEL-Format wird gelesen, sonst Drehwurm
    else                               // Intel speichert zuerst Lowbyte dann Highbyte ab
      if ((bfr[0] == 'M') &&(bfr[1] == 'M') &&(bfr[3] == 0x2A) &&(bfr[2] == 0))
        ok = FILEISOK | FILEISMACFORMAT;                   // nur INTEL-Format wird gelesen, sonst Drehwurm
      else                               // Intel speichert zuerst Lowbyte dann Highbyte ab
        ok = 0;                          // 0x1234 steht im Speicher als 34 12, bei Motorroller umgekehrt
  }
  SetFilePointer(file, currPos, 0, FILE_BEGIN);
  if (bl)
    CloseHandle(file);
  return ok;
}                                      // IsTiffFile

void mswab(byte* src, byte* dest, int isize)
{
  dest+= isize;
  dest--;
  for(int i = 0; i < isize; i++)
  {
    *dest = *src;
    src++;
    dest--;
  }
}

int getsize_tif (char *filename, int *iXRes, int *iYRes, bool *bDouble)
{
  HANDLE hfread;
  unsigned long DataOffset = 0;

  int width = 0, height = 0, ifdcnt = 0;
  long offset = 0, lh;
  unsigned short ush, ush2;
  DWORD read = 0;
  Bild *strBild;
  bool bmacformat = FALSE;

  *bDouble = FALSE;
  hfread = CreateFile(filename,
    GENERIC_READ,
    0,
    NULL,
    OPEN_EXISTING,
    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
    0);

  if (hfread== INVALID_HANDLE_VALUE)
  {
    return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
  }

  ifdcnt = IsTiffFile(NULL, hfread);
  if (ifdcnt == 0)
  {
    CloseHandle(hfread);
    return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
  }
  if((ifdcnt & FILEISMACFORMAT) == FILEISMACFORMAT)
    bmacformat = TRUE;

  SetFilePointer(hfread, 4, 0, FILE_BEGIN);
  ReadFile(hfread, &offset, sizeof(offset), &read, 0);
  if(bmacformat)
  {
    mswab((byte*)&offset, (byte*)&lh, sizeof(lh));
    offset = lh;
  }
  SetFilePointer(hfread, offset, 0, FILE_BEGIN);
  ReadFile(hfread, &ush, 2, &read, 0);
  if(bmacformat)
  {
    mswab((byte*)&ush, (byte*)&ush2, sizeof(ush2));
    ifdcnt = ush2;
  }
  else
    ifdcnt = ush;

  offset += 2;

  for (int i = 0; i < ifdcnt; i++)
  {
    SetFilePointer(hfread, offset + i * sizeof(TIFFEntry), 0, FILE_BEGIN);
    ReadFile(hfread, &TIFFEntry, sizeof(TIFFEntry), &read, 0);
    if(bmacformat)
    {
      mswab((byte*)&TIFFEntry.ftype, (byte*)&ush, sizeof(ush));
      TIFFEntry.ftype = ush;
      mswab((byte*)&TIFFEntry.TagField, (byte*)&ush, sizeof(ush));
      TIFFEntry.TagField = ush;
      mswab((byte*)&TIFFEntry.length, (byte*)&lh, sizeof(lh));
      TIFFEntry.length = lh;
      if(TIFFEntry.length == 1)
      {
        if(TIFFEntry.ftype == 3)
        {
          mswab((byte*)&TIFFEntry.Offset, (byte*)&ush, sizeof(ush));
          TIFFEntry.Offset = ush;
        }
        if(TIFFEntry.ftype == 4)
        {
          mswab((byte*)&TIFFEntry.Offset, (byte*)&lh, sizeof(lh));
          TIFFEntry.Offset = lh;
        }
      }
    }
    if (TIFFEntry.TagField == ImageWidth)
      width = TIFFEntry.Offset;
    if (TIFFEntry.TagField == ImageHeight)
      height = TIFFEntry.Offset;
    if (TIFFEntry.TagField == 0xC53F)
    {
      unsigned char* pucdat;
      int ilen = -1;

      DataOffset = TIFFEntry.Offset;
      if(TIFFEntry.ftype == 1)
      {
        ilen = TIFFEntry.length;
      }
      if(TIFFEntry.ftype == 3)
      {
        ilen = TIFFEntry.length * sizeof(short);
      }
      if(TIFFEntry.ftype == 4)
      {
        ilen = TIFFEntry.length * sizeof(long);
      }
      if (ilen == -1)
      {
        CloseHandle(hfread);
        return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
      }
      if(ilen > sizeof(Bild))
        ilen = sizeof(Bild);
      pucdat = (unsigned char*) malloc(ilen + 0x10);
      strBild = (Bild*)&pucdat[0];

      if((TIFFEntry.ftype == 4)||(TIFFEntry.ftype == 1))// Type 4: Multitif (alt); Type 1: Multitif und normales Tif (neu, 303) 
      {
        strBild = (Bild*)&pucdat[2];
      }

      SetFilePointer(hfread, DataOffset, 0, FILE_BEGIN);
      ReadFile(hfread, &pucdat[2], ilen, &read, 0);

      //ReadFile(hfread, &SCnt, sizeof(word), &read, 0);
      //ReadFile(hfread, &strBild.sTime, SCnt, &read, 0);

      if((strBild->iVersion == FILEVERSION200) ||
        (strBild->iVersion == FILEVERSION300) ||
        (strBild->iVersion == FILEVERSION301) ||
        (strBild->iVersion == FILEVERSION302) ||
        (strBild->iVersion == FILEVERSION303))
      {
        *bDouble = strBild->bDouble;
      }
      else
        *bDouble = FALSE;
      free(pucdat);
    }
  }

  CloseHandle(hfread);

  if (!((width != 0) &&(height != 0)))
  {
    *iXRes = 0;
    *iYRes = 0;
    return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
  }
  *iXRes = width;
  *iYRes = height;

  return (PCO_NOERROR);
}

int read_tif (char *filename, Bild* strBild, int iNoShifting)
{
  HANDLE hfread = NULL;
  void *im;

  int width = 0, height = 0, ifdcnt = 0, iImageType = 0, iBitsR = 0, 
    iBitsG = 0, iBitsB = 0, iRowsPerStrip = 0;
  unsigned long offset = 0, DataOffset = 0, lFileLengthL, lFileLengthH;
  long lh;
  unsigned short ush, ush2;
  int ishift;
  bool bmulti = FALSE;
  bool bmacformat = FALSE;
  int    SDataBytes = 0, SDataCnt = 0, SBytes = 0, SCnt = 0;

  DWORD *SDataOffset = NULL, *SByteCnt = NULL;
  DWORD read = 0;
  WORD  *p = NULL;
  int err = PCO_NOERROR;

  im = strBild->pic12;

  if(strBild->bAlignUpper)
  {
    ishift = (16 - strBild->iBitRes);
  }
  else
    ishift = 0;

  hfread = CreateFile(filename,
    GENERIC_READ,
    0,
    NULL,
    OPEN_EXISTING,
    FILE_ATTRIBUTE_NORMAL | FILE_FLAG_RANDOM_ACCESS,
    0);

  lFileLengthL = GetFileSize(hfread, &lFileLengthH);

  if (hfread== INVALID_HANDLE_VALUE)
  {
    return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
  }

  SCnt = IsTiffFile(NULL, hfread);
  if (SCnt == 0)
  {
    CloseHandle(hfread);
    return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
  }
  if((SCnt & FILEISMACFORMAT) == FILEISMACFORMAT)
    bmacformat = TRUE;

  SetFilePointer(hfread, 4, 0, FILE_BEGIN);
  ReadFile(hfread, &offset, sizeof(offset), &read, 0);
  if(bmacformat)
  {
    mswab((byte*)&offset, (byte*)&lh, sizeof(lh));
    offset = lh;
  }
  SetFilePointer(hfread, offset, 0, FILE_BEGIN);
  ReadFile(hfread, &ush, 2, &read, 0);
  if(bmacformat)
  {
    mswab((byte*)&ush, (byte*)&ush2, sizeof(ush2));
    ifdcnt = ush2;
  }
  else
    ifdcnt = ush;
  offset += 2;

  for (int i = 0; i < ifdcnt; i++)      // Einträge siehe TIFF - Beschreibung (tiff.pdf vom 03.06.1992)
  {                                    //                       (ist aktuelle Vers.! 24.06.1999/Franz)
    //                       (http:// www.adobe.com)
    SetFilePointer(hfread, offset + i * sizeof(TIFFEntry), 0, FILE_BEGIN);
    ReadFile(hfread, &TIFFEntry, sizeof(TIFFEntry), &read, 0);
    if(bmacformat)
    {
      mswab((byte*)&TIFFEntry.ftype, (byte*)&ush, sizeof(ush));
      TIFFEntry.ftype = ush;
      mswab((byte*)&TIFFEntry.TagField, (byte*)&ush, sizeof(ush));
      TIFFEntry.TagField = ush;
      mswab((byte*)&TIFFEntry.length, (byte*)&lh, sizeof(lh));
      TIFFEntry.length = lh;
      if(TIFFEntry.length == 1)
      {
        if(TIFFEntry.ftype == 3)
        {
          mswab((byte*)&TIFFEntry.Offset, (byte*)&ush, sizeof(ush));
          TIFFEntry.Offset = ush;
        }
        if(TIFFEntry.ftype == 4)
        {
          mswab((byte*)&TIFFEntry.Offset, (byte*)&lh, sizeof(lh));
          TIFFEntry.Offset = lh;
        }
      }
      else
      {
        mswab((byte*)&TIFFEntry.Offset, (byte*)&lh, sizeof(lh));
        TIFFEntry.Offset = lh;
      }
    }
    switch (TIFFEntry.TagField)
    {
      case ImageWidth:
      {
        if (TIFFEntry.ftype == 3)
          width = TIFFEntry.Offset;// word
        if (TIFFEntry.ftype == 4)
          width = TIFFEntry.Offset;// dword
        break;
      }
      case ImageHeight:
      {
        if (TIFFEntry.ftype == 3)
          height = TIFFEntry.Offset;// word
        if (TIFFEntry.ftype == 4)
          height = TIFFEntry.Offset;// dword
        break;
      }
      case BitsPerPixel:               // Auflösung in bit je Pixel (8bit BW, 16bit BW oder 24bit RGB)
      {
        short sValue;

        sValue =(short)TIFFEntry.Offset;// word
        if (TIFFEntry.length == 1)
        {
          if ((sValue > 16) || (sValue < 8))
            err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE;
          if (sValue <= 8)
            iImageType = 1;
          else
            iImageType = 2;            // 1 -> 8 bit BW; 2 -> 9...16 bit BW
        }
        if ((TIFFEntry.length == 3) ||(TIFFEntry.length == 4))
        {
          short buf[5];

          if (TIFFEntry.ftype == 3)
          {
            SetFilePointer(hfread, TIFFEntry.Offset, 0, FILE_BEGIN);
            ReadFile(hfread, buf, 6, &read, 0);
            if(bmacformat)
            {
              mswab((byte*)&buf[0], (byte*)&ush, sizeof(ush));
              buf[0] = ush;
              mswab((byte*)&buf[1], (byte*)&ush, sizeof(ush));
              buf[1] = ush;
              mswab((byte*)&buf[2], (byte*)&ush, sizeof(ush));
              buf[2] = ush;
            }
            iBitsR = buf[0];
            iBitsG = buf[1];
            iBitsB = buf[2];
            if ((iBitsR > 8) ||(iBitsG > 8) ||(iBitsB > 8))
              err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
            else
              iImageType = TIFFEntry.length;          // 3 -> RGB Bild
          }
          else
            err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
        }
        break;
      }
      case Compression:                // 1: kein Packen, 2: Huffman, 3: Fax G3,
      {                               // 4: Fax G4, 5: LZW, 32773: PackBits
        if (TIFFEntry.Offset != 1)// word
          err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
        break;
      }
      case PhotoInterp:                // 0: bilevel u. Graustufen, 0 ist weis, 1: bilevel u. Gr., 0 ist schwarz
      {                               // 2: RGB 3: RGB über Palette (nicht ausgeführt)
        if ((TIFFEntry.Offset != 1) &&(TIFFEntry.Offset != 2))// word
          err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
        break;
      }
      case 0xC53F:
      {
        unsigned char* pucdat;
        int ilen = -1;
        Bild *strbildl;

        DataOffset = TIFFEntry.Offset;
        if(TIFFEntry.ftype == 1)
        {
          ilen = TIFFEntry.length;
        }
        if(TIFFEntry.ftype == 3)
        {
          ilen = TIFFEntry.length * sizeof(short);
        }
        if(TIFFEntry.ftype == 4)
        {
          ilen = TIFFEntry.length * sizeof(long);
          bmulti = TRUE;
        }
        if (ilen == -1)
        {
          CloseHandle(hfread);
          return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
        }
        if(ilen > sizeof(Bild))
          ilen = sizeof(Bild);
        pucdat = (unsigned char*) malloc(ilen + 0x10);
#if defined _WIN64
        strbildl = (Bild*)&pucdat[0];
#else
        strbildl = (Bild*)&pucdat[4];
#endif
        if((TIFFEntry.ftype == 4)||(TIFFEntry.ftype == 1))// 4: Multi(alt) 1: tif und multi (neu, ab 303)
        {
#if defined _WIN64
          strbildl = (Bild*)&pucdat[2];
#else
          strbildl = (Bild*)&pucdat[6];
#endif
        }

        SetFilePointer(hfread, DataOffset, 0, FILE_BEGIN);
        ReadFile(hfread, &pucdat[6], ilen, &read, 0);
        if((strbildl->iVersion == FILEVERSION200) ||
          (strbildl->iVersion == FILEVERSION300) ||
          (strbildl->iVersion == FILEVERSION301) ||
          (strbildl->iVersion == FILEVERSION302) ||
          (strbildl->iVersion == FILEVERSION303))
        {
          memcpy(&strBild->sTime, &strbildl->sTime, sizeof(Bild) - sizeof(word*));
          if(strBild->iVersion < FILEVERSION200)
          {
            strBild->iBWMin2 = strBild->iBWMin;                   // Lut bw min
            strBild->iBWMax2 = strBild->iBWMax;                   // Lut bw max
            strBild->iBWLut2 = strBild->iBWLut;                   // Lut lin log
            strBild->iRMin2 = strBild->iRMin;                    // red min
            strBild->iRMax2 = strBild->iRMax;                    // red max
            strBild->iGMin2 = strBild->iGMin;                    // green min
            strBild->iGMax2 = strBild->iGMax;                    // green max
            strBild->iBMin2 = strBild->iBMin;                    // blue min
            strBild->iBMax2 = strBild->iBMax;                    // blue max
            strBild->iColLut2 = strBild->iColLut;                  // Lut lin log color
            strBild->bAlignUpper = FALSE;
          }

          if(strBild->iVersion < FILEVERSION300)
          {
            strBild->dGammaLut = 1.0;                 // Gamma value b/w
            strBild->dGammaLutC = 1.0;                // Gamma value color
            strBild->dGammaLut2 = 1.0;                // Gamma value b/w 2
            strBild->dGammaLutC2 = 1.0;               // Gamma value color 2
          }

          if(strBild->iVersion < FILEVERSION301)
          {
            strBild->iBitRes = 12;                    // Assume 14bit resolution
          }

          if(strBild->iVersion < FILEVERSION302)
            strBild->dSaturation = 100;
          // close filehandle
        }
        free(pucdat);

        break;
      }
      case StripOffsets:               // Adresse der einzelnen Strips in der Datei
      {
        DataOffset = TIFFEntry.Offset;
        if (TIFFEntry.length > 1)
        {
          char cSize[6] = { 0, 1, 0, 2, 4, 8 };
          int iBytes;

          if ((TIFFEntry.ftype != 3) &&(TIFFEntry.ftype != 4))
          {
            err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
            break;
          }
          iBytes = cSize[TIFFEntry.ftype];

          SDataOffset =(DWORD*)malloc(TIFFEntry.length * iBytes);

          SetFilePointer(hfread, DataOffset, 0, FILE_BEGIN);
          ReadFile(hfread, SDataOffset, TIFFEntry.length * iBytes, &read, 0);
          if(bmacformat)
          {
            for(unsigned int i = 0; i < TIFFEntry.length; i++)
            {
              if(iBytes == 2)
              {
                mswab((byte*)&SDataOffset[i], (byte*)&ush, sizeof(ush));
                SDataOffset[i] = ush;
              }
              else
              {
                mswab((byte*)&SDataOffset, (byte*)&lh, sizeof(lh));
                SDataOffset[i] = lh;
              }
            }
          }
          SDataBytes = iBytes;
          SDataCnt   = TIFFEntry.length;
          if (read !=(DWORD)(SDataCnt * SDataBytes))
          {
            err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
            break;
          }
        }
        else
        {
          if (TIFFEntry.length == 1)
          {
            char cSize[6] = { 0, 1, 0, 2, 4, 8 };
            int iBytes;

            if ((TIFFEntry.ftype != 3) &&(TIFFEntry.ftype != 4))
            {
              err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
              break;
            }
            iBytes = cSize[TIFFEntry.ftype];

            SDataOffset =(DWORD*)malloc(TIFFEntry.length * iBytes);
            *SDataOffset = DataOffset;
            SDataCnt   = TIFFEntry.length;
          }
          else
          {
            SDataOffset = NULL;
          }
        }
        break;
      }
      case SamplesPerPixel:            // 1: BW, 3: RGB (24bit), 4: RGB (32bit)
      {
        if ((TIFFEntry.Offset != 1) &&(TIFFEntry.Offset != 3) &&(TIFFEntry.Offset != 4))// word
          err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
        break;
      }
      case RowsPerStrip:               // Anzahl Pixel je Strip
      {
        if (TIFFEntry.ftype == 3)
          iRowsPerStrip = TIFFEntry.Offset;
        else
          if (TIFFEntry.ftype == 4)
            iRowsPerStrip = TIFFEntry.Offset;
          else
            err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
        break;
      }
      case StripByteCnt:               // Anzahl Bytes der jeweiligen Strips
      {
        DataOffset = TIFFEntry.Offset;
        char cSize[6] = { 0, 1, 0, 2, 4, 8 };
        int iBytes;

        if (TIFFEntry.length > 1)
        {
          if ((TIFFEntry.ftype != 3) &&(TIFFEntry.ftype != 4))
          {
            err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
            break;
          }
          iBytes = cSize[TIFFEntry.ftype];

          SByteCnt =(DWORD*)malloc(TIFFEntry.length * iBytes);

          SetFilePointer(hfread, DataOffset, 0, FILE_BEGIN);
          ReadFile(hfread, SByteCnt, TIFFEntry.length * iBytes, &read, 0);
          if(bmacformat)
          {
            for(unsigned int i = 0; i < TIFFEntry.length; i++)
            {
              if(iBytes == 2)
              {
                mswab((byte*)&SByteCnt[i], (byte*)&ush, sizeof(ush));
                SByteCnt[i] = ush;
              }
              else
              {
                mswab((byte*)&SByteCnt, (byte*)&lh, sizeof(lh));
                SByteCnt[i] = lh;
              }
            }
          }
          SBytes = iBytes;
          SCnt   = TIFFEntry.length;

          if (read !=(DWORD)(SCnt * SBytes))
          {
            err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
          }
        }
        else
        {
          if (TIFFEntry.length == 1)
          {
            if ((TIFFEntry.ftype != 3) &&(TIFFEntry.ftype != 4))
            {
              err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
              break;
            }
            iBytes = cSize[TIFFEntry.ftype];
            SBytes = iBytes;
            SByteCnt =(DWORD*)malloc(TIFFEntry.length * iBytes);
            *SByteCnt = TIFFEntry.Offset;
            SCnt   = TIFFEntry.length;
          }
          else
          {
            SByteCnt = NULL;
          }
        }
        break;
      }
      case PlanarConfig:               // 1: BW, normal RGB (RGBRGBRGB...) 2: Planes (nicht ausgeführt)
      {
        if (TIFFEntry.Offset != 1)// word
          err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
        break;
      }
    }
    if (err != 0)
      break;
  }

  if (SDataCnt != SCnt)
    err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;

  if(bmulti == TRUE)
  {
    if(*SByteCnt != (DWORD)(width * height * iImageType))
      *SByteCnt = (DWORD)(width * height * iImageType);// Korrektur eines Fehlers in mmfilewriter...
  }

  if (err)
  {
    CloseHandle(hfread);
    if (SDataOffset != NULL)
      free(SDataOffset);
    if (SByteCnt != NULL)
      free(SByteCnt);
    return (err);
  }

  WORD *os, *bc;
  DWORD *dwp;
  WORD  *cp;
  char  *cccp;
  unsigned long lOs, lBc;

  p =(WORD*)malloc(width * height * iImageType);

  os =(WORD*)SDataOffset;
  bc =(WORD*)SByteCnt;
  cp = p;
  for (short j = 0; j < SCnt; j++)      // Stripdaten aus File auslesen und in Zwischenspeicher eintragen
  {
    if (SBytes == 4)                    // Pointer sind DWORDs
    {
      dwp =(DWORD*)os;
      lOs = *dwp;
      dwp =(DWORD*)bc;
      lBc = *dwp;
    }
    else                               // Pointer sind WORDs
    {
      lOs = *os;
      lBc = *bc;
    }
    if (SetFilePointer(hfread, lOs, 0, FILE_BEGIN) != lOs)
    {                                  // Dateioffset setzen
      err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
      break;
    }

    ReadFile(hfread, cp, lBc, &read, 0);// Daten auslesen
    // DWORD x = GetLastError();// winerror.h winbase.h

    if (read != lBc)                    // gelesen = gefordert ?
    {
      err = PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_WRONGVALUE;
      break;
    }
    os++;                              // Pointer erhöhen um 2 Byte
    bc++;
    if (SBytes == 4)                    // falls DWORDs nochmals um 2 Byte erhöhen
    {
      os++;
      bc++;
    }
    // cp += (read>>1);
    cccp =(char*) cp;                 // Ziel um Anzahl gelesener Bytes erhöhen 
    cccp += read;
    cp =(WORD*) cccp;
  }

  CloseHandle(hfread);                 // Datei schliessen

  if (err != PCO_NOERROR)
  {                                    // bei Fehler Filedatenspeicher freigeben
    if (p != NULL)
    {
      free(p);
      p = NULL;
    }
  }

  if (SDataOffset != NULL)              // Speicher freigeben
    free(SDataOffset);
  if (SByteCnt != NULL)
    free(SByteCnt);
  os = NULL;
  bc = NULL;
  cp = NULL;

  if ((!((width != 0) &&(height != 0))) ||(err))
  {
    strBild->iXRes = 0;
    strBild->iYRes = 0;
    return (PCO_ERROR_CAMWARE + PCO_ERROR_APPLICATION + PCO_ERROR_NOFILE);
  }

  strBild->iXRes = width;              // Bildgrössen übergeben
  strBild->iYRes = height;

  if (iImageType == 1)                  // 8bit BW in 16bit Rohformat eintragen
  {
    short *spl;
    char  *cpl;
    long k, l;

    spl =(short*)im;                  // Zieldaten
    cpl =(char*) p;                   // Filedaten

    for (k = 0; k < height; k++)
    {
      for (l = 0; l < width; l++)
      {
        *spl = (unsigned char)*cpl;                   // 8bit übernehmen
        *spl <<= ishift;

        spl++;                         // nächstes Pixel
        cpl++;
      }
    }
  }

  if (iImageType == 2)                  // 16bit direkt kopieren
  {
    long m;
    WORD *o;

    o = p;
    memset(im, 0, width * height *2);    // Zieldaten auf 0 setzen
    for (m = 0; m < height; m++)        // zeilenweise umkopieren, da Bytes ausgeblendet
    {                                  // werden, um 4 Byte Grenzen einzuhalten
      if(bmacformat)
      {
        for (int ix = 0; ix < width; ix++)
        {
          ush = o[ix];
          mswab((byte*)&ush, (byte*)&ush2, sizeof(ush2));
          ((WORD*)im)[ix + m * width] = ush2;
        }
      }
      else
        memcpy((WORD*)im + m * width, (const void*)o, width * 2);

      o += width;                      // nächste Zeile
    }
  }

  if ((iImageType == 3) ||(iImageType == 4))// RGB auf Rohformat filtern
  {
    long k, l;
    bool bRed = TRUE;
    bool bTog = TRUE;
    unsigned char *cpl;
    unsigned short *spl;

    cpl =(unsigned char*) p;                   // Bilddaten aus File
    spl =(unsigned short*) im;                 // Bilddaten (Ziel)
    for (k = 0; k < height; k++)
    {
      bTog = TRUE;
      if (bRed)                         // rote Zeile
      {
        for (l = 0; l < width; l++)
        {
          *spl = (unsigned char)*cpl;   // 8bit Inhalt übernehmen
          *spl <<= ishift;
          if (bTog)                     // zeigt auf roten Pixel
          {
            bTog = FALSE;
            cpl += 4;                  // nächster ist grün
          }
          else                         // zeigt auf grünen Pixel
          {
            bTog = TRUE;
            cpl += 2;                  // nächster ist rot
          }
          if (iImageType == 4)          // 4. Byte ausblenden
            cpl++;
          spl++;
        } 
        if(bTog)
          cpl++;                        // vorbereiten auf blaue Zeile, nächster ist grün
        bRed = FALSE;
      }
      else                             // blaue Zeile
      {
        for (l = 0; l < width; l++)
        {
          *spl = (unsigned char)*cpl;
          *spl <<= ishift;
          if (bTog)                     // zeigt auf grünen Pixel
          {
            bTog = FALSE;
            cpl += 4;                  // nächster ist blau
          }
          else                         // zeigt auf blauen Pixel
          {
            bTog = TRUE;
            cpl += 2;                  // nächster ist grün
          }
          if (iImageType == 4)          // 4. Byte ausblenden
            cpl++;
          spl++;
        } 
        if(!bTog)
          cpl -= 2;                     // vorbereiten auf rote Zeile, nächster ist rot
        else
          cpl--;
        bRed = TRUE;
      }
    }
  }

  if(strBild->iVersion < FILEVERSION200)
  {
    strBild->iBWMin2 = strBild->iBWMin;                   // Lut bw min
    strBild->iBWMax2 = strBild->iBWMax;                   // Lut bw max
    strBild->iBWLut2 = strBild->iBWLut;                   // Lut lin log
    strBild->iRMin2 = strBild->iRMin;                    // red min
    strBild->iRMax2 = strBild->iRMax;                    // red max
    strBild->iGMin2 = strBild->iGMin;                    // green min
    strBild->iGMax2 = strBild->iGMax;                    // green max
    strBild->iBMin2 = strBild->iBMin;                    // blue min
    strBild->iBMax2 = strBild->iBMax;                    // blue max
    strBild->iColLut2 = strBild->iColLut;                  // Lut lin log color
    strBild->bAlignUpper = FALSE;
  }

  if(strBild->iVersion < FILEVERSION300)
  {
    strBild->dGammaLut = 1.0;                 // Gamma value b/w
    strBild->dGammaLutC = 1.0;                // Gamma value color
    strBild->dGammaLut2 = 1.0;                // Gamma value b/w 2
    strBild->dGammaLutC2 = 1.0;               // Gamma value color 2
  }

  if(strBild->iVersion < FILEVERSION301)
  {
    strBild->iBitRes = 14;                    // Assume 14bit resolution
  }

  if(strBild->iVersion < FILEVERSION302)
    strBild->dSaturation = 100;

  if((iImageType == 2) && (strBild->bAlignUpper) && (iNoShifting == 0))
  {
    Shift((WORD*)im, strBild->iXRes * strBild->iYRes, FALSE, 16 - strBild->iBitRes);
  }

  if (p != NULL)
    free(p);
  return (err);
}

