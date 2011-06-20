{
  This sample shows only the basic handling of the pco.camera SDK functions.
  The most difficult function to handle PCO_AllocateBuffer is also part of
  this sample. All other functions and structures can be derived from the
  source code below. Tested with Borland Delphi 2005 (free edition).
}

unit Delphi_Test;


interface


uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls;
function PCO_OpenCamera(hcam: Pointer; wcamnum: WORD): integer; stdcall; external 'sc2_cam.dll';
function PCO_CloseCamera(hcam: Pointer): integer; stdcall; external 'sc2_cam.dll';
function PCO_AllocateBuffer(hcam: Pointer;
                            sBufNr: psmallint;
                            size: cardinal;
                            wBuf: Pointer;
                            hEvent: Pointer): integer; stdcall; external 'sc2_cam.dll';
function PCO_FreeBuffer(hcam: Pointer; wBuf: WORD): integer; stdcall; external 'sc2_cam.dll';
function PCO_GetCameraDescription(hcam: Pointer; strDescription: Pointer):integer;stdcall; external 'sc2_cam.dll';

type
  TForm1 = class(TForm)
    Button1: TButton;
    Button2: TButton;
    Button3: TButton;
    Edit1: TEdit;
    procedure OnShow(Sender: TObject);
    procedure Acquire(Sender: TObject);
    procedure Quit(Sender: TObject);
    procedure Init(Sender: TObject);
  private
    { Private-Deklarationen }
  public
    { Public-Deklarationen }
  end;

type
  PCO_Description = record
  wSize                 :WORD;         // Sizeof this struct
  wSensorTypeDESC       :WORD;         // Sensor type
  wSensorSubTypeDESC    :WORD;         // Sensor subtype
  wMaxHorzResStdDESC    :WORD;         // Maxmimum horz. resolution in std.mode
  wMaxVertResStdDESC    :WORD;         // Maxmimum vert. resolution in std.mode
  wMaxHorzResExtDESC    :WORD;         // Maxmimum horz. resolution in ext.mode
  wMaxVertResExtDESC    :WORD;         // Maxmimum vert. resolution in ext.mode
  wDynResDESC           :WORD;         // Dynamic resolution of ADC in bit
  wMaxBinHorzDESC       :WORD;         // Maxmimum horz. binning
  wBinHorzSteppingDESC  :WORD;         // Horz. bin. stepping (0:bin, 1:lin)
  wMaxBinVertDESC       :WORD;         // Maxmimum vert. binning
  wBinVertSteppingDESC  :WORD;         // Vert. bin. stepping (0:bin, 1:lin)
  wRoiHorStepsDESC      :WORD;         // Minimum granularity of ROI in pixels
  wRoiVertStepsDESC     :WORD;         // Minimum granularity of ROI in pixels
  wNumADCsDESC          :WORD;         // Number of ADCs in system
  ZZwAlignDummy1        :WORD;
  dwPixelRateDESC: array [1..4] of DWORD;// Possible pixelrate in Hz
  ZZdwDummypr: array [1..20] of DWORD;
  wConvFactDESC: array [1..4] of WORD; // Possible conversion factor in e/cnt
  ZZdwDummycv: array [1..20] of WORD;
  wIRDESC               :WORD;         // IR enhancment possibility
  ZZwAlignDummy2        :WORD;
  dwMinDelayDESC        :DWORD;        // Minimum delay time in ns
  dwMaxDelayDESC        :DWORD;        // Maximum delay time in ms
  dwMinDelayStepDESC    :DWORD;        // Minimum stepping of delay time in ns
  dwMinExposureDESC     :DWORD;        // Minimum exposure time in ns
  dwMaxExposureDESC     :DWORD;        // Maximum exposure time in ms
  dwMinExposureStepDESC :DWORD;        // Minimum stepping of exposure time in ns
  dwMinDelayIRDESC      :DWORD;        // Minimum delay time in ns
  dwMaxDelayIRDESC      :DWORD;        // Maximum delay time in ms
  dwMinExposureIRDESC   :DWORD;        // Minimum exposure time in ns
  dwMaxExposureIRDESC   :DWORD;        // Maximum exposure time in ms
  wTimeTableDESC        :WORD;         // Timetable for exp/del possibility
  wDoubleImageDESC      :WORD;         // Double image mode possibility
  sMinCoolSetDESC       :SHORT;        // Minimum value for cooling
  sMaxCoolSetDESC       :SHORT;        // Maximum value for cooling
  sDefaultCoolSetDESC   :SHORT;        // Default value for cooling
  wPowerDownModeDESC    :WORD;         // Power down mode possibility
  wOffsetRegulationDESC :WORD;         // Offset regulation possibility
  wColorPatternDESC     :WORD;         // Color pattern of color chip
                                       // four nibbles (0,1,2,3) in word
                                       //  -----------------
                                       //  | 3 | 2 | 1 | 0 |
                                       //  -----------------
                                       //
                                       // describe row,column  2,2 2,1 1,2 1,1
                                       //
                                       //   column1 column2
                                       //  -----------------
                                       //  |       |       |
                                       //  |   0   |   1   |   row1
                                       //  |       |       |
                                       //  -----------------
                                       //  |       |       |
                                       //  |   2   |   3   |   row2
                                       //  |       |       |
                                       //  -----------------
                                       //
  wPatternTypeDESC      :WORD;         // Pattern type of color chip
                                       // 0: Bayer pattern RGB
                                       // 1: Bayer pattern CMY
  wDummy1               :WORD;         // former DSNU correction mode
  wDummy2               :WORD;         //
  dwGeneralCapsDESC1    :DWORD;        // General capabilities:
                                       // Bit 0: Noisefilter available
                                       // Bit 1: Hotpixelfilter available
                                       // Bit 2: Hotpixel works only with noisefilter
                                       // Bit 3: Timestamp ASCII only available (Timestamp mode 3 enabled)
  dwReservedDESC: array [1..7] of DWORD;// 32bit dummy
  ZZdwDummy: array [1..40] of DWORD;
  end;

var
  Form1: TForm1;
  err:   Integer;
  hCamera: Pointer;
  strDesc : PCO_Description;

implementation

{$R *.dfm}

procedure TForm1.Quit(Sender: TObject);
begin

  if hCamera <> nil then
    err := PCO_CloseCamera(hCamera);

  close;
end;

procedure TForm1.Init(Sender: TObject);
var
  buf : string[100];
begin
  hCamera := nil; //Mandatory!!!

  err := PCO_OpenCamera(@hCamera, 0);
  if err = 0 then
  begin
    strDesc.wSize := sizeof(strDesc);
    err := PCO_GetCameraDescription(hCamera, @strDesc);

    buf := Format('Init done max. X: %d max. Y: %d', [strDesc.wMaxHorzResStdDESC,
                  strDesc.wMaxVertResStdDESC]);
    Edit1.Text := buf;
    Button2.Enabled := true;
  end
  else
  begin
    buf := Format('Init error: 0x%x', [err]);
    Edit1.Text := buf;
  end;
end;

procedure TForm1.Acquire(Sender: TObject);
var
  hEvent: Pointer;
  sBufNr: smallint;
  pwBuffer: Pointer;
  width: cardinal;
  height: cardinal;
  buf : string[100];
  err2 : integer;
begin
  width := strDesc.wMaxHorzResStdDESC;
  height := strDesc.wMaxVertResStdDESC;
  sBufNr := -1; //Mandatory!!!
  pwbuffer := nil; //This allocates a buffer inside the sdk dll.
  err := PCO_AllocateBuffer(hCamera, @sBufNr, width * height * 2, @pwbuffer, @hEvent);
  err2 := PCO_FreeBuffer(hCamera, sbufNr);
  buf := Format('Buffer. Alloc Err %d, Free Err %d', [err, err2]);
  Edit1.Text := buf;
end;

procedure TForm1.OnShow(Sender: TObject);
begin
    Button2.Enabled := false;
end;

end.
