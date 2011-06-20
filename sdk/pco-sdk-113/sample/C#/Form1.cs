/////////////////////////////////////////////////////////////////////////////////////////////////////
// This sample does not show how to get images and how to play with the camRAM, etc.
// It is only intended to show how to access the pco.camera SDK while using C#.
/////////////////////////////////////////////////////////////////////////////////////////////////////

// extern "C" { #include c:\programme\digital camera toolbox\pco.camera.sdk\include\SC2_CamExport.h }
// Including is not possible with C#, so we have to rebuild all structures defined in SC2_CamExport.h.
// Additionally you can not create arrays inside a structure. We can work around this, by creating
// each member as an own entity.

using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;
using System.Data;
using System.Runtime;

using System.Runtime.InteropServices;

namespace CSharpDemo
{
  /// <summary>
  /// </summary>
  public unsafe class Form1 : System.Windows.Forms.Form
  {
    private System.Windows.Forms.Button button1;
    private System.Windows.Forms.Button button2;
    private System.Windows.Forms.Button button3;
    private Label label1;
    /// <summary>
    /// </summary>
    private System.ComponentModel.Container components = null;

    public Form1()
    {
      //
      //
      InitializeComponent();

      //
      // TODO: -
      //
    }

    /// <summary>
    /// </summary>
    protected override void Dispose( bool disposing )
    {
      if( disposing )
      {
        if (components != null) 
        {
          components.Dispose();
        }
      }
      base.Dispose( disposing );
    }

    #region Vom Windows Form-Designer generierter Code
    /// <summary>
    /// </summary>
    private void InitializeComponent()
    {
      System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
      this.button1 = new System.Windows.Forms.Button();
      this.button2 = new System.Windows.Forms.Button();
      this.button3 = new System.Windows.Forms.Button();
      this.label1 = new System.Windows.Forms.Label();
      this.SuspendLayout();
      // 
      // button1
      // 
      this.button1.Location = new System.Drawing.Point(82, 49);
      this.button1.Name = "button1";
      this.button1.Size = new System.Drawing.Size(136, 32);
      this.button1.TabIndex = 0;
      this.button1.Text = "PCO_OpenCamera";
      this.button1.Click += new System.EventHandler(this.button1_Click);
      // 
      // button2
      // 
      this.button2.Location = new System.Drawing.Point(82, 97);
      this.button2.Name = "button2";
      this.button2.Size = new System.Drawing.Size(136, 32);
      this.button2.TabIndex = 1;
      this.button2.Text = "PCO_GetDescription";
      this.button2.Click += new System.EventHandler(this.OnGetDescription);
      // 
      // button3
      // 
      this.button3.Location = new System.Drawing.Point(82, 145);
      this.button3.Name = "button3";
      this.button3.Size = new System.Drawing.Size(136, 32);
      this.button3.TabIndex = 2;
      this.button3.Text = "PCO_CloseCamera";
      this.button3.Click += new System.EventHandler(this.OnCloseCamera);
      // 
      // label1
      // 
      this.label1.AutoSize = true;
      this.label1.Location = new System.Drawing.Point(12, 23);
      this.label1.Name = "label1";
      this.label1.Size = new System.Drawing.Size(285, 13);
      this.label1.TabIndex = 3;
      this.label1.Text = "Place a breakpoint inside the code and see what happens.";
      // 
      // Form1
      // 
      this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
      this.ClientSize = new System.Drawing.Size(311, 191);
      this.Controls.Add(this.label1);
      this.Controls.Add(this.button3);
      this.Controls.Add(this.button2);
      this.Controls.Add(this.button1);
      this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
      this.Name = "Form1";
      this.Text = "C#_Demo";
      this.ResumeLayout(false);
      this.PerformLayout();

    }
    #endregion

    /// <summary>
    /// </summary>
    [STAThread]
    static void Main() 
    {
      Application.Run(new Form1());
    }

    int cameraHandle = 0;//IntPtr.Zero;
    PCO_Description pcoDescr;
    BWLUT *bwlut;

    private unsafe void button1_Click(object sender, System.EventArgs e)
    {
      int err = 0;
      UInt16 boardNum = 0;

      err = LibWrapper.PCO_OpenCamera(ref cameraHandle, boardNum);
            
    }

    private unsafe void OnGetDescription(object sender, System.EventArgs e)
    {
      pcoDescr.wSize = (ushort) sizeof(PCO_Description);
      int err = 0;
      short bufnr;
      int size;
      int evhandle;
      UInt16 *buf;
      UInt32 dwStatusDll, dwStatusDrv;

      err = LibWrapper.PCO_GetCameraDescription(cameraHandle, ref pcoDescr);
      String errortext = "";
      PCO_GetErrorTextClass geterrtxt = new PCO_GetErrorTextClass();

      geterrtxt.PCO_GetErrorText((uint)err, ref errortext);

      if (err != 0)
        return;


      size = pcoDescr.wMaxHorzResStdDESC * pcoDescr.wMaxVertResStdDESC * 2;
      evhandle = 0;
      bufnr = -1;
      buf = null;
      dwStatusDll = 0;
      dwStatusDrv = 0;
      err = LibWrapper.PCO_AllocateBuffer(cameraHandle, ref bufnr, size, ref buf, ref evhandle);

      err = LibWrapper.PCO_ArmCamera(cameraHandle);

      err = LibWrapper.PCO_CamLinkSetImageParameters(cameraHandle, pcoDescr.wMaxHorzResStdDESC, pcoDescr.wMaxVertResStdDESC);
      //Mandatory for Cameralink and GigE. Don't care for all other interfaces, so leave it intact here.


      err = LibWrapper.PCO_SetRecordingState(cameraHandle, 1);

      err = LibWrapper.PCO_AddBuffer(cameraHandle, 0, 0, bufnr);

      do
      {
        err = LibWrapper.PCO_GetBufferStatus(cameraHandle, bufnr, ref dwStatusDll, ref dwStatusDrv);
      } while ((dwStatusDll & 0x8000) == 0);

      err = LibWrapper.PCO_SetRecordingState(cameraHandle, 0);

      err = LibWrapper.PCO_FreeBuffer(cameraHandle, bufnr);
    }

    private void OnCloseCamera(object sender, System.EventArgs e)
    {
      int err = 0;

      err = LibWrapper.PCO_CloseCamera(cameraHandle);
    }
  }
}

[StructLayout(LayoutKind.Sequential)]
public unsafe struct PCO_Description
{
  public  ushort        wSize;                   // Sizeof this struct
  public  ushort        wSensorTypeDESC;         // Sensor type
  public  ushort        wSensorSubTypeDESC;      // Sensor subtype
  public  ushort        wMaxHorzResStdDESC;      // Maxmimum horz. resolution in std.mode
  public  ushort        wMaxVertResStdDESC;      // Maxmimum vert. resolution in std.mode
  public  ushort        wMaxHorzResExtDESC;      // Maxmimum horz. resolution in ext.mode
  public  ushort        wMaxVertResExtDESC;      // Maxmimum vert. resolution in ext.mode
  public  ushort        wDynResDESC;             // Dynamic resolution of ADC in bit
  public  ushort        wMaxBinHorzDESC;         // Maxmimum horz. binning
  public  ushort        wBinHorzSteppingDESC;    // Horz. bin. stepping (0:bin, 1:lin)
  public  ushort        wMaxBinVertDESC;         // Maxmimum vert. binning
  public  ushort        wBinVertSteppingDESC;    // Vert. bin. stepping (0:bin, 1:lin)
  public  ushort        wRoiHorStepsDESC;        // Minimum granularity of ROI in pixels
  public  ushort        wRoiVertStepsDESC;       // Minimum granularity of ROI in pixels
  public  ushort        wNumADCsDESC;            // Number of ADCs in system
  public  ushort        ZZwAlignDummy1;
  public  uint          dwPixelRateDESC1;      // Possible pixelrate in Hz
  public  uint          dwPixelRateDESC2;      // Possible pixelrate in Hz
  public  uint          dwPixelRateDESC3;      // Possible pixelrate in Hz
  public  uint          dwPixelRateDESC4;      // Possible pixelrate in Hz
  public  uint          ZZdwDummypr1, ZZdwDummypr2, ZZdwDummypr3, ZZdwDummypr4, ZZdwDummypr5;
  public  uint          ZZdwDummypr21, ZZdwDummypr22, ZZdwDummypr23, ZZdwDummypr24, ZZdwDummypr25;
  public  uint          ZZdwDummypr31, ZZdwDummypr32, ZZdwDummypr33, ZZdwDummypr34, ZZdwDummypr35;
  public  uint          ZZdwDummypr41, ZZdwDummypr42, ZZdwDummypr43, ZZdwDummypr44, ZZdwDummypr45;
  public  ushort        wConvFactDESC1;        // Possible conversion factor in e/cnt
  public  ushort        wConvFactDESC2;        // Possible conversion factor in e/cnt
  public  ushort        wConvFactDESC3;        // Possible conversion factor in e/cnt
  public  ushort        wConvFactDESC4;        // Possible conversion factor in e/cnt
  public  ushort        ZZdwDummycv1, ZZdwDummycv2, ZZdwDummycv3, ZZdwDummycv4, ZZdwDummycv5;
  public  ushort        ZZdwDummycv21, ZZdwDummycv22, ZZdwDummycv23, ZZdwDummycv24, ZZdwDummycv25;
  public  ushort        ZZdwDummycv31, ZZdwDummycv32, ZZdwDummycv33, ZZdwDummycv34, ZZdwDummycv35;
  public  ushort        ZZdwDummycv41, ZZdwDummycv42, ZZdwDummycv43, ZZdwDummycv44, ZZdwDummycv45;
  public  ushort        wIRDESC;                 // IR enhancment possibility
  public  ushort        ZZwAlignDummy2;
  public  uint          dwMinDelayDESC;          // Minimum delay time in ns
  public  uint          dwMaxDelayDESC;          // Maximum delay time in ms
  public  uint          dwMinDelayStepDESC;      // Minimum stepping of delay time in ns
  public  uint          dwMinExposureDESC;       // Minimum exposure time in ns
  public  uint          dwMaxExposureDESC;       // Maximum exposure time in ms
  public  uint          dwMinExposureStepDESC;   // Minimum stepping of exposure time in ns
  public  uint          dwMinDelayIRDESC;        // Minimum delay time in ns
  public  uint          dwMaxDelayIRDESC;        // Maximum delay time in ms
  public  uint          dwMinExposureIRDESC;     // Minimum exposure time in ns
  public  uint          dwMaxExposureIRDESC;     // Maximum exposure time in ms
  public  ushort        wTimeTableDESC;          // Timetable for exp/del possibility
  public  ushort        wDoubleImageDESC;        // Double image mode possibility
  public  short         sMinCoolSetDESC;         // Minimum value for cooling
  public  short         sMaxCoolSetDESC;         // Maximum value for cooling
  public  short         sDefaultCoolSetDESC;     // Default value for cooling
  public  ushort        wPowerDownModeDESC;      // Power down mode possibility 
  public  ushort        wOffsetRegulationDESC;   // Offset regulation possibility
  public  ushort        wColorPatternDESC;       // Color pattern of color chip
          // four nibbles (0,1,2,3) in ushort 
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
  public  ushort        wPatternTypeDESC;        // Pattern type of color chip
          // 0: Bayer pattern RGB
          // 1: Bayer pattern CMY
  public  ushort        wDSNUCorrectionModeDESC; // DSNU correction mode possibility
  public  ushort        ZZwAlignDummy3;          //
  public  uint     dwReservedDESC1, dwReservedDESC2, dwReservedDESC3, dwReservedDESC4;
  public  uint     dwReservedDESC5, dwReservedDESC6, dwReservedDESC7, dwReservedDESC8;
  public  uint     ZZdwDummy1, ZZdwDummy2, ZZdwDummy3, ZZdwDummy4, ZZdwDummy5;
  public  uint     ZZdwDummy21, ZZdwDummy22, ZZdwDummy23, ZZdwDummy24, ZZdwDummy25;
  public  uint     ZZdwDummy31, ZZdwDummy32, ZZdwDummy33, ZZdwDummy34, ZZdwDummy35;
  public  uint     ZZdwDummy41, ZZdwDummy42, ZZdwDummy43, ZZdwDummy44, ZZdwDummy45;
  public  uint     ZZdwDummy51, ZZdwDummy52, ZZdwDummy53, ZZdwDummy54, ZZdwDummy55;
  public  uint     ZZdwDummy61, ZZdwDummy62, ZZdwDummy63, ZZdwDummy64, ZZdwDummy65;
  public  uint     ZZdwDummy71, ZZdwDummy72, ZZdwDummy73, ZZdwDummy74, ZZdwDummy75;
  public  uint     ZZdwDummy81, ZZdwDummy82, ZZdwDummy83, ZZdwDummy84, ZZdwDummy85;
} ;

[StructLayout(LayoutKind.Sequential)]
public unsafe struct BWLUT
{
  public ushort    wSize;                     // size of bw_lut array
  public int       min;
  public int       max;
  public int       typ;
  public int       mid;
  public int       min_out;
  public int       max_out;
  public int       size;
  public void       *ptr;
  public int       bitpix; // eingefügt: 30.06.2003 - FRE
  public int       align; // align 0: rechtsbündig(wie SC, PCCam), 1: linksbündig(SC2); eingefügt: 30.06.2003 - FRE
  public double    dgamma; // eingefügt: 30.06.2003 - FRE
  public uint      ZZdwDummycv1, ZZdwDummycv2, ZZdwDummycv3, ZZdwDummycv4, ZZdwDummycv5;
  public uint      ZZdwDummycv6, ZZdwDummycv7, ZZdwDummycv8, ZZdwDummycv9, ZZdwDummycv10;
  public uint      ZZdwDummycv11, ZZdwDummycv12, ZZdwDummycv13, ZZdwDummycv14, ZZdwDummycv15;
  public uint      ZZdwDummycv16, ZZdwDummycv17, ZZdwDummycv18, ZZdwDummycv19, ZZdwDummycv20;
  public uint      ZZdwDummycv21, ZZdwDummycv22, ZZdwDummycv23, ZZdwDummycv24, ZZdwDummycv25;
  public uint      ZZdwDummycv26, ZZdwDummycv27, ZZdwDummycv28, ZZdwDummycv29, ZZdwDummycv30;
  public uint      ZZdwDummycv31, ZZdwDummycv32, ZZdwDummycv33, ZZdwDummycv34, ZZdwDummycv35;
  public uint      ZZdwDummycv36, ZZdwDummycv37, ZZdwDummycv38, ZZdwDummycv39, ZZdwDummycv40;
  public uint      ZZdwDummycv41, ZZdwDummycv42, ZZdwDummycv43, ZZdwDummycv44, ZZdwDummycv45;
  public uint      ZZdwDummycv46, ZZdwDummycv47, ZZdwDummycv48, ZZdwDummycv49, ZZdwDummycv50;
  //DWORD     dwzzDummy0[50];            // for future use: set to zero
  public uint      dwFlags;                   // Flags: Bit0(1)->align upper, Bit1(2)->Set parameter active
                                       //        Bit2(4)->do sRGB,     Bit3(8)->Inverted table
                                       //        Bit4(16)->16bit out
  public uint      ZZdwDummy1cv1, ZZdwDummy1cv2, ZZdwDummy1cv3, ZZdwDummy1cv4, ZZdwDummy1cv5;
  public uint      ZZdwDummy1cv6, ZZdwDummy1cv7, ZZdwDummy1cv8, ZZdwDummy1cv9, ZZdwDummy1cv10;
  public uint      ZZdwDummy1cv11, ZZdwDummy1cv12, ZZdwDummy1cv13, ZZdwDummy1cv14, ZZdwDummy1cv15;
  public uint      ZZdwDummy1cv16, ZZdwDummy1cv17, ZZdwDummy1cv18, ZZdwDummy1cv19, ZZdwDummy1cv20;
  public uint      ZZdwDummy1cv21, ZZdwDummy1cv22, ZZdwDummy1cv23, ZZdwDummy1cv24, ZZdwDummy1cv25;
  public uint      ZZdwDummy1cv26, ZZdwDummy1cv27, ZZdwDummy1cv28, ZZdwDummy1cv29, ZZdwDummy1cv30;
  public uint      ZZdwDummy1cv31, ZZdwDummy1cv32, ZZdwDummy1cv33, ZZdwDummy1cv34, ZZdwDummy1cv35;
  public uint      ZZdwDummy1cv36, ZZdwDummy1cv37, ZZdwDummy1cv38, ZZdwDummy1cv39, ZZdwDummy1cv40;
  public uint      ZZdwDummy1cv41, ZZdwDummy1cv42, ZZdwDummy1cv43, ZZdwDummy1cv44, ZZdwDummy1cv45;
  public uint      ZZdwDummy1cv46, ZZdwDummy1cv47, ZZdwDummy1cv48, ZZdwDummy1cv49, ZZdwDummy1cv50;
  public uint      ZZdwDummy1cv51, ZZdwDummy1cv52, ZZdwDummy1cv53, ZZdwDummy1cv54, ZZdwDummy1cv55;
  public uint      ZZdwDummy1cv56, ZZdwDummy1cv57, ZZdwDummy1cv58, ZZdwDummy1cv59, ZZdwDummy1cv60;
  public uint      ZZdwDummy1cv61, ZZdwDummy1cv62, ZZdwDummy1cv63, ZZdwDummy1cv64, ZZdwDummy1cv65;
  //DWORD     dwzzDummy1[65];            // for future use: set to zero
};

unsafe class LibWrapper
{
  [DllImport("sc2_cam.dll", EntryPoint="PCO_OpenCamera",
     ExactSpelling=false,CallingConvention=CallingConvention.Cdecl)]
  public static extern int PCO_OpenCamera(ref int pHandle, UInt16 wCamNum );

  [DllImport("sc2_cam.dll", EntryPoint="PCO_CloseCamera",
     ExactSpelling=false,CallingConvention=CallingConvention.Cdecl)]
  public static extern int PCO_CloseCamera(int pHandle);

  [DllImport("sc2_cam.dll", EntryPoint="PCO_GetCameraDescription",
     ExactSpelling=false,CallingConvention=CallingConvention.Cdecl)]
  public static extern int PCO_GetCameraDescription(int pHandle, ref PCO_Description strDescription);

  [DllImport("sc2_cam.dll", EntryPoint="PCO_AllocateBuffer",
     ExactSpelling=false,CallingConvention=CallingConvention.Cdecl)]
  public static extern int PCO_AllocateBuffer(int pHandle, ref short sBufNr, int size, ref UInt16 *wBuf, ref int hEvent);
  //HANDLE ph,SHORT* sBufNr,DWORD size,WORD** wBuf,HANDLE *hEvent

  [DllImport("sc2_cam.dll", EntryPoint = "PCO_FreeBuffer",
     ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
  public static extern int PCO_FreeBuffer(int pHandle, short sBufNr);

  [DllImport("sc2_cam.dll", EntryPoint = "PCO_ArmCamera",
     ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
  public static extern int PCO_ArmCamera(int pHandle);

  [DllImport("sc2_cam.dll", EntryPoint = "PCO_CamLinkSetImageParameters",
     ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
  public static extern int PCO_CamLinkSetImageParameters(int pHandle, UInt16 wXRes, UInt16 wYRes);

  [DllImport("sc2_cam.dll", EntryPoint = "PCO_SetRecordingState",
     ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
  public static extern int PCO_SetRecordingState(int pHandle, UInt16 wRecState);

  [DllImport("sc2_cam.dll", EntryPoint = "PCO_AddBuffer",
     ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
  public static extern int PCO_AddBuffer(int pHandle, UInt32 dwFirstImage, UInt32 dwLastImage , short sBufNr);

  [DllImport("sc2_cam.dll", EntryPoint = "PCO_GetBufferStatus",
     ExactSpelling = false, CallingConvention = CallingConvention.Cdecl)]
  public static extern int PCO_GetBufferStatus(int pHandle, short sBufNr, ref UInt32 dwStatusDll, ref UInt32 dwStatusDrv);
};
