Option Strict Off
Option Explicit On

Imports System.Runtime.InteropServices
Imports System.IO

Friend Class pco_camera
  Inherits System.Windows.Forms.Form
  '**********************************************************************
  'The following program is intended to illustrate
  'the calling sequence of SC2 camera SDK and
  'to provide some code segment that may be reused to
  'save development time.
  'Some comments may amend the basic description of library
  ' in SDK header files,  and the manual of "camera control commands".
  '**********************************************************************/
  ' THIS IS ONLY A DEMO TO SHOW BASIC FUNTIONALITY!

  'This sample was created mainly by a customer of pco. 
  'Thank's a lot, dear unknown customer ;-). May the correct code always be with you.
  'The code herein is provided AS IS without any kind of guarantee.

  ' Remarks by FRE:
  ' In case you're a VB crack you might improve the code to work as fast as
  ' our CamWare and send it to pco. The reward is a chocolate bar!


  Private larr(100) As Integer
  Private iArr(100) As Short


  Private Sub cmdClose_Click(ByVal eventSender As System.Object, ByVal eventArgs As System.EventArgs) Handles cmdClose.Click
    Dim i As Object

    PCO_SetRecordingState(hdriver, 0)
    PCO_FreeBuffer(hdriver, nBuf) 'essential call, otherwise you'll get a memory leak
    errorCode = PCO_CloseCamera(hdriver)
    hdriver = 0
    i = errorCode
    If errorCode = 0 Then
      Text1.Text = " Camera closed"
    Else
      Text1.Text = " Error 0x" & Hex(i) & " while closing camera"
    End If
  End Sub

  Public Function VarPtrBuf(ByVal o As Object) As Integer

    Dim GC As System.Runtime.InteropServices.GCHandle = System.Runtime.InteropServices.GCHandle.Alloc(o, System.Runtime.InteropServices.GCHandleType.Pinned)

    Dim ret As Integer = GC.AddrOfPinnedObject.ToInt32

    GC.Free()

    Return ret

  End Function

  Private Sub cmdGetbuffer_Click(ByVal eventSender As System.Object, ByVal eventArgs As System.EventArgs) Handles cmdGetbuffer.Click
    Dim BpP As Object
    Dim seg As Short
    Dim dwFrst, dwlast As Object
    Dim sbuf As Short
    Dim dwStatusDll As Integer
    Dim dwStatusDrv As Integer
    Dim loopcount As Integer
    Dim check As Integer
    Dim phelp As IntPtr
    Const mask As Integer = &H8000 'assign 0x00008000 to mask
    '^-& is essential!! Otherwise VB converts this to an int.
    '  This will give 0xFFFF8000 to mask, which is not itended.
    Dim data() As Byte
    Dim i As Integer
    Dim j As Integer
    Dim span As Integer
    Dim value As Integer
    Dim x As Integer
    Dim y As Integer

    check = 0
    seg = 1
    dwFrst = 0
    dwlast = 0
    BpP = 16
    sbuf = 0

    sbuf = 0
    errorCode = PCO_AddBufferEx(hdriver, dwFrst, dwlast, sbuf, Camera.Sensor.Resolution.xAct, Camera.Sensor.Resolution.yAct, BpP)

    loopcount = 0
    Do While Not (check) ' status of the dll must be checked or you use waitforsingleobject instead
      errorCode = PCO_GetBufferStatus(hdriver, sbuf, dwStatusDll, dwStatusDrv)
      check = Not ((dwStatusDll And mask) <> mask) ' event flag set?
      loopcount = loopcount + 1
      If loopcount > 600000 Then
        errorCode = -1
        Exit Do
      End If
    Loop

    If errorCode = 0 Then
      phelp = pwbuf

      ReDim b(iXres * iYres)
      ReDim data(iXres * iYres)


      'Looks easy, but this took some time to work...
      Marshal.Copy(pwbuf, b, 0, iXres * iYres)

      Text1.Text = "0x" & Hex(b(10)) & "   0x" & Hex(b(100)) & "   0x" & Hex(b(1000))

      span = (maxval.Value - minval.Value) * divide
      j = 0
      For i = 0 To iXres * iYres ' This loop converts from 16bit to 8bit using min and max
        value = b(i)
        If value < 0 Then ' Type cast from short to ushort? Forget it: Not with VB
          value = value * -1
          value = value + &H8000
        End If
        value = value * 255 / span
        If value > 255 Then
          value = 255
        End If
        If value < 0 Then
          value = 0
        End If

        data(i) = value
      Next

      ' Set the pixel values into the bitmap, using very fast SetPixel function ;-)
      If bmBildUsed <> 0 Then
        bmBild.Dispose()
      End If
      bmBild = New Bitmap(iXres, iYres, Imaging.PixelFormat.Format24bppRgb)
      bmBildUsed = 1
      i = 0
      For y = 0 To iYres - 1

        For x = 0 To iXres - 1
          bmBild.SetPixel(x, y, Color.FromArgb(data(i), data(i), data(i)))
          i = i + 1
        Next
      Next

      Picture.SizeMode = PictureBoxSizeMode.StretchImage ' Stretch it!
      Picture.Image = bmBild ' Set bitmap to PictureBox
      Picture.Refresh()  ' Show da image
      'bmBild.Dispose() Don't do this here, since it will be still in use by the PictureBox
      ' and guess what might happen...
    End If


  End Sub

  Private Sub cmdOpenCam_Click(ByVal eventSender As System.Object, ByVal eventArgs As System.EventArgs) Handles cmdOpenCam.Click
    Dim sizeL As Integer

    hdriver = 0
    errorCode = PCO_OpenCamera(hdriver, 0)
    If errorCode <> 0 Then
      MsgBox("Error detected: code: 0x" & Hex(Str(errorCode)))
      Return
    End If

    bmBildUsed = 0
    minval.Value = 0
    maxval.Value = 4095
    iArr(0) = Len(camDesc)

    camDesc.wSize = 436 'Len(PCO_Description)

    errorCode = PCO_GetCameraDescription(hdriver, camDesc)

    If errorCode >= 0 Then
      Text1.Text = " Camera openened, " & hdriver
    Else
      iArr(0) = MsgBox(" Error opening camera0x" & Hex(Str(errorCode)), MsgBoxStyle.Critical)
      End
    End If

    divide = 1 << (16 - camDesc.wDynResDESC)

    'errorCode = PCO_SetROI(hdriver, 673, 533, 1248, 908)
    errorCode = PCO_ResetSettingsToDefault(hdriver)

    errorCode = PCO_ArmCamera(hdriver)
    If errorCode >= 0 Then
      Text1.Text = "Camera armed"
    Else
      Text1.Text = "Camera not armed"
    End If



    'sensor
    errorCode = PCO_GetSensorFormat(hdriver, Camera.Sensor.format_Renamed)
    If errorCode < 0 Then
      Text1.Text = " Error while retrieving sensor format 0x" & Hex(Str(errorCode))
    End If

    errorCode = PCO_GetSizes(hdriver, Camera.Sensor.Resolution.xAct, Camera.Sensor.Resolution.yAct, Camera.Sensor.Resolution.xMax, Camera.Sensor.Resolution.yMax)

    If errorCode < 0 Then
      Text1.Text = " Error while retrieving sensor sizes 0x" & Hex(Str(errorCode))
    End If


    errorCode = PCO_GetROI(hdriver, Camera.Sensor.ROI.x0, Camera.Sensor.ROI.y0, Camera.Sensor.ROI.X1, Camera.Sensor.ROI.Y1)

    If errorCode < 0 Then
      Text1.Text = " Error while retrieving roi 0x" & Hex(Str(errorCode))
    End If

    iXres = Camera.Sensor.Resolution.xAct
    iYres = Camera.Sensor.Resolution.yAct

    errorCode = PCO_CamLinkSetImageParameters(hdriver, iXres, iYres) 'Mandatory for Cameralink and GigE
    ' Don't care for all other interfaces, so leave it intact here.
    If errorCode < 0 Then
      Text1.Text = " Error while setting CamLinkImageParameters" & Hex(Str(errorCode))
    End If

    sizeL = CDbl(iXres) * CDbl(iYres) * 2
    nBuf = -1
    errorCode = PCO_AllocateBuffer(hdriver, nBuf, sizeL, pwbuf, hevent)
    'pwbuf already holds the address of the buffer

    If errorCode = 0 Then
      Text1.Text = " Opened; buffer address: 0x" & Hex(pwbuf)
    Else
      Text1.Text = " Buffer allocation error 0x" & Hex(Str(errorCode))
    End If




  End Sub

  Private Sub cmdRec_Click(ByVal eventSender As System.Object, ByVal eventArgs As System.EventArgs) Handles cmdRec.Click
    errorCode = PCO_SetRecordingState(hdriver, 1)
    If errorCode = 0 Then
      Text1.Text = " Recording started"
    Else
      Text1.Text = " Error recording 0x" & Hex(Str(errorCode))
    End If

    cmdGetbuffer.Enabled = True
  End Sub

  Private Sub cmdStop_Click(ByVal eventSender As System.Object, ByVal eventArgs As System.EventArgs) Handles cmdStop.Click
    errorCode = PCO_SetRecordingState(hdriver, 0)
    If errorCode = 0 Then
      Text1.Text = " Recording stopped"
    Else
      Text1.Text = " Error recording 0x" & Hex(Str(errorCode))
    End If
  End Sub

  Private Sub minval_ValueChanged(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles minval.ValueChanged
    If minval.Value >= maxval.Value Then
      If minval.Value = 0 Then
        maxval.Value = minval.Value + 1
      Else
        minval.Value = maxval.Value - 1
      End If
    End If
  End Sub

  Private Sub maxval_ValueChanged(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles maxval.ValueChanged
    If maxval.Value <= minval.Value Then
      If maxval.Value > 65535 Then
        minval.Value = maxval.Value - 1
      Else
        maxval.Value = minval.Value + 1
      End If
    End If
  End Sub
End Class