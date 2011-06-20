<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()> Partial Class pco_camera
#Region "Windows Form Designer generated code "
	<System.Diagnostics.DebuggerNonUserCode()> Public Sub New()
		MyBase.New()
		'This call is required by the Windows Form Designer.
		InitializeComponent()
	End Sub
	'Form overrides dispose to clean up the component list.
	<System.Diagnostics.DebuggerNonUserCode()> Protected Overloads Overrides Sub Dispose(ByVal Disposing As Boolean)
		If Disposing Then
			If Not components Is Nothing Then
				components.Dispose()
			End If
		End If
		MyBase.Dispose(Disposing)
	End Sub
	'Required by the Windows Form Designer
	Private components As System.ComponentModel.IContainer
	Public ToolTip1 As System.Windows.Forms.ToolTip
	Public WithEvents Text1 As System.Windows.Forms.TextBox
	Public WithEvents cmdOpenCam As System.Windows.Forms.Button
	Public WithEvents cmdClose As System.Windows.Forms.Button
	Public WithEvents cmdStop As System.Windows.Forms.Button
	Public WithEvents cmdGetbuffer As System.Windows.Forms.Button
	Public WithEvents cmdRec As System.Windows.Forms.Button
	'NOTE: The following procedure is required by the Windows Form Designer
	'It can be modified using the Windows Form Designer.
	'Do not modify it using the code editor.
	<System.Diagnostics.DebuggerStepThrough()> Private Sub InitializeComponent()
    Me.components = New System.ComponentModel.Container
    Dim resources As System.ComponentModel.ComponentResourceManager = New System.ComponentModel.ComponentResourceManager(GetType(pco_camera))
    Me.ToolTip1 = New System.Windows.Forms.ToolTip(Me.components)
    Me.Text1 = New System.Windows.Forms.TextBox
    Me.cmdOpenCam = New System.Windows.Forms.Button
    Me.cmdClose = New System.Windows.Forms.Button
    Me.cmdStop = New System.Windows.Forms.Button
    Me.cmdGetbuffer = New System.Windows.Forms.Button
    Me.cmdRec = New System.Windows.Forms.Button
    Me.Picture = New System.Windows.Forms.PictureBox
    Me.minval = New System.Windows.Forms.NumericUpDown
    Me.maxval = New System.Windows.Forms.NumericUpDown
    Me.Label1 = New System.Windows.Forms.Label
    Me.Label2 = New System.Windows.Forms.Label
    CType(Me.Picture, System.ComponentModel.ISupportInitialize).BeginInit()
    CType(Me.minval, System.ComponentModel.ISupportInitialize).BeginInit()
    CType(Me.maxval, System.ComponentModel.ISupportInitialize).BeginInit()
    Me.SuspendLayout()
    '
    'Text1
    '
    Me.Text1.AcceptsReturn = True
    Me.Text1.BackColor = System.Drawing.SystemColors.Window
    Me.Text1.Cursor = System.Windows.Forms.Cursors.IBeam
    Me.Text1.ForeColor = System.Drawing.SystemColors.WindowText
    Me.Text1.Location = New System.Drawing.Point(8, 239)
    Me.Text1.MaxLength = 0
    Me.Text1.Name = "Text1"
    Me.Text1.RightToLeft = System.Windows.Forms.RightToLeft.No
    Me.Text1.Size = New System.Drawing.Size(193, 20)
    Me.Text1.TabIndex = 5
    Me.Text1.Text = "Click 'Open'"
    '
    'cmdOpenCam
    '
    Me.cmdOpenCam.BackColor = System.Drawing.SystemColors.Control
    Me.cmdOpenCam.Cursor = System.Windows.Forms.Cursors.Default
    Me.cmdOpenCam.ForeColor = System.Drawing.SystemColors.ControlText
    Me.cmdOpenCam.Location = New System.Drawing.Point(8, 8)
    Me.cmdOpenCam.Name = "cmdOpenCam"
    Me.cmdOpenCam.RightToLeft = System.Windows.Forms.RightToLeft.No
    Me.cmdOpenCam.Size = New System.Drawing.Size(193, 33)
    Me.cmdOpenCam.TabIndex = 4
    Me.cmdOpenCam.Text = "Open camera (1)"
    Me.cmdOpenCam.UseVisualStyleBackColor = False
    '
    'cmdClose
    '
    Me.cmdClose.BackColor = System.Drawing.SystemColors.Control
    Me.cmdClose.Cursor = System.Windows.Forms.Cursors.Default
    Me.cmdClose.ForeColor = System.Drawing.SystemColors.ControlText
    Me.cmdClose.Location = New System.Drawing.Point(8, 176)
    Me.cmdClose.Name = "cmdClose"
    Me.cmdClose.RightToLeft = System.Windows.Forms.RightToLeft.No
    Me.cmdClose.Size = New System.Drawing.Size(193, 25)
    Me.cmdClose.TabIndex = 3
    Me.cmdClose.Text = "Close camera (5)"
    Me.cmdClose.UseVisualStyleBackColor = False
    '
    'cmdStop
    '
    Me.cmdStop.BackColor = System.Drawing.SystemColors.Control
    Me.cmdStop.Cursor = System.Windows.Forms.Cursors.Default
    Me.cmdStop.ForeColor = System.Drawing.SystemColors.ControlText
    Me.cmdStop.Location = New System.Drawing.Point(8, 136)
    Me.cmdStop.Name = "cmdStop"
    Me.cmdStop.RightToLeft = System.Windows.Forms.RightToLeft.No
    Me.cmdStop.Size = New System.Drawing.Size(193, 25)
    Me.cmdStop.TabIndex = 2
    Me.cmdStop.Text = "Stop recording (4)"
    Me.cmdStop.UseVisualStyleBackColor = False
    '
    'cmdGetbuffer
    '
    Me.cmdGetbuffer.BackColor = System.Drawing.SystemColors.Control
    Me.cmdGetbuffer.Cursor = System.Windows.Forms.Cursors.Default
    Me.cmdGetbuffer.Enabled = False
    Me.cmdGetbuffer.ForeColor = System.Drawing.SystemColors.ControlText
    Me.cmdGetbuffer.Location = New System.Drawing.Point(8, 96)
    Me.cmdGetbuffer.Name = "cmdGetbuffer"
    Me.cmdGetbuffer.RightToLeft = System.Windows.Forms.RightToLeft.No
    Me.cmdGetbuffer.Size = New System.Drawing.Size(193, 25)
    Me.cmdGetbuffer.TabIndex = 1
    Me.cmdGetbuffer.Text = "Get image (3)"
    Me.cmdGetbuffer.UseVisualStyleBackColor = False
    '
    'cmdRec
    '
    Me.cmdRec.BackColor = System.Drawing.SystemColors.Control
    Me.cmdRec.Cursor = System.Windows.Forms.Cursors.Default
    Me.cmdRec.ForeColor = System.Drawing.SystemColors.ControlText
    Me.cmdRec.Location = New System.Drawing.Point(8, 56)
    Me.cmdRec.Name = "cmdRec"
    Me.cmdRec.RightToLeft = System.Windows.Forms.RightToLeft.No
    Me.cmdRec.Size = New System.Drawing.Size(193, 25)
    Me.cmdRec.TabIndex = 0
    Me.cmdRec.Text = "Start recording (2)"
    Me.cmdRec.UseVisualStyleBackColor = False
    '
    'Picture
    '
    Me.Picture.Location = New System.Drawing.Point(208, 13)
    Me.Picture.Name = "Picture"
    Me.Picture.Size = New System.Drawing.Size(449, 435)
    Me.Picture.TabIndex = 6
    Me.Picture.TabStop = False
    '
    'minval
    '
    Me.minval.Location = New System.Drawing.Point(48, 265)
    Me.minval.Maximum = New Decimal(New Integer() {65535, 0, 0, 0})
    Me.minval.Name = "minval"
    Me.minval.Size = New System.Drawing.Size(120, 20)
    Me.minval.TabIndex = 7
    '
    'maxval
    '
    Me.maxval.Location = New System.Drawing.Point(48, 291)
    Me.maxval.Maximum = New Decimal(New Integer() {65535, 0, 0, 0})
    Me.maxval.Name = "maxval"
    Me.maxval.Size = New System.Drawing.Size(120, 20)
    Me.maxval.TabIndex = 8
    '
    'Label1
    '
    Me.Label1.AutoSize = True
    Me.Label1.Location = New System.Drawing.Point(18, 267)
    Me.Label1.Name = "Label1"
    Me.Label1.Size = New System.Drawing.Size(24, 13)
    Me.Label1.TabIndex = 9
    Me.Label1.Text = "Min"
    '
    'Label2
    '
    Me.Label2.AutoSize = True
    Me.Label2.Location = New System.Drawing.Point(18, 293)
    Me.Label2.Name = "Label2"
    Me.Label2.Size = New System.Drawing.Size(27, 13)
    Me.Label2.TabIndex = 10
    Me.Label2.Text = "Max"
    '
    'pco_camera
    '
    Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 13.0!)
    Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
    Me.BackColor = System.Drawing.SystemColors.Control
    Me.ClientSize = New System.Drawing.Size(669, 460)
    Me.Controls.Add(Me.Label2)
    Me.Controls.Add(Me.Label1)
    Me.Controls.Add(Me.maxval)
    Me.Controls.Add(Me.minval)
    Me.Controls.Add(Me.Picture)
    Me.Controls.Add(Me.Text1)
    Me.Controls.Add(Me.cmdOpenCam)
    Me.Controls.Add(Me.cmdClose)
    Me.Controls.Add(Me.cmdStop)
    Me.Controls.Add(Me.cmdGetbuffer)
    Me.Controls.Add(Me.cmdRec)
    Me.Cursor = System.Windows.Forms.Cursors.Default
    Me.Icon = CType(resources.GetObject("$this.Icon"), System.Drawing.Icon)
    Me.Location = New System.Drawing.Point(4, 30)
    Me.Name = "pco_camera"
    Me.RightToLeft = System.Windows.Forms.RightToLeft.No
    Me.Text = "pco vb sample"
    CType(Me.Picture, System.ComponentModel.ISupportInitialize).EndInit()
    CType(Me.minval, System.ComponentModel.ISupportInitialize).EndInit()
    CType(Me.maxval, System.ComponentModel.ISupportInitialize).EndInit()
    Me.ResumeLayout(False)
    Me.PerformLayout()

  End Sub
  Friend WithEvents Picture As System.Windows.Forms.PictureBox
  Friend WithEvents minval As System.Windows.Forms.NumericUpDown
  Friend WithEvents maxval As System.Windows.Forms.NumericUpDown
  Friend WithEvents Label1 As System.Windows.Forms.Label
  Friend WithEvents Label2 As System.Windows.Forms.Label
#End Region 
End Class