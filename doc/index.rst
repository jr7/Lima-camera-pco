PCO
-------

.. image:: Under_cons.jpg
.. image:: pco.dimax-255x255.jpg
.. image:: pco.edge.jpg


Intoduction
```````````

- **TODO**


Module configuration
````````````````````
Configuration file **Lima/config.inc** (values set to 1):

.. code-block:: sh

  COMPILE_CORE=1
  COMPILE_PCO=1


See :ref:`Compilation`


OS
``
- **Win7 Professional (english) 32 bits SP1**



Software packages required
```````````````````````````

- **download links**

 - `PCO and Silicon Software download <ftp://pcoag.biz/>`_
 - `VC++ download <http://www.microsoft.com/visualstudio/en-us/products/2008-editions/express>`_
 - `GSL download <http://sourceforge.net/projects/gnuwin32/files/gsl/1.8/gsl-1.8.exe/download>`_
 - `python download <http://www.python.org/download/releases/2.6.6/>`_
 - `numpy download <http://sourceforge.net/projects/numpy/files/NumPy/1.5.1/>`_
 - `PyQt download <http://www.riverbankcomputing.co.uk/software/pyqt/download>`_
 - `PyTango download <http://www.tango-controls.org/download>`_

- **Silicon Software Runtime 5.1.4**

.. code-block:: sh

    58881018 DD_DCCLSISORT514WIN32_103.zip
    58930007 RuntimeSetup_v5.1.4_IA32.exe

- **pco-sdk 1.15**

.. code-block:: sh

    11608288 SW_PCOSDKWIN_115.exe
             ===> select: silicon softwre section dll meIV

- **camware 3.08** 

.. code-block:: sh

    12037008 SW_CAMWAREWIN32_308.exe
             ===> select: silicon softwre section dll meIV

- **VC++ express edition 2008 with SP1** 

.. code-block:: sh

    2728440 vcsetup.exe


- **GSL GNU Scientific Lib** - required for Process Lib 

.. code-block:: sh

    6476814 gsl-1.8.exe


- **Python** / **SIP** / **modules**

.. code-block:: sh

    15227904 python-2.6.6.msi
     2342045 numpy-1.5.1.win32-py2.6-nosse.exe
    27712518 PyQt-Py2.6-x86-gpl-4.8.6-1.exe
     3670016 PyTango-7.1.1.win32-py2.6.msi


Post installation actions
`````````````````````````
- **enable/disble PCO logs** (C:\ProgramData\pco)

.. code-block:: sh

                            rename .txt (disabled) files to .log (enabled) 
          0 camware.log     <---- created by hand
        385 PCO_CDlg.log
        385 PCO_Conv.log
        382 SC2_Cam.log


- **system variables** 

.. code-block:: sh

    PATH -> C:\Python26;

- **user variables** 

.. code-block:: sh

    TANGO_HOST -> xrme:20000


- **TODO**
- After installing pco modules :ref:`installation`

- And probably Tango server :ref:`tango_installation`



Configuration
``````````````

- **TODO**



PC characteristics (used for PCO EDGE)
``````````````````````````````````````

- **RAM**

.. code-block:: sh

    24 GB (6 x DDR3-1333 Reg. ECC 4 GB module)

- **PROC**

.. code-block:: sh

        2 x Intel Xeon E5645 Six-Core CPU, 2,40GHz, 80W, Socket LGA1366, 12MB 5,86GT/sec

        CPU's :*2 x Xeon SixCore E5645 2,40Ghz 12MB 5,86GT/sec*
        Intel Xeon E5645 Six-Core CPU, 2,40GHz, 80W, Socket LGA1366, 12MB
        external cache. 5,86GT/sec QPI speed. 1333MHz memory speed (DDR3 only).
        Intel Technologies: Intel Turbo Boost , Intel Hyper-Threading
        Technology, Intel Virtualization (VT-x), Intel Trusted Execution,
        Enhanced Intel SpeedStep, Intel Demand Based Switching, Execute
        Disable Bit.

- **HD**

.. code-block:: sh

    C:
    WDC WD5003ABYX-01WERA1
    Western Digital 500 GB, 7200 RPM, SATA 2, 300 Mbps

    D:
    Adaptec RAID 5405/5405Q with 2 HD of 450 Gb -> RAID0 837 GB
    HUS156045VLS600
    Hitachi 450GB, 15,000RPM SAS / Serial Attached SCSI, 6Gbps

- **graphic card**

.. code-block:: sh

    Matrox G200eW

- **PCI slots**

.. code-block:: sh

    1* PCIe x4 (in x8 slot)
    3* PCIe x8
    1* PCIe x8 (in x16 slot)
    2* PCIe x16


- **motherboard**

.. code-block:: sh

        Motherboard Extended ATX format 13,68" x 13", (34,7cm x 33cm) (W x H);
        2 socket LGA 1366-pin. It supports processors Quad-Core Intel Xeon
        series 5500; QPI bus system (up to 6.4GT/s); *chipset Intel 5520*;

        18 socket DIMM 240 pin, support for up to 288GB memory DDR3
        1333/1066/800MHz Registered or 48GB memory DDR3 unbuffered ECC, the real
        operating ram speed depends on the processor?s model and number of
        installed ram, best performances are achieved through a triple channel
        configuration;




