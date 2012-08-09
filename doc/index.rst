.. image:: Under_cons.jpg

PCO
---

.. image:: pco.dimax-255x255.jpg
.. image:: pco.edge.jpg


Intoduction
```````````

- **TODO**


Module configuration
````````````````````
Configuration file **Lima/config.inc**

.. code-block:: sh

 ===> set these values to 1
      COMPILE_CORE=1
      COMPILE_PCO=1


See :ref:`Compilation`


OS
``
- **Win7 Professional (english) 32 bits SP1**



Required software packages
``````````````````````````

- **download links**

 - `PCO and Silicon Software download (login/pw required) <ftp://pcoag.biz/>`_
 - `VC++ download <http://www.microsoft.com/visualstudio/en-us/products/2008-editions/express>`_
 - `GSL download <http://sourceforge.net/projects/gnuwin32/files/gsl/1.8/gsl-1.8.exe/download>`_
 - `python download <http://www.python.org/download/releases/2.6.6/>`_
 - `numpy download <http://sourceforge.net/projects/numpy/files/NumPy/1.5.1/>`_
 - `PyQt download <http://www.riverbankcomputing.co.uk/software/pyqt/download>`_
 - `PyTango download <http://www.tango-controls.org/download>`_
 - `GIT download <http://code.google.com/p/msysgit/downloads/list>`_

 
- **Silicon Software Runtime 5.1.4**

.. code-block:: sh

                         58881018 DD_DCCLSISORT514WIN32_103.zip
 545a30a4aa07260072615257c5c983cd
 
                         58930007 RuntimeSetup_v5.1.4_IA32.exe
 ca22c0385fc63b92e259d28b11b65021


- **pco-sdk 1.15**

.. code-block:: sh

 ===> select: silicon softwre section dll meIV
                         11608288 SW_PCOSDKWIN_115.exe
 89619de780943569dcfc885cb0a720d8
 
                         11560659 SW_PCOSDKWIN_115.zip
 bb030a743efd0954bb60b05dbfe3c45e 
 

- **camware 3.08** 

.. code-block:: sh

 ===> select: silicon softwre section dll meIV
                         12037008 SW_CAMWAREWIN32_308.exe
 8bd64a957c57287c4c30c0ce7811343e
 
                         11993859 SW_CAMWAREWIN32_308.zip
 ecf7936f451ed3a17cabbff7aa4ec645


- **VC++ express edition 2008 with SP1** 

.. code-block:: sh

                          2728440 vcsetup.exe
 62f764849e8fcdf8bfbc342685641304


- **GSL GNU Scientific Lib** - required for Process Lib 

.. code-block:: sh

                          6476814 gsl-1.8.exe
 d0c114f842933622a156bfa757ee14c2 
 

- **Python** / **SIP** / **modules**

.. code-block:: sh

                         15227904 python-2.6.6.msi
 80b1ef074a3b86f34a2e6b454a05c8eb
 
                          2342045 numpy-1.5.1.win32-py2.6-nosse.exe
 267627e760277e5f6a74a83772a4f1d0 
 
                         27712518 PyQt-Py2.6-x86-gpl-4.8.6-1.exe
 97a9c9b88fac273e1dfb4de9d0dce4ea 
 
                          3670016 PyTango-7.1.1.win32-py2.6.msi
 dc5200e3199947a9574014537905a4b7
 

- **GIT**

.. code-block:: sh

                         14026948 Git-1.7.7.1-preview20111027.exe
 a8ab298fb7a728d41f7d787aef7dc8d1


Post installation actions
`````````````````````````
- **enable/disable PCO logs** 

.. code-block:: sh

 ===> rename file extensions (C:\ProgramData\pco): 
      .txt (disabled) / .log (enabled) ----+  
                                   camware.log   <---- created by hand
                                  PCO_CDlg.log
                                  PCO_Conv.log
                                   SC2_Cam.log


- **system variables** 
 
.. code-block:: sh

 ===> add manually the python path (it is not set by the installation program)
      PATH -> C:\Python26;

 ===> used for some utility batch files
      PATH -> C:\blissadm\bat;


- **user variables** 

.. code-block:: sh

    TANGO_HOST -> <host>:20000


- **Command prompt console (Visual Studio)** 

.. code-block:: sh

  > All Programs
    > Microsoft Visual C++ 2008 Express Edition
      > Visual Studio Tools
        > Visual Studio 2008 Command Prompt
        

- **TODO**

- After installing PCO modules :ref:`installation`

- And probably Tango server :ref:`tango_installation`



Configuration
``````````````

- **TODO**


.. _pco-esrf-pc:


PCO EDGE notes
``````````````

.. toctree::
        :maxdepth: 2

        pco_edge
