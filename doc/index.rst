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



