@echo off

rem ================================================================================================================================
rem ================================================================================================================================

rem shortcut / tarjet:

rem %comspec% /k ""C:\Program Files\Microsoft Visual Studio 9.0\VC\lima_pco_ds_vc.bat"" install-2014-10-24_121113 160.103.36.67 edge

rem shortcut / start in : "C:\Program Files\Microsoft Visual Studio 9.0\"

rem this file MUST be located in C:\Program Files\Microsoft Visual Studio 9.0\VC\

rem the directory install (%1) must be located in c:\blissadm\

rem ================================================================================================================================
rem ================================================================================================================================


if not exist "%~dp0bin\vcvars32.bat" goto missing
call "%~dp0bin\vcvars32.bat"
goto doit


:missing
echo The specified configuration type is missing.  The tools for the
echo configuration might not be installed.
pause 
goto :eof

rem python LimaCCDs.py pcodimax1 -ORBendPoint giop:tcp:160.103.35.39:


:doit
c:
@echo on
rem cd c:\blissadm\lima\pco\%1
cd c:\blissadm\%1

python LimaCCDs.py %3 -ORBendPoint giop:tcp:%2:

pause

:eof

