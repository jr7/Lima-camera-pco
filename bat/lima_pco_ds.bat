@echo off

rem %comspec% /k ""C:\Program Files\Microsoft Visual Studio 9.0\VC\lima_pco_ds.bat"" install-ok

if not exist "%~dp0bin\vcvars32.bat" goto missing
call "%~dp0bin\vcvars32.bat"
goto doit


:missing
echo The specified configuration type is missing.  The tools for the
echo configuration might not be installed.
pause 
goto :eof


:doit
c:
@echo on
cd c:\blissadm\lima\pco\%1

python LimaCCDs.py pcodimax1 -ORBendPoint giop:tcp:160.103.35.39:
pause 

:eof

