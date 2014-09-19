@echo off

rem %comspec% /k ""C:\Program Files\Microsoft Visual Studio 9.0\VC\lima_pco_ds.bat"" install pco4k 160.103.35.39
rem %comspec% /k ""C:\Program Files\Microsoft Visual Studio 9.0\VC\lima_pco_ds.bat"" install-ok

rem if not exist "%~dp0bin\vcvars32.bat" goto missing
rem call "%~dp0bin\vcvars32.bat"
rem goto doit


@echo on

c:
cd "C:\Program Files\Microsoft Visual Studio 9.0\VC\" 

if not exist ".\bin\vcvars32.bat" goto missing
call ".\bin\vcvars32.bat"
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

python LimaCCDs.py %2 -ORBendPoint giop:tcp:%3:
pause 

:eof


