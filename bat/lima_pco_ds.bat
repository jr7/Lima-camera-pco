@echo off

if not exist "%~dp0bin\vcvars32.bat" goto missing
call "%~dp0bin\vcvars32.bat"
goto doit


:missing
echo The specified configuration type is missing.  The tools for the
echo configuration might not be installed.
goto :eof


:doit
c:
@echo on
cd c:\blissadm\lima\pco\%1

python LimaCCDs.py pcodimax1 -ORBendPoint giop:tcp:160.103.35.182:

