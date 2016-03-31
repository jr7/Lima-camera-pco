@echo off
rem =======================================================================
rem the install directory (from blissinstaller) must be in c:\blissadm\pco\
rem   it can be renamed for tests or different versions. 
rem   the actual name must be used as 1st parameter
rem the 2nd parameter is the personal name of the server
rem the 3rd parameter is the IP of the pc (used for ORBendPoint)
rem =======================================================================

c:
@echo on
cd c:\blissadm\pco\%1

python LimaCCDs.py %2 -ORBendPoint giop:tcp:%3:
pause 
