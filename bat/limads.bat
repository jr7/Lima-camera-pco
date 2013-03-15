call limasetdir
cd /D %LIMADIR%
mkdir install
cd install
dir *.py
python LimaCCDs.py pco
