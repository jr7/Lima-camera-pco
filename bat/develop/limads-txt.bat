call limasetdir
cd /D %GITLIMADIR%
cd install
dir *.py
python LimaCCDs.py pco > 1.txt
