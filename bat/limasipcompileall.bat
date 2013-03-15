call limasetdir
cd /D %LIMADIR%\third-party\Processlib\sip

python configure.py
nmake clean
nmake

cd /D %LIMADIR%
windowsSipCompilation.py --config
windowsSipCompilation.py
