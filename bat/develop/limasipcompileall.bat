call limasetdir
cd /D %GITLIMADIR%\third-party\Processlib\sip

python configure.py
nmake clean
nmake

cd /D %GITLIMADIR%
windowsSipCompilation.py --config
windowsSipCompilation.py
