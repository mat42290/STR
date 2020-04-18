current=$PWD
cd software/simulateur/dist/Debug/GNU-Linux
gnome-terminal -- ./simulateur
cd $current/software/monitor/monitor
gnome-terminal -- ./monitor
cd $current/software/raspberry/superviseur-robot/dist/Debug__PC_/GNU-Linux
gnome-terminal -- sudo ./superviseur-robot



