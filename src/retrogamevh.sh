#!/bin/bash

if [ $(id -u) -ne 0 ]; then
	echo "Installer must be run as root."
	echo "Try 'sudo bash $0'"
	exit 1
fi

# Add udev rule (will overwrite if present)
echo "SUBSYSTEM==\"input\", ATTRS{name}==\"retrogame\", \
     ENV{ID_INPUT_KEYBOARD}=\"1\"" > /etc/udev/rules.d/10-retrogame.rules

# Start on boot
grep retrogame /etc/rc.local >/dev/null
if [ $? -eq 0 ]; then
	# retrogame already in rc.local, but make sure correct:
	sed -i "s/^.*retrogame.*$/\/home\/pi\/Documents\/Skripte\/retrogamevh \/home\/pi\/Documents\/Skripte\/retrogamevh.cfg \&/g" /etc/rc.local >/dev/null
else
	# Insert retrogame into rc.local before final 'exit 0'
	sed -i "s/^exit 0/\/usr\/local\/bin\/retrogamevh \&\\nexit 0/g" /etc/rc.local >/dev/null
fi

./retrogamevh /home/pi/Documents/Skripte/retrogamevh.cfg
