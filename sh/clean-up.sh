cd /home/pi/picam360/picam360-capture
git remote set-url origin https://github.com/picam360/picam360-capture.git
git reset --hard & git pull
make
cd /home/pi/picam360/picam360-server
sudo rm -r userdata
mkdir userdata
git remote set-url origin https://github.com/picam360/picam360-server.git
git reset --hard & git pull

sudo rm /home/pi/.bash_history
sudo rm /home/pi/.nano_history
sudo rm /home/pi/.gdb_history
sudo rm /home/pi/.gitconfig
sudo cp /etc/rc.local.server /etc/rc.local
#need to be last because wifi connection will be disable
echo "disable wpa? [y/N]"
read WPA
case $WPA in
	y)
		sudo cp /etc/wpa_supplicant/wpa_supplicant.conf.init /etc/wpa_supplicant/wpa_supplicant.conf
		echo "wpa disabled."
		;;
	*)
		;;
esac
