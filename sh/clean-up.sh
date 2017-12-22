cd /home/pi/picam360/picam360-capture
git remote set-url origin https://github.com/picam360/picam360-capture.git
git reset --hard & git pull
make
cd /home/pi/picam360/picam360-driver
git remote set-url origin https://github.com/picam360/picam360-driver.git
git reset --hard & git pull
make
cd /home/pi/picam360/picam360-server
sudo rm -r userdata
mkdir userdata
git remote set-url origin https://github.com/picam360/picam360-server.git
git reset --hard & git pull

echo "rm history files"
sudo rm /home/pi/.*_history
sudo rm /home/pi/.gitconfig
sudo rm /home/pi/.xsession-errors*
sudo rm /home/pi/picam360/picam360-capture/.picam360_history
sudo rm /home/pi/picam360/picam360-driver/.picam360_history

echo "reset rc.local"
sudo cp /home/pi/picam360/picam360-capture/setup/rc.local /etc/rc.local

echo "reset interfaces"
sudo cp /home/pi/picam360/picam360-capture/setup/interfaces /etc/network/interfaces

echo "reset config.json"
cp /home/pi/picam360/picam360-capture/config.json.tmp /home/pi/picam360/picam360-capture/config.json
cp /home/pi/picam360/picam360-driver/config.json.tmp /home/pi/picam360/picam360-driver/config.json
cp /home/pi/picam360/picam360-server/config.json.tmp /home/pi/picam360/picam360-server/config.json
cp /home/pi/picam360/picam360-server/www/config.json.tmp /home/pi/picam360/picam360-server/www/config.json

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
