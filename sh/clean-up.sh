cd /home/pi/picam360/picam360-capture
git remote set-url origin https://github.com/picam360/picam360-capture.git
git reset --hard & git pull
make
cd /home/pi/picam360/picam360-software
git remote set-url origin https://github.com/picam360/picam360-software.git
git reset --hard & git pull

sudo rm /home/pi/.bash_history
#need to be last because wifi connection will be disable
sudo cp /etc/wpa_supplicant/wpa_supplicant.conf.init /etc/wpa_supplicant/wpa_supplicant.conf
