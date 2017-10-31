sudo killall node
/home/pi/.nodebrew/current/bin/node /home/pi/.nodebrew/current/bin/forever start --minUptime 10000 --spinSleepTime 10000 -c '/home/pi/.nodebrew/current/bin/node --expose_gc' /home/pi/picam360/picam360-server/app.js 1> /tmp/app.log 2> /tmp/app.err
