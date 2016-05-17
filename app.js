process.chdir(__dirname);

var os = require('os');
var agent = require('webkit-devtools-agent');
var OpenPilot = require('./openpilot.js');
var child_process = require('child_process');
var async = require('async');
var fs = require("fs");
var express = require('express');
var picam360 = require("picam360");
var cam1;
var cam2;
var piblaster = require('pi-blaster.js');
var moment = require("moment");

var recording = false;
var framecount = 0;
var needToCapture = true;
var frame_duration = 0;
var last_frame_date = null;
var memoryusage_start = 0;
var GC_THRESH = 16*1024*1024;//16MB

var op = new OpenPilot();
async.waterfall([ function(callback) {// exit sequence
	process.on('SIGINT', function() {
		console.log("led shutting down");
		piblaster.setPwm(40, 0);
		piblaster.setPwm(41, 0);
		// console.log("camera shutting down");
		// cam1.stop();
		console.log("exit process done");
		process.exit();
	});
	process.on('SIGUSR2', function () {
		if (agent.server) {
	 		agent.stop();
		} else {
		    agent.start({
		        port: 9999,
		        bind_to: '192.168.3.103',
		        ipc_port: 3333,
		        verbose: true
		    });
		}
	});
	callback(null);
}, function(callback) {// led startup
	console.log("led starting up");
	piblaster.setPwm(40, 0);
	piblaster.setPwm(41, 0);
	callback(null);
}, function(callback) {// camera startup
	console.log("camera starting up");
	child_process.exec('sudo killall uv4l', function() {
		child_process.exec('sh sh/start-uv4l.sh "--width=720 --height=720 --sharpness=100 --output-buffers=1 --framerate=10 --statistics=off --text-overlay=off"', function() {	
			setTimeout(function() {
				cam1 = new picam360.Camera("/dev/video0", 1440, 720);
				cam1.start();
				cam1.setRotation(0, 0, 0);
				setInterval(function() {
					if (recording) {
						nowTime = new Date();
						var duration = (last_frame_date == null) ? frame_duration : nowTime.getTime() - last_frame_date.getTime();//milisec
						if(duration >= frame_duration) {
							cam1.capture(function(){
								cam1.addFrame(cam2);
								framecount++;
								if (framecount == 300) {
									recording = false;
									framecount = 0;
									cam1.stopRecord();
									console.log("camera recording stop");
								}
							});
							last_frame_date = nowTime;
							return;
						}
					}
					if(needToCapture) {
						cam1.capture();
						needToCapture = false;
					}
					if(global.gc && os.freemem() < GC_THRESH) {
						console.log("gc at : " + process.memoryUsage().rss);
						global.gc();
					}
				}, 100);
				//cam2 = new v4l2camera.Camera("/dev/video1");
				//cam2.start();
				//cam2.capture(function loop2() {
				//	cam2.capture(loop2);
				//});
				callback(null);
			}, 3000);
		});
	});
}, function(callback) {// connect to openpilot
	//op.init(function() {
	//	callback(null);
	//});
	callback(null);
}, function(callback) {// connect to openpilot
	//op.connect(function() {
	//	callback(null);
	//});
	callback(null);
}, function(callback) {// start up websocket server
	console.log("websocket server starting up");

	var veicle_attitude = {
		// -180 - 180
		Roll : 0,
		// -180 - 180
		Pitch : 0,
		// -180 - 180
		Yaw : 0
	};
	var app = require('express')();
	var http = require('http').Server(app);
	var io = require("socket.io").listen(http);
	
	app.get('/img/picam360.jpeg', function(req, res){
		fs.readFile('/tmp/vr.jpeg', function(err, data) {
			if (err) {
				res.writeHead(404);
				res.end();
				console.log("404");
			} else {
				res.writeHead(200, {
					'Content-Type' : 'image/jpeg',
					'Content-Length' : data.length,
					'Cache-Control' : 'private, no-cache, no-store, must-revalidate',
					'Expires' : '-1',
					'Pragma' : 'no-cache',
				});
				res.end(data);
				console.log("200");
			}
			cam1.toJpeg('/tmp/_vr.jpeg');
			child_process.exec('mv /tmp/_vr.jpeg /tmp/vr.jpeg');
			needToCapture = true;
		});
	});
	
	app.get('/img/*.mp4', function(req, res){
		var url = req.url.split("?")[0];
		var query = req.url.split("?")[1];
		var filepath = 'userdata/' + url.split("/")[2];
		console.log(url);
		console.log(query);
		console.log(filepath);
		fs.readFile(filepath, function(err, data) {
			if (err) {
				res.writeHead(404);
				res.end();
				console.log("404");
			} else {
				res.writeHead(200, {
					'Content-Type' : 'video/mp4',
					'Content-Length' : data.length,
					'Cache-Control' : 'private, no-cache, no-store, must-revalidate',
					'Expires' : '-1',
					'Pragma' : 'no-cache',
				});
				res.end(data);
				console.log("200");
			}
		});
	});	
	
	app.use(express.static('www'));//this need be set after all dynamic files

	var yaw_offset = 0;
	var controlValue = {
		// 0% - 100%
		Throttle : 0,
		// -180 - 180
		Roll : 0,
		// -180 - 180
		Pitch : 0,
		// -180 - 180
		Yaw : 0
	};
	var accel_factor = 0.02;

	function degToOne(value) {
		value = value % 180;// -180 <-> +180
		return value / 180;// -1 <-> +1
	}
	function degToRad(value) {
		return Math.PI * value / 180;
	}
	function radToDeg(value) {
		return 180 * value / Math.PI;
	}
	function clone(src) {
		var dst = {}
		for ( var k in src) {
			dst[k] = src[k];
		}
		return dst;
	}

	var lastThrottle = 0;
	var controlValueUpdating = false;
	op.onAttitudeStateChanged(function(attitude) {
		veicle_attitude = attitude;
		cam1.setRotation(-veicle_attitude.Roll, -veicle_attitude.Pitch, -veicle_attitude.Yaw);
		if (controlValue.Throttle > 0 || lastThrottle != 0) {
			if (controlValueUpdating) {
				return;
			}
			controlValueUpdating = true;

			var value = {
				// 0 - 1
				Throttle : 0,
				// -1 - 1
				Roll : 0,
				// -1 - 1
				Pitch : 0,
				// -1 - 1
				Yaw : 0
			};

			var length = Math.sqrt(controlValue.Roll * controlValue.Roll + controlValue.Pitch * controlValue.Pitch);
			var angle = radToDeg(Math.atan2(controlValue.Roll, controlValue.Pitch));
			var roll = 0;
			var pitch = 0;
			if (attitude.Roll < -90 || attitude.Roll > 90) {
				angle -= attitude.Yaw - yaw_offset;
				roll = length * -Math.sin(degToRad(angle));
				pitch = length * Math.cos(degToRad(angle));
			} else {
				angle += attitude.Yaw - yaw_offset;
				roll = length * Math.sin(degToRad(angle));
				pitch = length * Math.cos(degToRad(angle));
			}

			value.Throttle = controlValue.Throttle / 100;
			value.Roll = degToOne(roll);
			value.Pitch = degToOne(pitch);
			value.Yaw = degToOne(controlValue.Yaw);
			op.setControlValue(value, function(res) {
				controlValueUpdating = false;
			});
			lastThrottle = value.Throttle;
		}
	});

	io.sockets.on("connection", function(socket) {

		socket.on("connected", function() {
		});

		socket.on("ping", function(time) {
			op.getObject("ActuatorCommand", function(actuatorCommand) {
				op.getObject("FlightStatus", function(flightStatus) {
					op.getObject("FlightTelemetryStats", function(flightTelemetryStats) {
						socket.emit("pong", {
							ActuatorCommand : actuatorCommand,
							FlightStatus : flightStatus,
							FlightTelemetryStats : flightTelemetryStats
						});
					});
				});
			});
		});

		socket.on("accelerate_throttle", function(value, callback) {
			controlValue.Throttle += accel_factor * value;
			if (controlValue.Throttle < 0.0) {
				controlValue.Throttle = 0.0;
			} else if (controlValue.Throttle > 1.0) {
				controlValue.Throttle = 1.0;
			}
			op.setThrust(controlValue.Throttle, function() {
				callback();
			});
		});

		socket.on("connectFcm", function(callback) {
			op.connect(function() {
				callback();
			});
		});

		socket.on("setArm", function(bln, callback) {
			controlValue.Throttle = 0.0;
			op.setArm(bln, function(res) {
				callback(res);
			});
		});

		socket.on("calibrateLevel", function(callback) {
			var LEVEL_SAMPLES = 100;
			op.calibrateLevel(LEVEL_SAMPLES, function(res) {
				op.getObject("AttitudeState", function(attitude) {
					yaw_offset = attitude.Yaw;
					console.log("yaw_offset : " + yaw_offset);
					callback(res);
				});
			});
		});

		socket.on("setControlValue", function(value, callback) {
			controlValue = value;
			callback();
		});

		socket.on("getControlValue", function(callback) {
			callback(controlValue);
		});

		socket.on("setActuatorValue", function(value, callback) {
			op.setActuator(value, function(res) {
				callback(res);
			});
		});

		socket.on("getActuatorValue", function(callback) {
			op.getObject("ActuatorCommand", function(obj) {
				callback(obj);
			}, true);
		});

		socket.on("getAttitude", function(callback) {
			op.getObject("AttitudeState", function(obj) {
				var dst = clone(obj);
				dst.Yaw -= yaw_offset;
				callback(dst);
			}, true);
		});

		socket.on("setUdpProxyEnabled", function(value) {
			op.setUdpProxyEnabled(value);
		});

		socket.on("setUpperLedValue", function(value) {
			piblaster.setPwm(40, value / 100.0);
		});

		socket.on("setBottomLedValue", function(value) {
			piblaster.setPwm(41, value / 100.0);
		});

		socket.on("startRecord", function(duration) {
			if(recording)
				return;
			duration = (duration==null)?0:duration;
			cam1.startRecord('/tmp/movie.h264', 16000);
			console.log("camera recording start duration=" + duration);
			recording = true;
			frame_duration = duration;
			last_frame_date = null;
		});

		socket.on("stopRecord", function(callback) {
			recording = false;
			cam1.stopRecord();
			console.log("camera recording stop");
			var filename = moment().format('YYYYMMDD_hhmmss') + '.mp4';
			var ffmpeg_cmd = 'ffmpeg -y -r 5 -i /tmp/movie.h264 -c:v copy userdata/' + filename;
			var delh264_cmd = 'rm /tmp/movie.h264';
			var spatialmedia_cmd = 'python submodules/spatial-media/spatialmedia -i userdata/' + filename + ' /tmp/vr.mp4';
			//var cmd = ffmpeg_cmd + ' && ' + delh264_cmd + ' && ' + spatialmedia_cmd;
			var cmd = ffmpeg_cmd;
			console.log(cmd);
			child_process.exec(cmd, function(){
				callback(filename);
				});
		});

		socket.on("isRecording", function(callback) {
			callback(recording);
		});

		socket.on("disconnect", function() {
		});

	});
	
	http.listen(9001, function(){
	  console.log('listening on *:9001');
	});
	
	callback(null);
}, function(callback) {// start up websocket server
	console.log("poling start!");
	var step = 0;
	setInterval(function() {
		step++;
		switch (step % 2) {
		case 0:
			op.getObject("FlightStatus", function(res) {
				// console.log(res);
			}, true);
			break;
		case 1:
			op.getObject("FlightTelemetryStats", function(res) {
				// console.log(res);
			}, true);
			break;
		}
	}, 1000);
	callback(null);
} ], function(err, result) {
});