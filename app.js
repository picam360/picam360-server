process.chdir(__dirname);

var os = require('os');
var disk = require('diskusage');
var OpenPilot = require('./openpilot.js');
var child_process = require('child_process');
var async = require('async');
var fs = require("fs");
var express = require('express');
//var piblaster = require('pi-blaster.js');
var moment = require("moment");
var sprintf = require('sprintf-js').sprintf;


function watchFile(filepath, oncreate, ondelete) {
  var
    fs = require('fs'),
    path = require('path'),
    filedir = path.dirname(filepath),
    filename = path.basename(filepath);

  fs.watch(filedir, function(event, who) {
    if (event === 'rename' && who === filename) {
      if (fs.existsSync(filepath)) {
        if(oncreate) oncreate();
      } else {
    	if(ondelete) ondelete();
      }
    }
  });
}


var recording = false;
var framecount = 0;
var frame_duration = 0;
var last_frame_date = null;
var memoryusage_start = 0;
var GC_THRESH = 16*1024*1024;//16MB

var capture_if;
var capture_process;
var op = new OpenPilot();
async.waterfall([ function(callback) {// exit sequence
	process.on('SIGINT', function() {
		console.log("led shutting down");
		//piblaster.setPwm(40, 0);
		//piblaster.setPwm(41, 0);
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
	//console.log("led starting up");
	//piblaster.setPwm(40, 0);
	//piblaster.setPwm(41, 0);
	callback(null);
}, function(callback) {// capture startup
	console.log("camera starting up");
	child_process.exec('sudo killall picam360-capture.bin', function() {
		//var cmd = 'bash ../picam360-capture/lunch.sh -w 2048 -h 2048 -f 5 -W 800 -H 800 -r -n 2 -c MJPEG -S';
		var cmd = 'bash ../picam360-capture/lunch.sh -w 2048 -h 2048 -f 5 -W 800 -H 800 -c MJPEG -S';
		capture_process = child_process.exec(cmd, function() {	
		});
		capture_if = capture_process.stdin;
	});
	setTimeout(function() {
		capture_if = fs.createWriteStream('../picam360-capture/cmd');
		callback(null);
	}, 3000);
}, function(callback) {//cam
	console.log("camera instance");
	
	var disk_free = 0;
	setInterval(function() {
		disk.check('/tmp', function(err, info) {
			disk_free = info.available;
		});
	}, 1000);
	setInterval(function() {
		if(global.gc && os.freemem() < GC_THRESH) {
			console.log("gc : free=" + os.freemem() + " usage=" + process.memoryUsage().rss);
			console.log("disk_free=" + disk_free);
			global.gc();
		}
	}, 100);
	callback(null);
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
		Pitch : 0,
		// -180 - 180
		Yaw : 0,
		// -180 - 180
		Roll : 0,
	};
	var app = require('express')();
	var http = require('http').Server(app);
	var io = require("socket.io").listen(http);
	
	app.get('/img/*.jpeg', function(req, res){
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
					'Content-Type' : 'image/jpeg',
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
		Pitch : 0,
		// -180 - 180
		Yaw : 0,
		// -180 - 180
		Roll : 0,
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

	io.sockets.on("connection", function(socket) {

		socket.on("connected", function() {
		});

		socket.on("set_view_orientation", function(orientation) {
			var id = 0;
			var cmd = sprintf('set_view_orientation %d=%f,%f,%f\n', id, orientation.Pitch, orientation.Yaw, orientation.Roll);
			//console.log(cmd);
			capture_if.write(cmd);
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
			//piblaster.setPwm(40, value / 100.0);
		});

		socket.on("setBottomLedValue", function(value) {
			//piblaster.setPwm(41, value / 100.0);
		});

		socket.on("snap", function(callback) {
			var filename = moment().format('YYYYMMDD_hhmmss') + '.jpeg';
			var cmd = 'snap -E -W 3072 -H 1536 -o /tmp/' + filename + '\n';
			capture_if.write(cmd);
			console.log(cmd);
			watchFile('/tmp/' + filename, function() {
				console.log(filename + ' saved.');
				var cmd = 'mv' + ' /tmp/' + filename + ' userdata/' + filename;
				console.log(cmd);
				child_process.exec(cmd, function(){
					callback(filename);
				});
			});
		});

		socket.on("startRecord", function(duration) {
			if(recording)
				return;
			duration = (duration==null)?0:duration;
			capture_if.write('set_duration ' + duration + '\n');
			capture_if.write('start_record /tmp/movie.h264\n');
			console.log("camera recording start duration=" + duration);
			recording = true;
			frame_duration = duration;
			last_frame_date = null;
		});

		socket.on("stopRecord", function(callback) {
			recording = false;
			capture_if.write('stop_record\n');
			console.log("camera recording stop");
			var filename = moment().format('YYYYMMDD_hhmmss') + '.mp4';
			var ffmpeg_cmd = 'ffmpeg -y -r 5 -i /tmp/movie.h264 -c:v copy userdata/' + filename;
			var delh264_cmd = 'rm /tmp/movie.h264';
			var spatialmedia_cmd = 'python submodules/spatial-media/spatialmedia -i userdata/' + filename + ' /tmp/vr.mp4';
			//var cmd = ffmpeg_cmd + ' && ' + delh264_cmd + ' && ' + spatialmedia_cmd;
			var cmd = ffmpeg_cmd + ' ; ' + delh264_cmd;
			console.log(cmd);
			child_process.exec(cmd, function(){
				callback(filename);
				});
		});

		socket.on("isRecording", function(callback) {
			if(recording) {
				callback(true);
			} else {
				fs.exists('/tmp/movie.h264', function(exists) { 
					callback(exists);
				}); 
			}			
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