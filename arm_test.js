process.chdir(__dirname);

var OpenPilot = require('./openpilot.js');
var child_process = require('child_process');
var async = require('async');
var fs = require("fs");

var op = new OpenPilot();
op.debug = process.argv[2];
async.waterfall([ function(callback) {// connect to openpilot
	op.init(function() {
		callback(null);
	});
}, function(callback) {// connect to openpilot
	op.connect(function() {
		callback(null);
	});
}, function(callback) {// start up websocket server
	op.setArm(true, function(res) {
		console.log(res);
		callback(null);
	});
}, function(callback) {// start up websocket server
	var step = 0;
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
	var flightMode = 0;
	setInterval(function() {
		step++;
		switch (step % 7) {
		case 0:
			op.getObject("FlightStatus", function(res) {
				console.log(res);
			});
			break;
		case 1:
			op.getObject("FlightTelemetryStats", function(res) {
				console.log(res);
			});
			break;
		case 2:
			op.getObject("ActuatorCommand", function(res) {
				console.log(res);
			});
			break;
		case 3:
			op.getObject("ActuatorSettings", function(res) {
				console.log(res);
			});
			break;
		case 4:
			op.getObject("ManualControlCommand", function(res) {
				console.log(res);
			});
			break;
		case 5:
			op.getObject("ManualControlSettings", function(res) {
				console.log(res);
			});
			break;
		case 6:
			function degToOne(value) {
				value = value % 180;// -180 <-> +180
				return value / 180;// -1 <-> +1
			}
			controlValue.Throttle += 1;
			var value = {};
			value.Throttle = controlValue.Throttle / 100;
			value.Roll = degToOne(controlValue.Roll);
			value.Pitch = degToOne(controlValue.Pitch);
			value.Yaw = degToOne(controlValue.Yaw);
			op.setControlValue(value, function(res) {
			});
			flightMode += 0.1;
			op.setFlightModeSwitchPosition(flightMode, function(res) {
			});
			break;
		}
	}, 1000);
	callback(null);
} ], function(err, result) {
	console.log(result);
});