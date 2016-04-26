process.chdir(__dirname);

var OpenPilot = require('./openpilot.js');
var child_process = require('child_process');
var async = require('async');
var fs = require("fs");

var LEVEL_SAMPLES = 100;
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
	op.calibrateLevel(LEVEL_SAMPLES, function(res) {
		callback(null, res);
	});
} ], function(err, result) {
	console.log(result);
});