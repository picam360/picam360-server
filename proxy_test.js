process.chdir(__dirname);

var os = require('os');
var agent = require('webkit-devtools-agent');
var child_process = require('child_process');
var async = require('async');
var fs = require("fs");
var express = require('express');
var moment = require("moment");

async.waterfall([ function(callback) {// exit sequence
	process.on('SIGINT', function() {
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
}, function(callback) {// start up websocket server
	console.log("request");
	
	var size = 0;
	var request = require('request');
	fs.createReadStream('/tmp/picam360.jpeg').on('error', function(err) {
		console.log(err);
	}).on('data', function(chunk) {
		size += chunk.length;
		console.log(size);
	}).on('end', function(err) {
		callback(null);
	}).pipe(request.put('http://www.ohmydigifab.com:9001/img/picam360.jpeg'));
	
} ], function(err, result) {
	console.log("done");
});