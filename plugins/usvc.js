module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create usv plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;

		var SerialPort = require('serialport');

		var COM_PORT = "/dev/ttyACM0";
		var DELIMITER = '\r\n';

		var sp = null;
		var max_way_point_num = 0;

		async.waterfall([
			function(callback) {
				var sp_callback = null;
				sp = new SerialPort(COM_PORT, {
					baudRate : 9600
				});
				var parser = sp.pipe(new SerialPort.parsers.Readline({
					delimiter : DELIMITER
				}));
				function sp_send(str, _callback) {
					sp.write(new Buffer(str + DELIMITER), function() {
						sp_callback = _callback;
						sp.drain();
					});
				}
				sp.on("open", function() {
					setInterval(function() {
						sp_send("get_max_way_point_num", function(ret) {
							console.log("get_max_way_point_num:" + ret);
							max_way_point_num = ret;
							function get_way_point_walkthrough(idx) {
								sp_send("get_way_point " + idx, function(ret) {
									console.log("get_way_point " + idx + ":"
										+ ret);
									if (idx + 1 < max_way_point_num) {
										get_way_point_walkthrough(idx + 1);
									}
								});
							};
							get_way_point_walkthrough(0);
						});
					}, 1000);
				});
				parser.on("data", function(data) {
					var ret = data.toString('utf-8', 0, data.length);
					if (ret.startsWith("INFO:")) {
						console.log(ret);
					} else if (ret.startsWith("ERROR:")) {
						console.log(ret);
					} else if (sp_callback != null) {
						sp_callback(ret);
					}
				});
				callback(null);
			}, function(callback) {
				callback(null);
			}], function(err, result) {
		});

		var plugin = {
			name : "usvc",
			command_handler : function(cmd) {
				var split = cmd.split(' ');
				cmd = split[0].split('.')[1];
				switch (cmd) {
					case "dummy" :
						break;
				}
			}
		};
		return plugin;
	}
};