module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create usv plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;

		var SerialPort = require('serialport');

		var COM_PORT = "/dev/ttyACM0";
		var DELIMITER = '\r\n';
		var TIMEOUT_MS = 5000;

		var sp = null;
		var max_way_point_num = 0;

		async.waterfall([function(callback) {
			var sp_callback = null;
			var sp_send_list = [];
			sp = new SerialPort(COM_PORT, {
				baudRate : 9600
			});
			var parser = sp.pipe(new SerialPort.parsers.Readline({
				delimiter : DELIMITER
			}));
			function _sp_send() {
				var params = sp_send_list[0];
				sp.write(new Buffer(params[0] + DELIMITER), function() {
					var timer = setTimeout(_sp_send, TIMEOUT_MS);
					sp_callback = function(ret) {
						clearTimeout(timer);
						if (params[1]) {
							params[1](ret, params[2]);
						}
						sp_send_list.shift();
						if (sp_send_list.length != 0) {
							_sp_send();
						}
					};
					sp.drain();
				});
			}
			function sp_send(str, _callback, user_data) {
				sp_send_list.push([str, _callback, user_data]);
				if (sp_send_list.length == 1) {
					_sp_send();
				}
			}
			sp.on("open", function() {
				setInterval(function() {
					sp_send("get_max_way_point_num", function(ret) {
						console.log("get_max_way_point_num:" + ret);
						max_way_point_num = ret;
						for (var i = 0; i < max_way_point_num; i++) {
							sp_send("get_way_point " + i, function(ret, idx) {
								console.log("get_way_point " + idx + ":" + ret);
							}, i);
						};
					});
				}, 5000);
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