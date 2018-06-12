module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create usv plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;
		var SerialPort = require('serialport');

		var PLUGIN_NAME = "usvc";
		var COM_PORT = "/dev/ttyACM0";
		var DELIMITER = '\r\n';
		var TIMEOUT_MS = 5000;

		var sp = null;
		var way_points = [];

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
						var new_way_points = new Array(parseInt(ret));
						for (var i = 0; i < new_way_points.length; i++) {
							sp_send("get_way_point " + i, function(ret, idx) {
								new_way_points[idx] = ret;
								if (idx == new_way_points.length - 1) {
									way_points = new_way_points;
								}
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
			plugin_host.add_status(PLUGIN_NAME + ".way_points", function() {
				var str = JSON.stringify(way_points);
				return {
					succeeded : true,
					value : str
				};
			});
			callback(null);
		}], function(err, result) {
		});

		var plugin = {
			name : PLUGIN_NAME,
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