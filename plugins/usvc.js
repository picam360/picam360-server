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

		var sp_send = null;
		var status_arrived = false;
		var way_points = [];
		var latitude = 0;
		var longitude = 0;
		var next_way_point_distance = 0;
		var next_way_point_idx = 0;
		var next_way_point_direction = 0;
		var heading = 0;
		var rudder_pwm = 0;
		var rudder_mode = 0;
		var adc_values = [];

		async.waterfall([function(callback) {
			var sp_callback = null;
			var sp_send_list = [];
			var sp = new SerialPort(COM_PORT, {
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
			sp_send = function(str, _callback, user_data) {
				sp_send_list.push([str, _callback, user_data]);
				if (sp_send_list.length == 1) {
					_sp_send();
				}
			};
			sp.on("open", function() {
				setInterval(function() {
					sp_send("get_max_way_point_num", function(ret) {
						var new_way_points = new Array(parseInt(ret[1]));
						for (var i = 0; i < new_way_points.length; i++) {
							sp_send("get_way_point " + i, function(ret, idx) {
								var fary = [];
								for (var i = 1; i < ret.length; i++) {
									fary[i - 1] = parseFloat(ret[i]);
								}
								new_way_points[idx] = fary;
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
				var params = ret.split(",");
				switch (params[0]) {
					case "$INFO" :
						console.log(ret);
						break;
					case "$ERROR" :
						console.log(ret);
						break;
					case "$STATUS" :
						latitude = parseFloat(params[1]);
						longitude = parseFloat(params[2]);
						next_way_point_distance = parseFloat(params[3]);
						next_way_point_idx = parseFloat(params[4]);
						next_way_point_direction = parseFloat(params[5]);
						heading = parseFloat(params[6]);
						rudder_mode = parseFloat(params[7]);
						rudder_pwm = parseFloat(params[8]);
						status_arrived = true;
						break;
					case "$ADC" :
						adc_values = [];
						for (var i = 1; i < params.length; i++) {
							adc_values.push(parseInt(params[i]));
						}
						break;
					case "$RET" :
						if (sp_callback != null) {
							sp_callback(params);
						}
						break;
				}
			});
			callback(null);
		}, function(callback) {
			plugin_host.add_status(PLUGIN_NAME + ".status", function() {
				if (status_arrived) {
					status_arrived = false;
					var status = {
						way_points : way_points,
						latitude : latitude,
						longitude : longitude,
						heading : heading,
						next_way_point_distance : next_way_point_distance,
						next_way_point_idx : next_way_point_idx,
						next_way_point_direction : next_way_point_direction,
						rudder_mode : rudder_mode,
						rudder_pwm : rudder_pwm,
					};
					return {
						succeeded : true,
						value : JSON.stringify(status)
					};
				} else {
					return {
						succeeded : false,
						value : null
					};
				}
			});
			callback(null);
		}], function(err, result) {
		});

		var plugin = {
			name : PLUGIN_NAME,
			client_id : null,
			init_options : function(options) {
				plugin.client_id = options.aws_iot_client_id || null;
			},
			command_handler : function(cmd) {
				var split = cmd.split(' ');
				cmd = split[0].split('.')[1];
				switch (cmd) {
					case "set_rudder_mode" :
						var v = parseInt(split[1]);
						sp_send("set_rudder_mode " + v, function(ret) {
							// console.log(ret);
						});
						break;
					case "set_rudder_pwm" :
						var v = parseInt(split[1]);
						// console.log(split);
						sp_send("set_rudder_pwm " + v, function(ret) {
							// console.log(ret);
						});
						break;
				}
			},
			aws_iot_handler : function(device) {
				if (!plugin.client_id || (latitude == 0 && longitude == 0)) {
					return;
				}
				var record = {
					"device_id" : plugin.client_id,
					"timestamp" : parseInt(Date.now() / 1000),
					"latitude" : latitude,
					"longitude" : longitude,
					"heading" : heading,
					"adc_values" : adc_values,
				};
				var message = JSON.stringify(record);
				console.log("Publish: " + message);
				device.publish('asv_data', message);
			}
		};
		return plugin;
	}
};