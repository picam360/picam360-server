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
		var waypoints = [];
		var latitude = 0;
		var longitude = 0;
		var next_waypoint_distance = 0;
		var next_waypoint_idx = 0;
		var next_waypoint_direction = 0;
		var heading = 0;
		var rudder_pwm = 0;
		var skrew_pwm = 0;
		var mode_flag = [0, 0];
		var adc_values = [];
		var history = [];

		var waypoints_required = false;
		var history_required = false;
		var rudder_pwm_candidate = null;
		var skrew_pwm_candidate = null;

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
				sp_send("get max_waypoint_num", function(ret) {
					var new_waypoints = new Array(parseInt(ret[1]));
					for (var i = 0; i < new_waypoints.length; i++) {
						sp_send("get waypoint " + i, function(ret, idx) {
							var fary = {
								latitude : parseFloat(ret[1]),
								longitude : parseFloat(ret[2]),
								allowable_error : parseFloat(ret[3]),
							};
							new_waypoints[idx] = fary;
							if (idx == new_waypoints.length - 1) {
								waypoints = new_waypoints;
							}
						}, i);
					};
				});

				setInterval(function() {
					if (rudder_pwm_candidate) {
						var cmd = "set rudder_pwm " + rudder_pwm_candidate;
						// console.log(cmd);
						sp_send(cmd, function(ret) {
							console.log(cmd);
						});
						rudder_pwm_candidate = null;
					}
					if (skrew_pwm_candidate) {
						var cmd = "set skrew_pwm " + skrew_pwm_candidate;
						// console.log(cmd);
						sp_send(cmd, function(ret) {
							console.log(cmd);
						});
						skrew_pwm_candidate = null;
					}
				}, 200);
				setInterval(function() {
					var cmd = "set network " + "1";
					sp_send(cmd, function(ret) {
						// console.log(cmd);
					});
				}, 1000);
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
						console.log(ret);
						latitude = parseFloat(params[1]);
						longitude = parseFloat(params[2]);
						next_waypoint_distance = parseFloat(params[3]);
						next_waypoint_idx = parseFloat(params[4]);
						next_waypoint_direction = parseFloat(params[5]);
						heading = parseFloat(params[6]);
						mode_flag = params[7].split(":");
						rudder_pwm = parseFloat(params[8]);
						skrew_pwm = parseFloat(params[9]);
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
						latitude : latitude,
						longitude : longitude,
						heading : heading,
						next_waypoint_distance : next_waypoint_distance,
						next_waypoint_idx : next_waypoint_idx,
						next_waypoint_direction : next_waypoint_direction,
						mode_flag : {
							autonomous : parseInt(mode_flag[0]) ? true : false,
							network : parseInt(mode_flag[1]) ? true : false,
						},
						rudder_pwm : rudder_pwm,
						skrew_pwm : skrew_pwm,
					};
					if (waypoints_required) {
						status.waypoints = waypoints;
						waypoints_required = false
					}
					if (history_required) {
						status.history = history;
						history_required = false
					}
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
			init_options : function(options) {
				if (options.aws_iot_enabled) {
					var AWS = require("aws-sdk");

					AWS.config.update({
						region : options.aws_region,
					});

					var docClient = new AWS.DynamoDB.DocumentClient();

					var params = {
						TableName : "asv_data",
						KeyConditionExpression : "device_id = :device_id",
						ExpressionAttributeValues : {
							":device_id" : options.aws_iot_device_id
						},
						ScanIndexForward : false,
						Limit : 60,
					};

					docClient.query(params, function(err, data) {
						if (err) {
							console.error("Unable to query. Error:", JSON
								.stringify(err, null, 2));
						} else {
							console.log("DynamoDB Query succeeded. "
								+ data.Items.length);
							history = data.Items;
						}
					});
				}
			},
			command_handler : function(cmd) {
				var split = cmd.split(' ');
				cmd = split[0].split('.')[1];
				switch (cmd) {
					case "set_waypoints" :
						var json_str = decodeURIComponent(split[1]);
						var new_waypoints = JSON.parse(json_str);
						var cmd = "set max_waypoint_num "
							+ new_waypoints.length;
						console.log(cmd);
						sp_send(cmd, function(ret) {
							for (var i = 0; i < new_waypoints.length; i++) {
								var cmd = "set waypoint " + i + " "
									+ new_waypoints[i].latitude.toFixed(6)
									+ " "
									+ new_waypoints[i].longitude.toFixed(6)
									+ " " + new_waypoints[i].allowable_error;
								console.log(cmd);
								sp_send(cmd, function(ret, idx) {
									if (idx == new_waypoints.length - 1) {
										waypoints = new_waypoints;
									}
								}, i);
							};
						});
						break;
					case "set_autonomous" :
						var v = parseInt(split[1]);
						sp_send("set autonomous " + v, function(ret) {
							// console.log(ret);
						});
						break;
					case "set_rudder_pwm" :
						rudder_pwm_candidate = parseInt(split[1]);
						break;
					case "set_skrew_pwm" :
						skrew_pwm_candidate = parseInt(split[1]);
						break;
					case "get_waypoints" :
						waypoints_required = true;
						break;
					case "get_history" :
						history_required = true;
						break;
				}
			},
			aws_iot_handler : function(device) {
				if (latitude == 0 && longitude == 0) {
					return;
				}
				var record = {
					"latitude" : latitude,
					"longitude" : longitude,
					"heading" : heading,
					"adc_values" : adc_values,
				};
				var message = JSON.stringify(record);
				console.log("Publish: " + message);
				device.publish('asv_data', message);
				history.unshift({
					payload : record,
					timestamp : Date.now(),
				});
				history.pop();
			}
		};
		return plugin;
	}
};