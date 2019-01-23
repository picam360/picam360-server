module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create usv plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;
		var SerialPort = require('serialport');
		var gpsd = require('node-gpsd');

		var PLUGIN_NAME = "usvc";
		var COM_PORT = "/dev/ttyACM0";
		var DELIMITER = '\r\n';
		var TIMEOUT_MS = 5000;
		var ADS7828_ADDRESS = 0x48;
		var SKREW_PINS = [17, 18, 27, 23];
		var PWM_MIDDLE_MS = 1480;
		var PWM_MARGIN_MS = 100;

		var i2c = null;
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

		var options = {};

		// aws_iot
		var clientTokenUpdate;
		var clientTokenGet;

		function set_skrew_pwm(idx, us, _fd) {
			var need_to_close;
			var fd;
			if (_fd == null) {
				fd = fs.openSync("/dev/pi-blaster", 'w');
				need_to_close = true;
			} else {
				fd = _fd;
				need_to_close = false;
			}
			fs.writeSync(fd, sprintf("%d=%dus\n", SKREW_PINS[idx], us));
			if (need_to_close) {
				fs.closeSync(fd);
			}
		}

		get_ads7828_value = function(ch, cnt) {
			var ch2addr = [0x0, 0x4, 0x1, 0x5, 0x2, 0x6, 0x3, 0x7];
			const
			i2c1 = i2c.openSync(1);
			var config = 0x80 | (ch2addr[ch] << 4) | (0x03 << 2);
			var value = 0;
			var buff = new Buffer(2);
			if (cnt <= 0) {
				cnt = 1;
			}
			for (var i = 0; i < cnt; i++) {
				i2c1.readI2cBlockSync(ADS7828_ADDRESS, config, 2, buff);
				value += (buff[0] << 8) + buff[1];
			}
			value /= cnt;
			// console.log("ch" + ch + "=b" + config.toString(2) + ",buff0=" +
			// buff[0].toString(2) + ",buff1=" + buff[1].toString(2) + ",v=" +
			// value.toString(2));
			i2c1.closeSync();
			return value;
		}

		async
			.waterfall([
				function(callback) {
					i2c = require('i2c-bus');
					// var wire = new i2c(ADC_ADDRESS, {device: '/dev/i2c-1'});
					// // point
					// to your i2c address, debug provides REPL interface
					setInterval(function() {
						var ch0_raw = get_ads7828_value(0, 10);
						var ch1_raw = get_ads7828_value(1, 10);
						var ch2_raw = get_ads7828_value(2, 10);
						var ch3_raw = get_ads7828_value(3, 10);
						var ch4_raw = get_ads7828_value(4, 10);
						var ch5_raw = get_ads7828_value(5, 10);
						var ch6_raw = get_ads7828_value(6, 10);
						var ch7_raw = get_ads7828_value(7, 10);

						adc_values = [ch0_raw, ch1_raw, ch2_raw, ch3_raw,
							ch4_raw, ch5_raw, ch6_raw, ch7_raw];

						var ch0 = 6 * 2.5 * ch0_raw / (1 << 12);
						var ch1 = 2.5 * ch1_raw / (1 << 12);
						var ch2 = 2.5 * ch2_raw / (1 << 12);
						var ch3 = 2.5 * ch3_raw / (1 << 12);
						var ch1_ms = ch1 / 4 * 20000;
						var ch2_ms = ch2 / 4 * 20000;
						var ch3_ms = ch3 / 4 * 20000;
						var skrew_ch0 = PWM_MIDDLE_MS;
						var skrew_ch1 = PWM_MIDDLE_MS;
						var skrew_ch2 = PWM_MIDDLE_MS;
						var skrew_ch3 = PWM_MIDDLE_MS;

						if (ch1_ms > PWM_MIDDLE_MS + PWM_MARGIN_MS) {
							skrew_ch0 = PWM_MIDDLE_MS + PWM_MARGIN_MS;
							skrew_ch2 = PWM_MIDDLE_MS + PWM_MARGIN_MS;
						} else if (ch1_ms < PWM_MIDDLE_MS - PWM_MARGIN_MS) {
							skrew_ch0 = PWM_MIDDLE_MS - PWM_MARGIN_MS;
							skrew_ch2 = PWM_MIDDLE_MS - PWM_MARGIN_MS;
						}
						if (ch2_ms > PWM_MIDDLE_MS + PWM_MARGIN_MS) {
							skrew_ch1 = PWM_MIDDLE_MS + PWM_MARGIN_MS;
							skrew_ch3 = PWM_MIDDLE_MS + PWM_MARGIN_MS;
						} else if (ch2_ms < PWM_MIDDLE_MS - PWM_MARGIN_MS) {
							skrew_ch1 = PWM_MIDDLE_MS - PWM_MARGIN_MS;
							skrew_ch3 = PWM_MIDDLE_MS - PWM_MARGIN_MS;
						}
						// console.log(ch0.toFixed(3) + "V " + ch1.toFixed(3) +
						// "V " +
						// ch2.toFixed(3) + "V " + ch3.toFixed(3) + "V");
						// console.log(ch0.toFixed(3) + "V " + ch1_ms.toFixed(3)
						// + "ms "
						// + ch2_ms.toFixed(3) + "ms " + ch3_ms.toFixed(3) +
						// "ms");
						{
							heading = plugin_host.get_vehicle_north();
						}
						{
							var fd = fs.openSync("/dev/pi-blaster", 'w');
							if (1500 < ch3_ms && ch3_ms < 2000) {
								set_skrew_pwm(0, skrew_ch0, fd);
								set_skrew_pwm(1, skrew_ch1, fd);
								set_skrew_pwm(2, skrew_ch2, fd);
								set_skrew_pwm(3, skrew_ch3, fd);
							} else {
								set_skrew_pwm(0, PWM_MIDDLE_MS, fd);
								set_skrew_pwm(1, PWM_MIDDLE_MS, fd);
								set_skrew_pwm(2, PWM_MIDDLE_MS, fd);
								set_skrew_pwm(3, PWM_MIDDLE_MS, fd);
							}
							fs.closeSync(fd);
						}
					}, 200);
					callback(null);
				},
				function(callback) {
					var listener = new gpsd.Listener({
						port : 2947,
						hostname : 'localhost',
						logger : {
							info : function() {
							},
							warn : console.warn,
							error : console.error
						},
						parse : true
					});
					listener.connect(function() {
						console.log('GPSD Connected');
						listener.on('TPV', function(tpvData) {
							// console.log(tpvData);
							if (tpvData.lat && tpvData.lon) {
								latitude = tpvData.lat;
								longitude = tpvData.lon;
							} else {
								latitude = 0;
								longitude = 0;
							}
						});
						listener.watch();
					});
					// listener.disconnect(function() {
					// console.log('Disconnected');
					// });
					callback(null);
				},
				function(callback) {
					plugin_host
						.add_status(PLUGIN_NAME + ".status", function() {
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
										autonomous : parseInt(mode_flag[0])
											? true
											: false,
										network : parseInt(mode_flag[1])
											? true
											: false,
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
			init_options : function(_options) {
				options = _options.usvc || {};
				if (_options.aws_iot_enabled) {
					// var AWS = require("aws-sdk");
					//
					// AWS.config.update({
					// region : options.aws_region,
					// });
					//
					// var docClient = new AWS.DynamoDB.DocumentClient();
					//
					// var params = {
					// TableName : "asv_data",
					// KeyConditionExpression : "device_id = :device_id",
					// ExpressionAttributeValues : {
					// ":device_id" : options.aws_iot_device_id
					// },
					// ScanIndexForward : false,
					// Limit : 60,
					// };
					//
					// docClient.query(params, function(err, data) {
					// if (err) {
					// console.error("Unable to query. Error:", JSON
					// .stringify(err, null, 2));
					// } else {
					// console.log("DynamoDB Query succeeded. "
					// + data.Items.length);
					// history = data.Items;
					// }
					// });
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
			aws_iot_conneced : function(thingShadow, client_id) {
				var delta_fnc = function(state) {
					if (state.waypoints) {
						waypoints = state.waypoints;
					}
					var report = {
						"state" : {
							"desired" : null,
							"reported" : {
								"waypoints" : waypoints
							}
						}
					};
					clientTokenUpdate = thingShadow.update(client_id, report);
				}
				thingShadow.on('status', function(thingName, stat, clientToken,
					stateObject) {
					if (clientToken == clientTokenGet) {
						if (stateObject.state.reported.waypoints) {
							waypoints = stateObject.state.reported.waypoints;
						}
						if (stateObject.state.delta) {
							delta_fnc(stateObject.state.delta);
						}
					}
					// console.log('received ' + stat + ' on ' + thingName + ':
					// '
					// + JSON.stringify(stateObject));
				});
				thingShadow.on('delta', function(thingName, stateObject) {
					delta_fnc(stateObject.state);
				});
				thingShadow.on('timeout', function(thingName, clientToken) {
					console.log('received timeout : ' + clientToken);
				});
			},
			aws_iot_registered : function(thingShadow, client_id) {
				var interval_sec = options.aws_iot_interval_sec || 60;
				clientTokenGet = thingShadow.get(client_id);
				var fnc = function() {
					if (latitude == 0 && longitude == 0) {
						return;
					}
					var state = {
						"latitude" : latitude,
						"longitude" : longitude,
						"heading" : heading,
						"adc_values" : adc_values,
					};
					var report = {
						"state" : {
							"reported" : state
						}
					};
					console.log("report shadow: " + JSON.stringify(report));
					clientTokenUpdate = thingShadow.update(client_id, report);

					var message = JSON.stringify(state);
					console.log("publish: " + message);
					thingShadow.publish('asv_data', message);
					// history.unshift({
					// payload : record,
					// timestamp : Date.now(),
					// });
					// history.pop();
				};
				fnc();
				setInterval(fnc, interval_sec * 1000);
			}
		};
		return plugin;
	}
};