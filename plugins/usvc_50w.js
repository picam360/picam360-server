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
		var north = 0;
		var rudder_pwm = 0;
		var skrew_pwm = 0;
		var adc_values = [];
		var thruster_mode = 0;
		var auto_mode = false;
		var battery = 0;

		var history_1min = {};
		var history_10min = {};
		var history_100min = {};
		var history_1000min = {};
		var history_10000min = {};// around 6days

		var waypoints_required = false;
		var history_required = false;
		var rudder_pwm_candidate = null;
		var skrew_pwm_candidate = null;

		var options = {};

		// aws_iot
		var clientTokenUpdate;
		var clientTokenGet;
		var clientTokenGetCallback;

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

		get_ads7828_value_single = function(ch, cnt) {
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

		get_ads7828_value_differential = function(ch, cnt) {
			var ch2addr = [0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7];
			var i2c1 = i2c.openSync(1);
			var config = 0x00 | (ch2addr[ch] << 4) | (0x03 << 2);
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

		get_ads7828_value = function(ch, cnt) {
			if (ch < 8) {
				return get_ads7828_value_single(ch, cnt);
			} else {
				var pos = get_ads7828_value_differential(ch - 8, cnt);
				var neg = get_ads7828_value_differential(ch - 8 + 4, cnt);
				if (pos > neg) {
					return pos;
				} else {
					return -neg;
				}
			}
		}

		async
			.waterfall([
				function(callback) {
					i2c = require('i2c-bus');
					// var wire = new i2c(ADC_ADDRESS, {device: '/dev/i2c-1'});
					// // point
					// to your i2c address, debug provides REPL interface
					setInterval(function() {
						adc_values = [];
						for (var i = 0; i < 12; i++) {
							adc_values[i] = get_ads7828_value(i, 10);
						}

						battery = 6 * 2.5 * adc_values[0] / (1 << 12);

						var ch1 = 2.5 * adc_values[1] / (1 << 12);
						var ch2 = 2.5 * adc_values[2] / (1 << 12);
						var ch3 = 2.5 * adc_values[3] / (1 << 12);
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
							north = plugin_host.get_vehicle_north();
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
									north : north,
									next_waypoint_distance : next_waypoint_distance,
									next_waypoint_idx : next_waypoint_idx,
									next_waypoint_direction : next_waypoint_direction,
									rudder_pwm : rudder_pwm,
									skrew_pwm : skrew_pwm,
								};
								if (waypoints_required) {
									status.waypoints = waypoints;
									waypoints_required = false
								}
								if (history_required) {
									status.history = Object
										.assign({}, history_1min, history_10min, history_100min, history_1000min, history_10000min);
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
				thruster_mode = options.default_thruster_mode || "SINGLE";
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
				var reported_fnc = function(state, is_delta) {
					var report = {};
					if (state.waypoints !== undefined) {
						waypoints = state.waypoints;
						report.waypoints = waypoints;
					}
					if (state.thruster_mode !== undefined) {
						thruster_mode = state.thruster_mode;
						report.thruster_mode = thruster_mode;
					}
					if (state.next_waypoint_idx !== undefined) {
						next_waypoint_idx = state.next_waypoint_idx;
						report.next_waypoint_idx = next_waypoint_idx;
					}
					if (state.auto_mode !== undefined) {
						auto_mode = state.auto_mode;
						report.auto_mode = auto_mode;
					}
					return report;
				}
				var delta_fnc = function(state) {
					var report = reported_fnc(state, true);
					var cmd = {
						"state" : {
							"desired" : null,
							"reported" : report
						}
					};
					clientTokenUpdate = thingShadow.update(client_id, cmd);
				}
				thingShadow.on('status', function(thingName, stat, clientToken,
					stateObject) {
					if (clientToken == clientTokenGet) {
						if (stateObject.state.reported) {
							var state = stateObject.state.reported;
							reported_fnc(state, false);

							if (state.history_1min) {
								history_min = state.history_1min;
							}
							if (state.history_10min) {
								history_hour = state.history_10min;
							}
							if (state.history_100min) {
								history_day = state.history_100min;
							}
							if (state.history_1000min) {
								history_week = state.history_1000min;
							}
							if (state.history_10000min) {
								history_month = state.history_10000min;
							}
						}
						if (stateObject.state.delta) {
							delta_fnc(stateObject.state.delta);
						}
						if (clientTokenGetCallback) {
							clientTokenGetCallback();
							clientTokenGetCallback = null;
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
					if (clientToken == clientTokenGet) {
						clientTokenGet = thingShadow.get(client_id);
					}
					console.log('received timeout : ' + clientToken);
				});
			},
			aws_iot_registered : function(thingShadow, client_id) {
				this
					.aws_iot_get(thingShadow, client_id, function() {
						// start sync
						setInterval(function() {
							if (latitude == 0 && longitude == 0) {
								return;
							}
							var state = {
								"lat" : latitude.toFixed(6),
								"lon" : longitude.toFixed(6),
								"north" : north.toFixed(3),
								"bat" : battery.toFixed(3),
								"adc" : adc_values,
							};
							// update history
							// care shadow size limited 8k
							var history_tbl = [history_1min, history_10min,
								history_100min, history_1000min,
								history_10000min];
							var report_tbl = [{}, {}, {}, {}, {}];
							var max_count_tbl = [5, 5, 5, 5, 5];// max25nodes30days
							var interval_s_tbl = [60, 60 * 10, 60 * 100,
								60 * 1000, 60 * 10000];
							var new_key = parseInt(Date.now() / 1000);
							var new_value = {
								"lat" : state.lat,
								"lon" : state.lon,
							};
							for (var i = 0; i < history_tbl.length - 1; i++) {
								var keys = Object.keys(history_tbl[i]);
								if (new_key - (keys[keys.length - 1] || 0) > interval_s_tbl[i]) {
									history_tbl[i][new_key] = new_value;
									report_tbl[i][new_key] = new_value;
								} else {
									break;
								}
								if (keys.length + 1 > max_count_tbl[i]) {
									new_key = keys[0];
									new_value = history_tbl[i][new_key];
									delete history_tbl[i][new_key];
									report_tbl[i][new_key] = null;
								} else {
									break;
								}
							}
							state.history_1min = report_tbl[0];
							state.history_10min = report_tbl[1];
							state.history_100min = report_tbl[2];
							state.history_1000min = report_tbl[3];
							state.history_10000min = report_tbl[4];

							var cmd = {
								"state" : {
									"reported" : state
								}
							};
							console
								.log("report shadow: " + JSON.stringify(cmd));
							clientTokenUpdate = thingShadow
								.update(client_id, cmd);
							// var message = JSON.stringify(state);
							// console.log("publish: " + message);
							// thingShadow.publish('asv_data', message);
						}, (options.aws_iot_interval_sec || 10) * 1000);
					});
			},
			aws_iot_get : function(thingShadow, client_id, callback) {
				clientTokenGetCallback = callback;
				clientTokenGet = thingShadow.get(client_id);
			},
		};
		return plugin;
	}
};