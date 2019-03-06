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

		var i2c = null;
		var gps_arrived = false;
		var gps_valid = false;
		var latitude = 0;
		var longitude = 0;
		var north = 0;
		var rudder_pwm = 0;
		var thruster_pwm = 0;
		var adc_values = [];
		var battery = 0;

		var history_1min = {};
		var history_10min = {};
		var history_100min = {};
		var history_1000min = {};
		var history_10000min = {};// around 6days

		// downstream status
		var waypoints_required = false;
		var history_required = false;

		// propo
		var m_ch_us = [];

		// manual
		var m_manual_rudder_pwm = undefined;
		var m_manual_thruster_pwm = undefined;

		// auto
		var next_waypoint_distance = 0;
		var next_waypoint_direction = 0;
		var p_d_direction = undefined;
		var waypoint_data = null;
		var m_last_sample_time_ms = 0;
		var m_last_heading;

		var options = {
			thruster_mode : "SINGLE",
			// auto
			propo_enabled : false,
			auto_mode : false,
			waypoints : [],
			next_waypoint_idx : 0,
			gain_kp : 160,
			gain_kv : 40,
			low_gain_kp : 40,
			low_gain_kv : 10,
			low_gain_deg : 5,
			SKREW_PINS : [17, 18, 27, 23],
			PWM_MIN_US : 1300,
			PWM_MIDDLE_US : 1500,
			PWM_MAX_US : 1700,
			PWM_MARGIN_MS : 100,
		};
		var m_status = {};

		// aws_iot
		var clientTokenUpdate;
		var clientTokenPublish;
		var clientTokenGet;
		var clientTokenGetCallback;

		function toFixedFloat(value, c) {
			return parseFloat(value ? value.toFixed(c) : 0);
		}

		function set_thruster_pwm(idx, us, _fd) {
			var need_to_close;
			var fd;
			if (_fd == null) {
				fd = fs.openSync("/dev/pi-blaster", 'w');
				need_to_close = true;
			} else {
				fd = _fd;
				need_to_close = false;
			}
			fs.writeSync(fd, sprintf("%d=%dus\n", options.SKREW_PINS[idx], us));
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
			if (options.ads7828_debug) {
				console.log("ch" + ch + "=b" + config.toString(2) + ",buff0="
					+ buff[0].toString(2) + ",buff1=" + buff[1].toString(2)
					+ ",v=" + value.toString(2));
			}
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
			if (options.ads7828_debug) {
				console.log("ch" + ch + "=b" + config.toString(2) + ",buff0="
					+ buff[0].toString(2) + ",buff1=" + buff[1].toString(2)
					+ ",v=" + value.toString(2));
			}
			i2c1.closeSync();
			return value;
		}

		get_ads7828_value = function(ch, cnt) {
			if (ch < 8) {
				return get_ads7828_value_single(ch, cnt);
			} else {
				return get_ads7828_value_differential(ch - 8, cnt);
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
						for (var i = 0; i < 16; i++) {
							adc_values[i] = get_ads7828_value(i, 1);
						}
						if (options.adc_debug) {
							var msg = "adc";
							for (var i = 0; i < 16; i++) {
								msg += " ch" + i + " : " + adc_values[i];
							}
							console.log(msg);
						}
						{// battery
							battery = 6 * 2.5 * adc_values[0] / (1 << 12);
						}
						{// north
							north = plugin_host.get_vehicle_north();
						}
						if (options.propo_enabled) { // propo
							var debug_str = "";
							for (var i = 1; i <= 3; i++) {
								var ch_v = 2.5 * adc_values[i] / (1 << 12);
								var ch_us = ch_v / 4 * 20000;

								if (m_ch_us[i] === undefined) {
									var mid = (options.propo[i] && options.propo[i].PWM_MIDDLE_US)
										|| options.PWM_MIDDLE_US;
									if (mid - 50 < ch_us && ch_us < mid + 50) {
										m_ch_us[i] = mid;
									}
								} else {
									m_ch_us[i] = ((m_ch_us[i] * 9) + ch_us) / 10;
								}
								if (options.propo_debug) {
									debug_str += "ch"
										+ i
										+ "="
										+ ch_v.toFixed(3)
										+ "V,"
										+ ch_us.toFixed()
										+ "us,"
										+ (m_ch_us[i]
											? m_ch_us[i].toFixed()
											: "****") + "us;";
								}
							}
							if (options.propo_debug) {
								console.log(debug_str);
							}
						}
						// pwm output
						function get_cutoff_us(value, ch_options) {
							var min = (ch_options && ch_options.PWM_MIN_US)
								|| options.PWM_MIN_US;
							var mid = (ch_options && ch_options.PWM_MIDDLE_US)
								|| options.PWM_MIDDLE_US;
							var max = (ch_options && ch_options.PWM_MAX_US)
								|| options.PWM_MAX_US;
							if (value === undefined) {
								value = mid;
							}
							if (value > max) {
								value = max;
							}
							if (value < min) {
								value = min;
							}
							return value;
						}
						function get_invert_pwm_us(value, ch_options, bln) {
							var mid = (ch_options && ch_options.PWM_MIDDLE_US)
								|| options.PWM_MIDDLE_US;
							if (value === undefined) {
								return mid;
							}
							if (bln) {
								return -(value - mid) + mid;
							} else {
								return value;
							}
						}
						function get_pwm_us(value, ch_options) {
							var inv = ch_options && ch_options.invert;
							value = get_invert_pwm_us(value, ch_options, inv);
							value = get_cutoff_us(value, ch_options);
							return value;
						}
						if (!options.auto_mode) {
							if (options.propo_enabled && 1500 < m_ch_us[3]
								&& m_ch_us[3] < 2000) {
								rudder_pwm = get_pwm_us(m_ch_us[1], options.propo[1]);
								thruster_pwm = get_pwm_us(m_ch_us[2], options.propo[2]);
							} else if (options.manual_enabled) {
								rudder_pwm = get_pwm_us(m_manual_rudder_pwm, options.propo[1]);
								thruster_pwm = get_pwm_us(m_manual_thruster_pwm, options.propo[2]);
								if (options.manual_debug) {
									console.log("manual : rud "
										+ m_manual_rudder_pwm + " : thr "
										+ m_manual_thruster_pwm);
								}
							} else {
								rudder_pwm = undefined;// to middle
								thruster_pwm = undefined;// to middle
							}
						}
						if (options.thruster_mode == 'SINGLE') {
							var ch0_us = get_pwm_us(rudder_pwm, options.ch[0]);
							var ch1_us = get_pwm_us(thruster_pwm, options.ch[1]);
							var ch2_us = options.PWM_MIDDLE_US;
							var ch3_us = options.PWM_MIDDLE_US;
							var fd = fs.openSync("/dev/pi-blaster", 'w');
							set_thruster_pwm(0, ch0_us, fd);
							set_thruster_pwm(1, ch1_us, fd);
							set_thruster_pwm(2, ch2_us, fd);
							set_thruster_pwm(3, ch3_us, fd);
							fs.closeSync(fd);
							if (options.single_debug) {
								console.log("single : " + ch0_us + " us, "
									+ ch1_us + " us, " + ch2_us + " us, "
									+ ch3_us + " us;");
							}
						} else if (options.thruster_mode == 'DOUBLE') {
							var thr_chl;
							var thr_chr;
							var thr_ch_ext0;
							var thr_ch_ext1;
							if (options.thruster_angle) {
								thr_chl = 2;
								thr_chr = 3;
								thr_ch_ext0 = 0;
								thr_ch_ext1 = 1;
							} else {
								thr_chl = 0;
								thr_chr = 1;
								thr_ch_ext0 = 2;
								thr_ch_ext1 = 3;
							}
							var rudder_delta = (rudder_pwm || options.PWM_MIDDLE_US)
								- options.PWM_MIDDLE_US;
							var thr_chl_us = thruster_pwm - rudder_delta;
							var thr_chr_us = thruster_pwm + rudder_delta;

							set_thruster_pwm(thr_chl, thr_chl_us, fd);
							set_thruster_pwm(thr_chr, thr_chr_us, fd);
							set_thruster_pwm(thr_ch_ext0, options.PWM_MIDDLE_US, fd);
							set_thruster_pwm(thr_ch_ext1, options.PWM_MIDDLE_US, fd);
							if (options.double_debug) {
								console.log("double : "
									+ options.thruster_angle + " deg, "
									+ thr_chl_us + " us, " + thr_chr_us
									+ " us, " + options.PWM_MIDDLE_US + " us, "
									+ options.PWM_MIDDLE_US + " us;");
							}
						} else if (options.thruster_mode == 'QUAD') {
							var rudder = rudder_pwm - options.PWM_MIDDLE_US;
							var thruster = thruster_pwm - options.PWM_MIDDLE_US;
							var angle = 180 * rudder
								/ (options.PWM_MAX_US - options.PWM_MIN_US);// -90:+90
							var thr_chv_us = thruster
								* Math.cos(Math.PI * angle / 180)
								+ options.PWM_MIDDLE_US;
							var thr_chh_us = thruster
								* Math.sin(Math.PI * angle / 180)
								+ options.PWM_MIDDLE_US;
							var thr_ch0_us = thr_chv_us;
							var thr_ch1_us = thr_chv_us;
							var thr_ch2_us = thr_chh_us;
							var thr_ch3_us = thr_chh_us;

							set_thruster_pwm(0, thr_ch0_us, fd);
							set_thruster_pwm(1, thr_ch1_us, fd);
							set_thruster_pwm(2, thr_ch2_us, fd);
							set_thruster_pwm(3, thr_ch3_us, fd);
							if (options.quad_debug) {
								console.log("quad : " + angle + " deg, "
									+ thr_ch0_us + " us, " + thr_ch1_us
									+ " us, " + thr_ch2_us + " us, "
									+ thr_ch3_us + " us;");
							}
						} else {
							set_thruster_pwm(0, options.PWM_MIDDLE_US, fd);
							set_thruster_pwm(1, options.PWM_MIDDLE_US, fd);
							set_thruster_pwm(2, options.PWM_MIDDLE_US, fd);
							set_thruster_pwm(3, options.PWM_MIDDLE_US, fd);
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
							if (options.gps_debug) {
								if (options.gps_test) {
									tpvData = options.gps_test;
								}
								console.log(tpvData);
							}
							if (tpvData.lat && tpvData.lon) {
								latitude = tpvData.lat;
								longitude = tpvData.lon;
								gps_valid = true;
							} else { // GPS_LOST
								gps_valid = false;
							}
							gps_arrived = true;
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
							// update status
							m_status = {
								gps : gps_valid,
								lat : toFixedFloat(latitude, 6),
								lon : toFixedFloat(longitude, 6),
								heading : toFixedFloat(-north, 3), // heading_from_north_clockwise
								bat : toFixedFloat(battery, 3),
								adc : adc_values,
								next_waypoint_idx : options.next_waypoint_idx,
								next_waypoint_distance : next_waypoint_distance,
								rudder_pwm : toFixedFloat(rudder_pwm, 0),
								thruster_pwm : toFixedFloat(thruster_pwm, 0),
								auto_mode : options.auto_mode,
								gain_kp : options.gain_kp,
								gain_kv : options.gain_kv,
								low_gain_kp : options.low_gain_kp,
								low_gain_kv : options.low_gain_kv,
								low_gain_deg : options.low_gain_deg,
							};
							if (gps_arrived) {
								var status = Object.assign({}, m_status);
								gps_arrived = false;
								if (waypoints_required) {
									status.waypoints = options.waypoints;
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
				},
				function(callback) {
					// auto operation
					setInterval(function() {
						if (!options.auto_mode) {
							return;
						}
						// reset pwm
						rudder_pwm = options.PWM_MIDDLE_US;
						thruster_pwm = options.PWM_MIDDLE_US;
						if (!gps_valid) { // GPS_LOST
							p_d_direction = undefined;
							return;
						}
						if (0) {
							if (options.next_waypoint_idx >= options.waypoints.length) {
								options.next_waypoint_idx = 0;
							}
						}
						if (!options.waypoints[options.next_waypoint_idx]) {
							return;
						}
						var waypoint = options.waypoints[options.next_waypoint_idx];

						var earth_r_km = 6356.752;
						var equator_r_km = 6378.137;
						var lat_deg2m = earth_r_km * Math.PI / 180 * 1000;
						var lon_deg2m = equator_r_km
							* Math.cos(latitude * Math.PI / 180) * Math.PI
							/ 180 * 1000;
						var d_lat_deg = waypoint.lat - latitude;
						var d_lon_deg = waypoint.lon - longitude;
						var d_lat_m = d_lat_deg * lat_deg2m;
						var d_lon_m = d_lon_deg * lon_deg2m;

						next_waypoint_distance = Math.sqrt(d_lat_m * d_lat_m
							+ d_lon_m * d_lon_m);
						next_waypoint_direction = Math.atan2(d_lon_m, d_lat_m)
							* 180 / Math.PI; // direction_from_north_clockwise
						if (Math.abs(next_waypoint_distance) < (waypoint.tol || 5)) { // tol_is_Tolerance
							// arrived
							if (options.auto_debug) {
								console.log("arrived at : "
									+ options.next_waypoint_idx);
							}
							if (!waypoint_data) {
								waypoint_data = {
									cmds : []
								};
							}
							if (waypoint.cmds) {
								function isObject(o) {
									return (o instanceof Object && !(o instanceof Array))
										? true
										: false;
								};
								var wait = function(data, params) {
									if (!isObject(params)) {
										params = {
											value : params
										};
									}
									if (!data.start) {
										data.start = params.start || Date.now()
											/ 1000;
									}
									return (Date.now() / 1000 > data.start
										+ params.value);
								}
								var sampling = function(data, params) {
									if (!isObject(params)) {
										params = {
											ch : params,
											cnt : 10
										};
									}
									var value = get_ads7828_value(params.ch, params.cnt);
									var state = {
										"lat" : latitude,
										"lon" : longitude,
										"value" : value,
									};
									plugin
										.aws_iot_publish(plugin.aws_thing_shadow, plugin.aws_client_id, params.topic, state);
									return true;
								}
								var funcs = {
									"wait" : wait,
									"sampling" : sampling,
								};
								var cmds = waypoint.cmds;
								if (!Array.isArray(cmds)) {
									cmds = [cmds];
								}
								for (var i = 0; i < cmds.length; i++) {
									var func = funcs[cmds[i].func];
									if (!func) {
										// error
										continue;
									}
									if (!waypoint_data.cmds[i]) {
										waypoint_data.cmds[i] = {};
									}
									var ret = func(waypoint_data.cmds[i], cmds[i].params);
									if (!ret) {
										return;
									} else {
										continue;
									}
								}
							}
							options.next_waypoint_idx++;
							waypoint_data = null;
							return;
						}
						// control
						var sample_time_ms = Date.now() / 1000;
						var sample_time_ms_dif;
						var heading = -north; // heading_from_north_clockwise
						if (heading == m_last_heading) {
							return;// skip
						} else {
							if (m_last_sample_time_ms == 0) {
								m_last_sample_time_ms = sample_time_ms;
								return;
							}
							sample_time_ms_dif = sample_time_ms
								- m_last_sample_time_ms;
							m_last_heading = heading;
							m_last_sample_time_ms = sample_time_ms;
						}
						var d_direction = next_waypoint_direction - heading;
						if (d_direction < -180) {
							d_direction += 2 * 180;
						}
						if (d_direction > 180) {
							d_direction -= 2 * 180;
						}
						if (p_d_direction === undefined) {
							p_d_direction = d_direction;
							return;
						}
						// pd
						var rudder_pwm_mid = options.PWM_MIDDLE_US;
						if (Math.abs(d_direction) < options.low_gain_deg) {
							rudder_pwm = options.low_gain_kp
								* (d_direction * Math.PI / 180)
								- options.low_gain_kv
								* ((d_direction - p_d_direction) * Math.PI / 180)
								/ sample_time_ms_dif * 1000 + rudder_pwm_mid;
						} else {
							rudder_pwm = options.gain_kp
								* (d_direction * Math.PI / 180)
								- options.gain_kv
								* ((d_direction - p_d_direction) * Math.PI / 180)
								/ sample_time_ms_dif * 1000 + rudder_pwm_mid;
						}
						thruster_pwm = options.PWM_MAX_US;// will_be_cut_off_by_options.ch[2].PWM_MAX_US
						p_d_direction = d_direction;
						if (options.auto_debug) {
							console.log("rud " + rudder_pwm + " : thr "
								+ thruster_pwm + " : head " + heading
								+ " : nw_dir " + next_waypoint_direction);
						}
					}, 200);
					callback(null);
				}], function(err, result) {
			});
		var plugin = {
			name : PLUGIN_NAME,
			aws_thing_shadow : null,
			aws_client_id : null,
			init_options : function(_options) {
				options = Object.assign(options, _options.usvc);
			},
			command_handler : function(cmd) {
				var split = cmd.split(' ');
				cmd = split[0].split('.')[1];
				switch (cmd) {
					case "set_waypoints" :
						var json_str = decodeURIComponent(split[1]);
						var new_waypoints = JSON.parse(json_str);
						console.log(new_waypoints);
						options.waypoints = new_waypoints;
						plugin
							.aws_iot_update(plugin.aws_thing_shadow, plugin.aws_client_id, {
								waypoints : options.waypoints
							});
						break;
					case "set_next_waypoint_idx" :
						var v = parseInt(split[1]);
						options.next_waypoint_idx = v;
						break;
					case "set_automode" :
						var v = parseInt(split[1]);
						options.auto_mode = v ? true : false;
						break;
					case "set_rudder_pwm" :
						m_manual_rudder_pwm = parseInt(split[1]);
						break;
					case "set_thruster_pwm" :
						m_manual_thruster_pwm = parseInt(split[1]);
						break;
					case "get_waypoints" :
						waypoints_required = true;
						break;
					case "get_history" :
						history_required = true;
						break;
				}
			},
			aws_iot_conneced : function(thing_shadow, client_id) {
				this.aws_thing_shadow = thing_shadow;
				this.aws_client_id = client_id;
				var reported_fnc = function(state, is_delta) {
					var report = {};
					if (state.lat !== undefined) {
						latitude = state.lat;
						report.lat = latitude;
					}
					if (state.lon !== undefined) {
						longitude = state.lon;
						report.lon = longitude;
					}
					if (state.waypoints !== undefined) {
						options.waypoints = state.waypoints;
						report.waypoints = options.waypoints;
					}
					if (state.thruster_mode !== undefined) {
						options.thruster_mode = state.thruster_mode;
						report.thruster_mode = options.thruster_mode;

						// reset
						set_thruster_pwm(0, options.PWM_MIDDLE_US, fd);
						set_thruster_pwm(1, options.PWM_MIDDLE_US, fd);
						set_thruster_pwm(2, options.PWM_MIDDLE_US, fd);
						set_thruster_pwm(3, options.PWM_MIDDLE_US, fd);
					}
					if (state.next_waypoint_idx !== undefined) {
						options.next_waypoint_idx = state.next_waypoint_idx;
						report.next_waypoint_idx = options.next_waypoint_idx;
					}
					if (state.auto_mode !== undefined) {
						options.auto_mode = state.auto_mode;
						report.auto_mode = options.auto_mode;
					}
					if (state.gain_kp !== undefined) {
						options.gain_kp = state.gain_kp;
						report.gain_kp = options.gain_kp;
					}
					if (state.gain_kv !== undefined) {
						options.gain_kv = state.gain_kv;
						report.gain_kv = options.gain_kv;
					}
					if (state.low_gain_kp !== undefined) {
						options.low_gain_kp = state.low_gain_kp;
						report.low_gain_kp = options.low_gain_kp;
					}
					if (state.low_gain_kv !== undefined) {
						options.low_gain_kv = state.low_gain_kv;
						report.low_gain_kv = options.low_gain_kv;
					}
					if (state.low_gain_deg !== undefined) {
						options.low_gain_deg = state.low_gain_deg;
						report.low_gain_deg = options.low_gain_deg;
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
					clientTokenUpdate = thing_shadow.update(client_id, cmd);
				}
				thing_shadow
					.on('status', function(thingName, stat, clientToken,
						stateObject) {
						if (stat == 'rejected') {
							console.log(clientToken + " rejected : "
								+ stateObject);
							return;
						}
						if (clientToken == clientTokenPublish) {
							console.log(clientToken + " puglish : " + stat
								+ " : " + stateObject);
						}
						if (clientToken == clientTokenGet) {
							if (stateObject.state.reported) {
								var state = stateObject.state.reported;
								reported_fnc(state, false);

								if (state.history_1min) {
									history_1min = state.history_1min;
								}
								if (state.history_10min) {
									history_10min = state.history_10min;
								}
								if (state.history_100min) {
									history_100min = state.history_100min;
								}
								if (state.history_1000min) {
									history_1000min = state.history_1000min;
								}
								if (state.history_10000min) {
									history_10000min = state.history_10000min;
								}
							}
							if (stateObject.state.delta) {
								delta_fnc(stateObject.state.delta);
							}
							if (clientTokenGetCallback) {
								clientTokenGetCallback();
								clientTokenGetCallback = null;
							}
						} // end of get
						if (options.aws_iot_debug) {
							console.log('received ' + stat + ' on ' + thingName
								+ ':' + JSON.stringify(stateObject));
						}
					});
				thing_shadow.on('delta', function(thingName, stateObject) {
					delta_fnc(stateObject.state);
				});
				thing_shadow.on('timeout', function(thingName, clientToken) {
					if (clientToken == clientTokenGet) {
						clientTokenGet = thing_shadow.get(client_id);
					}
					console.log('received timeout : ' + clientToken);
				});
			},
			aws_iot_registered : function(thing_shadow, client_id) {
				this
					.aws_iot_get(thing_shadow, client_id, function() {
						// start sync
						setInterval(function() {
							var state = Object.assign({}, m_status);
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
							var new_value;
							if (!gps_valid) { // GPS_LOST
								new_value = {
									"gps" : false,
									"bat" : state.bat,
								};
							} else {
								new_value = {
									"lat" : state.lat,
									"lon" : state.lon,
									"bat" : state.bat,
								};
							}
							for (var i = 0; i < history_tbl.length - 1; i++) {
								var keys = Object.keys(history_tbl[i]);
								for (var j = 0; j < keys.length
									- max_count_tbl[i]; j++) {
									delete history_tbl[i][keys[j]];
									report_tbl[i][keys[j]] = null;
								}
							}
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

							plugin
								.aws_iot_update(thing_shadow, client_id, state);
						}, (options.aws_iot_interval_sec || 10) * 1000);
					});
			},
			aws_iot_get : function(thing_shadow, client_id, callback) {
				clientTokenGetCallback = callback;
				clientTokenGet = thing_shadow.get(client_id);
			},
			aws_iot_update : function(thing_shadow, client_id, state) {
				var cmd = {
					"state" : {
						"reported" : state
					}
				};
				clientTokenUpdate = thing_shadow.update(client_id, cmd);
				if (options.aws_iot_debug) {
					console.log("report shadow: " + JSON.stringify(cmd));
				}
			},
			aws_iot_publish : function(thing_shadow, client_id, topic, state) {
				var message = JSON.stringify(state);
				if (options.aws_iot_debug) {
					console.log("publish: " + message);
				}
				clientTokenPublish = thing_shadow.publish(topic, message);
			},
		};
		return plugin;
	}
};