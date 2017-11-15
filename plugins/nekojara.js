module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create nekojara plugin");
		var async = require('async');
		var fs = require("fs");
		var BLUE_A1 = 17;
		var WHITE_A2 = 18;
		var YELLOW_B1 = 22;
		var RED_B2 = 23;
		var MODE = 27;
		var step = 0;
		var target_step = 0;
		var ROUND_STEP = 47 * 4;

		function export_pin(pin) {
			if (fs.existsSync('/sys/class/gpio/gpio' + pin)) {
			} else {
				fs.writeFileSync('/sys/class/gpio/export', pin);
			}
		}

		async
			.waterfall([
				function(callback) {// export
					export_pin(BLUE_A1);
					export_pin(WHITE_A2);
					export_pin(YELLOW_B1);
					export_pin(RED_B2);
					export_pin(MODE);
					setTimeout(function() {
						callback(null);
					}, 1000);
				},
				function(callback) {// direction
					fs.writeFileSync('/sys/class/gpio/gpio' + BLUE_A1
						+ '/direction', 'out');
					fs.writeFileSync('/sys/class/gpio/gpio' + WHITE_A2
						+ '/direction', 'out');
					fs.writeFileSync('/sys/class/gpio/gpio' + YELLOW_B1
						+ '/direction', 'out');
					fs.writeFileSync('/sys/class/gpio/gpio' + RED_B2
						+ '/direction', 'out');
					fs.writeFileSync('/sys/class/gpio/gpio' + MODE
						+ '/direction', 'out');
					callback(null);
				},
				function(callback) {// step timer
					var phase = 0;
					setInterval(function() {
						var diff_step = target_step - step;
						if (Math.abs(diff_step) < 1) {
							return;
						}
						switch (phase) {
							case 0 :
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ BLUE_A1 + '/value', 1);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ WHITE_A2 + '/value', 0);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ YELLOW_B1 + '/value', 0);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ RED_B2 + '/value', 1);
								break;
							case 1 :
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ BLUE_A1 + '/value', 1);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ WHITE_A2 + '/value', 0);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ YELLOW_B1 + '/value', 1);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ RED_B2 + '/value', 0);
								break;
							case 2 :
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ BLUE_A1 + '/value', 0);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ WHITE_A2 + '/value', 1);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ YELLOW_B1 + '/value', 1);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ RED_B2 + '/value', 0);
								break;
							case 3 :
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ BLUE_A1 + '/value', 0);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ WHITE_A2 + '/value', 1);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ YELLOW_B1 + '/value', 0);
								fs.writeFileSync('/sys/class/gpio/gpio'
									+ RED_B2 + '/value', 1);
								break;
						}
						if (diff_step > 0) {
							phase++;
						} else {
							phase--;
						}
						if (phase < 0) {
							phase = 3;
							step--;
						} else if (phase > 3) {
							phase = 0;
							step++;
						}
					}, 20);
					callback(null);
				}, function(callback) {// check view quaternion timer
					function toEulerianAngle(q) {
						var q2sqr = q[2] * q[2];
						var t0 = -2.0 * (q2sqr + q[3] * q[3]) + 1.0;
						var t1 = +2.0 * (q[1] * q[2] + q[0] * q[3]);
						var t2 = -2.0 * (q[1] * q[3] - q[0] * q[2]);
						var t3 = +2.0 * (q[2] * q[3] + q[0] * q[1]);
						var t4 = -2.0 * (q[1] * q[1] + q2sqr) + 1.0;

						t2 = t2 > 1.0 ? 1.0 : t2;
						t2 = t2 < -1.0 ? -1.0 : t2;

						return {
							pitch : Math.asin(t2),
							roll : Math.atan2(t3, t4),
							yaw : Math.atan2(t1, t0)
						};
					}
					setInterval(function() {
						var q = plugin_host.get_view_quaternion();
						var euler = toEulerianAngle(q);
						var taget_yaw = euler.yaw;
						target_step = taget_yaw / Math.PI * ROUND_STEP;
					}, 100);
					callback(null);
				}], function(err, result) {
			});

		var plugin = {

		};
		return plugin;
	}
};