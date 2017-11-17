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
		var GEAR_RATIO = 4;
		var ROUND_STEP = 48 * GEAR_RATIO;
		var NUM_OF_PHASE = 4;

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
					fs
						.writeFileSync('/sys/class/gpio/gpio' + MODE + '/value', 0);
					callback(null);
				},
				function(callback) {// step timer
					setInterval(function() {
						if (target_step == step) {
							return;
						} else if (target_step > step) {
							step++;
						} else {
							step--;
						}
						var phase = step % NUM_OF_PHASE;
						if (phase < 0) {
							phase += NUM_OF_PHASE;
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
					}, 20);
					callback(null);
				},
				function(callback) {// check view quaternion timer
					// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
					function toEulerianAngle(quat, seq) {
						var q;// q0:w,q1:x,q2:y,q3:z, XYZ
						if (seq == "YXZ") {
							q = [quat[3], quat[1], quat[0], quat[2]];
						}
						// euler
						var cosx = -2.0 * (q[1] * q[1] + q[2] * q[2]) + 1.0;
						var sinx = +2.0 * (q[2] * q[3] + q[0] * q[1]);
						var siny = -2.0 * (q[1] * q[3] - q[0] * q[2]);
						var cosz = -2.0 * (q[2] * q[2] + q[3] * q[3]) + 1.0;
						var sinz = +2.0 * (q[1] * q[2] + q[0] * q[3]);

						siny = siny > 1.0 ? 1.0 : siny;
						siny = siny < -1.0 ? -1.0 : siny;

						if (seq == "YXZ") {
							return {
								y : Math.atan2(sinx, cosx) * 180 / Math.PI,
								x : Math.asin(siny) * 180 / Math.PI,
								z : Math.atan2(sinz, cosz) * 180 / Math.PI
							};
						}
					}
					setInterval(function() {
						var q = plugin_host.get_view_quaternion();
						var euler = toEulerianAngle(q, "YXZ");
						var next_taget_yaw = (Math.abs(euler.z) < 90)
							? euler.y
							: euler.y + 180;//TODO:horizon area is not stable
						//console.log("y:" + euler.y + "x:" + euler.x + "z:"
						//	+ euler.z)
						var base_step = Math.round(target_step / ROUND_STEP)
							* ROUND_STEP;
						var taget_yaw = (target_step - base_step) / ROUND_STEP
							* 360;
						diff_yaw = next_taget_yaw - taget_yaw;
						if (diff_yaw > 180) {
							diff_yaw -= 360;
						} else if (diff_yaw < -180) {
							diff_yaw += 360;
						}
						target_step += Math.round(diff_yaw / 360 * ROUND_STEP);
						// console.log("taget_yaw:" + taget_yaw
						// + ",next_taget_yaw:" + next_taget_yaw
						// + ".target_step:" + target_step);
					}, 100);
					callback(null);
				}], function(err, result) {
			});

		var plugin = {

		};
		return plugin;
	}
};