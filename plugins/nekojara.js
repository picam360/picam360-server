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
					function mat4_fromQuat(q) {
						var out = [];
						var x = q[0], y = q[1], z = q[2], w = q[3], x2 = x + x, y2 = y
							+ y, z2 = z + z,

						xx = x * x2, yx = y * x2, yy = y * y2, zx = z * x2, zy = z
							* y2, zz = z * z2, wx = w * x2, wy = w * y2, wz = w
							* z2;

						out[0] = 1 - yy - zz;
						out[1] = yx + wz;
						out[2] = zx - wy;
						out[3] = 0;

						out[4] = yx - wz;
						out[5] = 1 - xx - zz;
						out[6] = zy + wx;
						out[7] = 0;

						out[8] = zx + wy;
						out[9] = zy - wx;
						out[10] = 1 - xx - yy;
						out[11] = 0;

						out[12] = 0;
						out[13] = 0;
						out[14] = 0;
						out[15] = 1;

						return out;
					}
					function mat4_multiply(a, b) {
						var out = [];
						var a00 = a[0], a01 = a[1], a02 = a[2], a03 = a[3], a10 = a[4], a11 = a[5], a12 = a[6], a13 = a[7], a20 = a[8], a21 = a[9], a22 = a[10], a23 = a[11], a30 = a[12], a31 = a[13], a32 = a[14], a33 = a[15];

						// Cache only the current line of the second matrix
						var b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
						out[0] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
						out[1] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
						out[2] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
						out[3] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

						b0 = b[4];
						b1 = b[5];
						b2 = b[6];
						b3 = b[7];
						out[4] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
						out[5] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
						out[6] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
						out[7] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

						b0 = b[8];
						b1 = b[9];
						b2 = b[10];
						b3 = b[11];
						out[8] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
						out[9] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
						out[10] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
						out[11] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;

						b0 = b[12];
						b1 = b[13];
						b2 = b[14];
						b3 = b[15];
						out[12] = b0 * a00 + b1 * a10 + b2 * a20 + b3 * a30;
						out[13] = b0 * a01 + b1 * a11 + b2 * a21 + b3 * a31;
						out[14] = b0 * a02 + b1 * a12 + b2 * a22 + b3 * a32;
						out[15] = b0 * a03 + b1 * a13 + b2 * a23 + b3 * a33;
						return out;
					}

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
						var trans_mat4 = mat4_fromQuat(q);
						var mat4 = [//
						0, -1, 0, 1,//
						0, 0, 0, 1,//
						0, 0, 0, 1,//
						0, 0, 0, 1];
						mat4 = mat4_multiply(trans_mat4, mat4);
						var euler_xy = {
							y : Math.atan2(-mat4[2], mat4[0]) * 180 / Math.PI,
							x : Math.asin(mat4[1]) * 180 / Math.PI,
							z : 0
						};
						var euler_xyz = toEulerianAngle(q, "YXZ");
						var next_taget_yaw = (Math.abs(euler_xy.x) > -80)
							? euler_xy.y
							: euler_xyz.y;
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