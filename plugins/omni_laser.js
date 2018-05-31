module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create nekojara plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;
		var phase_pins = [17, 22, 24, 20];
		var enable_pins = [18, 23, 25, 21];
		var directions = [1, 1, 1, 1];
		var tool_pins = [4];
		var COIL_A = 0;
		var COIL_B = 1;
		var NUM_OF_COIL = 2;
		var FIRE = 0;
		var step = [0, 0];
		var target_step = [0, 0];
		var direction = [-1, 1];
		var GEAR_RATIO = 4.6;
		var NUM_OF_PHASE = 4;
		var ROUND_STEP = 12 * NUM_OF_PHASE * GEAR_RATIO;

		var m_duty = 50;// %
		var m_offset_yaw = 0;
		var m_offset_pitch = 0;
		var m_fire_required = false;
		var m_fire_off_required = false;
		var m_firing = false;

		async
			.waterfall([
				function(callback) {// step timer
					// http://akizukidenshi.com/download/ds/sanyos/MDP-35A_a.pdf
					function setPwm(pin, duty, _fd) {
						var need_to_close;
						var fd;
						if (_fd == null) {
							fd = fs.openSync("/dev/pi-blaster", 'w');
							need_to_close = true;
						} else {
							fd = _fd;
							need_to_close = false;
						}
						fs.writeSync(fd, sprintf("%d=%.3f\n", pin, duty));
						if (need_to_close) {
							fs.closeSync(fd);
						}
					}
					function setPhase(idx, v, _fd) {
						var need_to_close;
						var fd;
						if (_fd == null) {
							fd = fs.openSync("/dev/pi-blaster", 'w');
							need_to_close = true;
						} else {
							fd = _fd;
							need_to_close = false;
						}
						if (v != 0) {
							setPwm(enable_pins[idx], m_duty / 100, fd);
							setPwm(phase_pins[idx], (v * directions[idx] > 0)
								? 1
								: 0, fd);
						} else {
							setPwm(enable_pins[idx], 0, fd);
						}
						if (need_to_close) {
							fs.closeSync(fd);
						}
					}
					function brake(md_id, phase) {
						setPhase(md_id * NUM_OF_COIL + COIL_A, 0);
						setPhase(md_id * NUM_OF_COIL + COIL_B, 0);
					}
					function phase4(md_id, phase) {
						switch (phase) {
							case 0 :
								setPhase(md_id * NUM_OF_COIL + COIL_A, 1);
								break;
							case 1 :
								setPhase(md_id * NUM_OF_COIL + COIL_B, 1);
								break;
							case 2 :
								setPhase(md_id * NUM_OF_COIL + COIL_A, -1);
								break;
							case 3 :
								setPhase(md_id * NUM_OF_COIL + COIL_B, -1);
								break;
						}
					}
					function phase8(md_id, phase) {
						switch (phase) {
							case 0 :
								setPhase(MD0_A, 0);
								break;
							case 1 :
								setPhase(MD0_A, 1);
								break;
							case 2 :
								setPhase(MD0_B, 0);
								break;
							case 3 :
								setPhase(MD0_B, 1);
								break;
							case 4 :
								setPhase(MD0_A, 0);
								break;
							case 5 :
								setPhase(MD0_A, -1);
								break;
							case 6 :
								setPhase(MD0_B, 0);
								break;
							case 7 :
								setPhase(MD0_B, -1);
								break;
						}
					}
					setInterval(function() {
						for (var i = 0; i < step.length; i++) {
							if (target_step[i] == step[i]) {
								brake(i);
								continue;
							} else if (target_step[i] > step[i]) {
								step[i]++;
							} else {
								step[i]--;
							}
							var phase = step[i] % NUM_OF_PHASE;
							if (phase < 0) {
								phase += NUM_OF_PHASE;
							}
							switch (NUM_OF_PHASE) {
								case 4 :
									phase4(i, phase);
									break;
								case 8 :
									phase8(i, phase);
									break;
							}
						}
						if (m_fire_required) {
							m_fire_required = false;
							m_firing = true;
							setPwm(tool_pins[FIRE], 1);
						}
						if (m_fire_off_required) {
							m_fire_off_required = false;
							m_firing = false;
							setPwm(tool_pins[FIRE], 0);
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
							// start from z axis, z axis direction is oposite
							y : Math.atan2(-mat4[0], -mat4[2]) * 180 / Math.PI,
							x : Math.asin(mat4[1]) * 180 / Math.PI,
							z : 0
						};
						var euler_xyz = toEulerianAngle(q, "YXZ");
						var next_taget_yaw = (euler_xy.x > -80)
							? euler_xy.y
							: euler_xyz.y;
						var base_step_yaw = Math.round(target_step[0]
							/ ROUND_STEP)
							* ROUND_STEP;
						var taget_yaw = direction[0]
							* (target_step[0] - base_step_yaw) / ROUND_STEP
							* 360;
						var diff_yaw = (next_taget_yaw + m_offset_yaw)
							- taget_yaw;
						while (true) {
							if (diff_yaw > 180) {
								diff_yaw -= 360;
							} else if (diff_yaw < -180) {
								diff_yaw += 360;
							} else {
								break;
							}
						}
						var diff_step_yaw = direction[0]
							* Math.round(diff_yaw / 360 * ROUND_STEP);

						var next_taget_pitch = (euler_xy.x > -80)
							? euler_xy.x
							: euler_xyz.x;
						var taget_pitch = direction[1]
							* (target_step[1] - target_step[0]) / ROUND_STEP
							* 360;
						var diff_pitch = (next_taget_pitch + m_offset_pitch)
							- taget_pitch;
						var diff_step_pitch = direction[1]
							* Math.round(diff_pitch / 360 * ROUND_STEP);

						target_step[0] += diff_step_yaw;
						target_step[1] += diff_step_yaw + diff_step_pitch;
						// console.log("taget_yaw:" + taget_yaw
						// + ",next_taget_yaw:" + next_taget_yaw
						// + ".target_step:" + target_step);
					}, 50);
					callback(null);
				}], function(err, result) {
			});

		var plugin = {
			name : "omni_laser",
			command_handler : function(cmd) {
				var split = cmd.split(' ');
				cmd = split[0].split('.')[1];
				switch (cmd) {
					case "fire" :
						m_fire_required = true;
						break;
					case "fire_off" :
						m_fire_off_required = true;
						break;
					case "increment_yaw" :
						m_offset_yaw += parseFloat(split[1]);
						break;
					case "increment_pitch" :
						m_offset_pitch += parseFloat(split[1]);
						break;
				}
			}
		};
		return plugin;
	}
};