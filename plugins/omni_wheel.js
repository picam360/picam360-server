module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create nekojara plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;
		var phase_pins = [5, 6, 26, 16];
		var enable_pins = [12, 13, 27, 19];
		var directions = [1, 1, 1, 1];
		var inversions = [0, 0, 1, 1];
		var MD0 = 0;
		var MD1 = 1;
		var MD2 = 2;
		var MD3 = 3;
		// 0 - 1
		// | |
		// 3 - 2
		
		var MD_PAIR_NA = -1;
		var MD_PAIR_0 = 0;// MD0 and MD2
		var MD_PAIR_1 = 1;// MD1 and MD3

		var DIR_NA = -1;
		var DIR0 = 0;
		var DIR1 = 1;
		var DIR2 = 2;
		var DIR3 = 3;

		var m_duty = 25;// %
		var m_view_yaw_offset = 0;
		var m_low_duty_ratio = 65;// %
		var m_low_duty_thresh_time = 40;// ms
		var m_move_time = 0;
		var m_move_speed = 10;// % (Not being used)
		var m_move_step = 20;// ms
		var m_last_dir = DIR_NA;
		var m_last_dir_cnt = 0;// last dir continuous count
		var m_moved_step_counts = [0, 0];
		var m_step_ramains = [0, 0];
		var m_last_step_ratios = [0, 0];

		for (var i = 0; i < 4; i++) {
			invertPwm(i, inversions[i]);
		}
		function invertPwm(idx, invert, _fd) {
			var need_to_close;
			var fd;
			if (_fd == null) {
				fd = fs.openSync("/dev/pi-blaster", 'w');
				need_to_close = true;
			} else {
				fd = _fd;
				need_to_close = false;
			}
			fs
				.writeSync(fd, sprintf("invert %d=%d\n", enable_pins[idx], invert));
			if (need_to_close) {
				fs.closeSync(fd);
			}
		}
		function setPwm(idx, _duty, _fd) {
			var need_to_close;
			var fd;
			var duty = inversions[idx] ? (1.0 - _duty) : _duty;
			if (_fd == null) {
				fd = fs.openSync("/dev/pi-blaster", 'w');
				need_to_close = true;
			} else {
				fd = _fd;
				need_to_close = false;
			}
			fs.writeSync(fd, sprintf("%d=%.3f\n", enable_pins[idx], duty));
			if (need_to_close) {
				fs.closeSync(fd);
			}
		}
		function setPhase(idx, v, duty_ratio, _fd) {
			var need_to_close;
			var fd;
			var phase = (v * directions[idx] > 0) ? 1 : 0;
			if (_fd == null) {
				fd = fs.openSync("/dev/pi-blaster", 'w');
				need_to_close = true;
			} else {
				fd = _fd;
				need_to_close = false;
			}
			if (v != 0) {
				setPwm(idx, m_duty / 100 * duty_ratio / 100, fd);
				fs.writeSync(fd, sprintf("%d=%.3f\n", phase_pins[idx], phase));
			} else {
				setPwm(idx, 0, fd);
			}
			if (need_to_close) {
				fs.closeSync(fd);
			}
		}
		function stop(pair_idx = DIR_NA) {
			if (pair_idx == MD_PAIR_0) {
				setPhase(MD0, 0, 100);
				setPhase(MD2, 0, 100);
			} else if (pair_idx == MD_PAIR_1) {
				setPhase(MD1, 0, 100);
				setPhase(MD3, 0, 100);
			} else {
				setPhase(MD0, 0, 100);
				setPhase(MD1, 0, 100);
				setPhase(MD2, 0, 100);
				setPhase(MD3, 0, 100);
			}
		}
		function move(dir_idx, ratio = 100) {
			if (dir_idx == DIR0) {
				stop(MD_PAIR_1);			
				setPhase(MD0, 1, ratio);
				setPhase(MD2, -1, ratio);
			} else if (dir_idx == DIR1) {
				stop(MD_PAIR_1);
				setPhase(MD0, -1, ratio);
				setPhase(MD2, 1, ratio);
			} else if (dir_idx == DIR2) {
				stop(MD_PAIR_0);
				setPhase(MD1, 1, ratio);
				setPhase(MD3, -1, ratio);
			} else if (dir_idx == DIR3) {
				stop(MD_PAIR_0);
				setPhase(MD1, -1, ratio);
				setPhase(MD3, 1, ratio);
			} 
		}
		
		setInterval(function() {
			if (m_move_time <= 0) {
				m_last_dir = DIR_NA;
				m_last_dir_cnt = 0;
				m_moved_step_counts[MD_PAIR_0] = 0;
				m_moved_step_counts[MD_PAIR_1] = 0;
				m_last_step_ratios[MD_PAIR_0] = 0;
				m_last_step_ratios[MD_PAIR_1] = 0;
				m_step_ramains[MD_PAIR_0] = 0;
				m_step_ramains[MD_PAIR_1] = 0;
				stop();
				return;
			}

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
			var view_yaw = (euler_xy.x > -80) ? euler_xy.y : euler_xyz.y;
			view_yaw += m_view_yaw_offset;

			var step_ratios = [0, 0];
			step_ratios[MD_PAIR_0] = Math.sin(view_yaw * Math.PI / 180);
			step_ratios[MD_PAIR_1] = Math.cos(view_yaw * Math.PI / 180);
			var absMax = Math.max(Math.abs(step_ratios[MD_PAIR_0]), Math.abs(step_ratios[MD_PAIR_1]));
			step_ratios[MD_PAIR_0] /= absMax;
			step_ratios[MD_PAIR_1] /= absMax;
			
			// Reset when direction reversed.
			if(Math.abs(m_last_step_ratios[MD_PAIR_0] * step_ratios[MD_PAIR_0]) < 0){
				m_moved_step_counts[MD_PAIR_0] = 0;
				m_step_ramains[MD_PAIR_0] = 0;
			}
			if(Math.abs(m_last_step_ratios[MD_PAIR_1] * step_ratios[MD_PAIR_1]) < 0){
				m_moved_step_counts[MD_PAIR_1] = 0;
				m_step_ramains[MD_PAIR_1] = 0;
			}
			m_last_step_ratios[MD_PAIR_0] = step_ratios[MD_PAIR_0];
			m_last_step_ratios[MD_PAIR_1] = step_ratios[MD_PAIR_1];
			
			var next_dir = DIR_NA;
			function SetNextDirFromPair0() {
				var v = Math.abs(step_ratios[MD_PAIR_0]) * m_moved_step_counts[MD_PAIR_1] + m_step_ramains[MD_PAIR_0];
				if(v >= 1){
					if(step_ratios[MD_PAIR_0] >= 0){
						next_dir = DIR0;
					}else{
						next_dir = DIR1;
					}	
					m_step_ramains[MD_PAIR_0] = (v-1);
					m_moved_step_counts[MD_PAIR_0] = 1;
					m_moved_step_counts[MD_PAIR_1] = 0;
				}
			}
			function SetNextDirFromPair1() {
				var v = Math.abs(step_ratios[MD_PAIR_1]) * m_moved_step_counts[MD_PAIR_0] + m_step_ramains[MD_PAIR_1];
				if(v >= 1){
					if(step_ratios[MD_PAIR_1] >= 0){
						next_dir = DIR2;
					}else{
						next_dir = DIR3;
					}
					m_step_ramains[MD_PAIR_1] = (v-1);
					m_moved_step_counts[MD_PAIR_0] = 0;
					m_moved_step_counts[MD_PAIR_1] = 1;
				}
			}
			
			// Get next dir.
			if(m_last_dir == DIR0 || m_last_dir == DIR1){
				if(next_dir == DIR_NA) SetNextDirFromPair1();
				if(next_dir == DIR_NA) SetNextDirFromPair0();
				
			}else if(m_last_dir == DIR2 || m_last_dir == DIR3){
				if(next_dir == DIR_NA) SetNextDirFromPair0();
				if(next_dir == DIR_NA) SetNextDirFromPair1();
			}
			if(next_dir == DIR_NA){
				if(Math.abs(step_ratios[MD_PAIR_0]) >= Math.abs(step_ratios[MD_PAIR_1])){
					if(step_ratios[MD_PAIR_0] >= 0){
						next_dir = DIR0;
					}else{
						next_dir = DIR1;
					}
					m_moved_step_counts[MD_PAIR_0] += 1;
				}else{
					if(step_ratios[MD_PAIR_1] >= 0){
						next_dir = DIR2;
					}else{
						next_dir = DIR3;
					}
					m_moved_step_counts[MD_PAIR_1] += 1;
				}
			}
			
			// Move.
			if(m_last_dir == next_dir && m_low_duty_thresh_time <= m_move_step * m_last_dir_cnt){
				move(next_dir, m_low_duty_ratio);
			}else{
				move(next_dir);
			}
			if(m_last_dir == next_dir){
				m_last_dir_cnt++;
			}else{
				m_last_dir_cnt = 1;
			}	
			m_last_dir = next_dir;
			
			m_move_time -= m_move_step;
		}, m_move_step);

		function mat4_fromQuat(q) {
			var out = [];
			var x = q[0], y = q[1], z = q[2], w = q[3], x2 = x + x, y2 = y + y, z2 = z
				+ z,

			xx = x * x2, yx = y * x2, yy = y * y2, zx = z * x2, zy = z * y2, zz = z
				* z2, wx = w * x2, wy = w * y2, wz = w * z2;

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

		var plugin = {
			name : "omni_wheel",
			command_handler : function(cmd) {
				var split = cmd.split(' ');
				cmd = split[0].split('.')[1];
				switch (cmd) {
					case "fire" :
						m_fire_required = true;
						break;
					case "move" :
					case "stop" :
						m_move_time = parseFloat(split[1]);
						m_move_speed = parseFloat(split[2]);
						break;
				}
			},
			init_options : function(options) {
				if (options[plugin.name + ".duty"]) {
					m_duty = options[plugin.name + ".duty"];
				}
				if (options[plugin.name + ".view_yaw_offset"]) {
					m_view_yaw_offset = options[plugin.name
						+ ".view_yaw_offset"];
				}
			}
		};
		return plugin;
	}
};
