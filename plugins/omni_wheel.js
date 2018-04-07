module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create nekojara plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;
		var phase_pins = [5, 12, 26, 16];
		var enable_pins = [6, 13, 27, 19];
		var directions = [1, 1, 1, 1];
		var MD0 = 0;
		var MD1 = 1;
		var MD2 = 2;
		var MD3 = 3;
		// 0 - 1
		// | |
		// 3 - 2

		var m_duty = 25;// %
		var m_move_time = 0;
		var m_move_speed = 10;// %
		var m_move_step = 20;// ms
		var m_remaining_steps = [0, 0];// float count of m_move_step
		var m_last_dir = 0;// -1 == g0, 0 == not designated, 1 == g1 dir

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
				setPwm(enable_pins[idx], m_duty/100, fd);
				setPwm(phase_pins[idx], (v * directions[idx] > 0) ? 1 : 0, fd);
			} else {
				setPwm(enable_pins[idx], 0, fd);
			}
			if (need_to_close) {
				fs.closeSync(fd);
			}
		}
		setInterval(function() {
			if (m_move_time <= 0) {
				m_remaining_steps[0] = 0;
				m_remaining_steps[1] = 0;
				m_last_dir = 0;
				setPhase(MD0, 0);
				setPhase(MD1, 0);
				setPhase(MD2, 0);
				setPhase(MD3, 0);
				return;
			}

			var q = plugin_host.get_view_quaternion();
			var euler_xyz = toEulerianAngle(q, "YXZ");

			var g0 = Math.sin(euler_xyz.y * Math.PI / 180);
			var g1 = Math.cos(euler_xyz.y * Math.PI / 180);

			m_remaining_steps[0] += g0;
			m_remaining_steps[1] += g1;

			var dir = 0;// 0 == not designated
			if (1 <= Math.abs(m_remaining_steps[0])
				&& 1 <= Math.abs(m_remaining_steps[1])) {
				dir = m_last_dir * -1;
				if (dir == 0)
					dir = -1;
			}

			if ((dir == 0 || dir == -1) && 1 <= m_remaining_steps[0]) {
				setPhase(MD1, 0);
				setPhase(MD3, 0);

				setPhase(MD0, 1);
				setPhase(MD2, -1);

				m_remaining_steps[0] -= 1;
				m_last_dir = -1;
			} else if ((dir == 0 || dir == -1) && m_remaining_steps[0] <= -1) {
				setPhase(MD1, 0);
				setPhase(MD3, 0);

				setPhase(MD0, -1);
				setPhase(MD2, 1);

				m_remaining_steps[0] += 1;
				m_last_dir = -1;
			} else {
				setPhase(MD0, 0);
				setPhase(MD2, 0);
			}

			if ((dir == 0 || dir == 1) && 1 <= m_remaining_steps[1]) {
				setPhase(MD0, 0);
				setPhase(MD2, 0);

				setPhase(MD1, 1);
				setPhase(MD3, -1);

				m_remaining_steps[1] -= 1;
				m_last_dir = 1;
			} else if ((dir == 0 || dir == 1) && m_remaining_steps[1] <= -1) {
				setPhase(MD0, 0);
				setPhase(MD2, 0);

				setPhase(MD1, -1);
				setPhase(MD3, 1);

				m_remaining_steps[1] += 1;
				m_last_dir = 1;
			} else {
				setPhase(MD1, 0);
				setPhase(MD3, 0);
			}

			m_move_time -= m_move_step;
		}, m_move_step);

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
			}
		};
		return plugin;
	}
};
