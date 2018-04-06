module.exports = {
	create_plugin : function(plugin_host) {
		console.log("create nekojara plugin");
		var async = require('async');
		var fs = require("fs");
		var sprintf = require('sprintf-js').sprintf;
		var pins = [ //
			// [ 17, 18, 22, 23, 27 ], // motor driver 1
			// [ 24, 25, 20, 21, 16 ], // motor driver 2
//			// [ 5, 6, 12, 13, 19 ], // motor driver 3
//			[17,18], // dc motor 0
//			[22,23], // dc motor 1
//			[24,25], // dc motor 2
//			[20,21] // dc motor 3
//			[5,6],
//			[12,13],
//			[26,27],
//			[16,19]
			[6,5],
			[13,12],
			[27,26],
			[19,16]
		];
		// 0 - 1
		// | |
		// 3 - 2
		
		var fd = fs.openSync("/dev/pi-blaster", 'w');
		
		var m_pwm_cycle = 2000;
		var m_us = m_pwm_cycle * 20 / 100;

		var m_move_time = 0;
		var m_move_speed = 10;// %
		var m_move_step = 20;// ms
		var m_remaining_steps = [0,0];//float count of m_move_step
		var m_last_dir = 0;//-1 == g0, 0 == not designated, 1 == g1 dir
		
		setInterval(function() {		
			if (m_move_time <= 0) {
				m_remaining_steps[0] = 0;
				m_remaining_steps[1] = 0;
				m_last_dir = 0;
				writeUs(fd, 0, 0, 0)
				writeUs(fd, 1, 0, 0)
				writeUs(fd, 2, 0, 0)
				writeUs(fd, 3, 0, 0)
				writeUs(fd, 0, 1, 0)
				writeUs(fd, 1, 1, 0)
				writeUs(fd, 2, 1, 0)
				writeUs(fd, 3, 1, 0)
				return;
			}

			var q = plugin_host.get_view_quaternion();
			var euler_xyz = toEulerianAngle(q, "YXZ");

			var g0 = Math.sin(euler_xyz.y*Math.PI/180);
			var g1 = Math.cos(euler_xyz.y*Math.PI/180);
			
			m_remaining_steps[0] += g0;
			m_remaining_steps[1] += g1;
			
			var dir = 0;// 0 == not designated
			if(1 <= Math.abs(m_remaining_steps[0]) && 1 <= Math.abs(m_remaining_steps[1]))
			{
				dir = m_last_dir * -1;
				if(dir == 0) dir = -1;
			}
			
			if((dir == 0 || dir == -1) && 1 <= m_remaining_steps[0]) {
				writeUs(fd, 1, 0, 0);
				writeUs(fd, 3, 0, 0);
				
				writeUs(fd, 0, 0, m_us);
				writeUs(fd, 0, 1, 0);
				writeUs(fd, 2, 0, m_us);
				writeUs(fd, 2, 1, m_pwm_cycle);
				
				m_remaining_steps[0] -= 1;
				m_last_dir = -1;
			}
			else if((dir == 0 || dir == -1) && m_remaining_steps[0] <= -1){
				writeUs(fd, 1, 0, 0);
				writeUs(fd, 3, 0, 0);
				
				writeUs(fd, 0, 0, m_us);
				writeUs(fd, 0, 1, m_pwm_cycle);
				writeUs(fd, 2, 0, m_us);
				writeUs(fd, 2, 1, 0);
				
				m_remaining_steps[0] += 1;
				m_last_dir = -1;
			}
			else{
				writeUs(fd, 0, 0, 0);
				writeUs(fd, 2, 0, 0);
			}
			
			if((dir == 0 || dir == 1) && 1 <= m_remaining_steps[1]) {
				writeUs(fd, 0, 0, 0);
				writeUs(fd, 2, 0, 0);
				
				writeUs(fd, 1, 0, m_us);
				writeUs(fd, 1, 1, 0);
				writeUs(fd, 3, 0, m_us);
				writeUs(fd, 3, 1, m_pwm_cycle);
				
				m_remaining_steps[1] -= 1;
				m_last_dir = 1;
			}
			else if((dir == 0 || dir == 1) && m_remaining_steps[1] <= -1){
				writeUs(fd, 0, 0, 0);
				writeUs(fd, 2, 0, 0);
				
				writeUs(fd, 1, 0, m_us);
				writeUs(fd, 1, 1, m_pwm_cycle);
				writeUs(fd, 3, 0, m_us);
				writeUs(fd, 3, 1, 0);		
				
				m_remaining_steps[1] += 1;
				m_last_dir = 1;
			}
			else{
				writeUs(fd, 1, 0, 0);
				writeUs(fd, 3, 0, 0);
			}
			
			m_move_time -= m_move_step;
		}, m_move_step);
		
		function writeUs(fd, i0, i1, us) {
			var v =  sprintf("%d=%dus\n", pins[i0][i1], us);
			fs.writeSync(fd, v);
		}

		// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
		function toEulerianAngle(quat, seq) {
			var q;// q0:w,q1:x,q2:y,q3:z, XYZ
			if (seq == "YXZ") {
				q = [ quat[3], quat[1], quat[0], quat[2] ];
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
				case "fire":
					m_fire_required = true;
					break;
				case "move":
				case "stop":
					m_move_time = parseFloat(split[1]);
					m_move_speed = parseFloat(split[2]);
					break;
				}
			}
		};
		return plugin;
	}
};
