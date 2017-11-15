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
				function(callback) {// timer
					var phase = 0;
					setInterval(function() {
						var diff_step = target_step - step;
						if (diff_step == 0) {
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
							step++;
						} else if (phase > 3) {
							phase = 0;
							step--;
						}
					}, 20);
					callback(null);
				}], function(err, result) {
			});

		var plugin = {

		};
		return plugin;
	}
};