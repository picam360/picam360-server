process.chdir(__dirname);
var os = require('os');
var disk = require('diskusage');
var child_process = require('child_process');
var async = require('async');
var fs = require("fs");
var express = require('express');
var moment = require("moment");
var sprintf = require('sprintf-js').sprintf;
var rtp = require("./rtp.js");
var rtcp = require("./rtcp.js");
var uuidv1 = require('uuid/v1');

var UPSTREAM_DOMAIN = "upstream.";
var SERVER_DOMAIN = "";
var CAPTURE_DOMAIN = UPSTREAM_DOMAIN;
var DRIVER_DOMAIN = UPSTREAM_DOMAIN + UPSTREAM_DOMAIN;
var PT_STATUS = 100;
var PT_CMD = 101;
var PT_FILE = 102;
var PT_CAM_BASE = 110;

function watchFile(filepath, oncreate, ondelete) {
	var fs = require('fs'), path = require('path'), filedir = path
		.dirname(filepath), filename = path.basename(filepath);
	fs.watch(filedir, function(event, who) {
		if (event === 'rename' && who === filename) {
			if (fs.existsSync(filepath)) {
				if (oncreate)
					oncreate();
			} else {
				if (ondelete)
					ondelete();
			}
		}
	});
}
function removeArray(array, value) {
	for (var i = array.length - 1; i >= 0; i--) {
		if (array[i] === value) {
			array.splice(i, 1);
		}
	}
}
function clone(src) {
	var dst = {}
	for ( var k in src) {
		dst[k] = src[k];
	}
	return dst;
}

var plugin_host = {};
var rtp_rx_watcher = [];
var cmd2upstream_list = [];
var cmd_list = [];
var watches = [];
var statuses = [];

var is_recording = false;
var framecount = 0;
var frame_duration = 0;
var last_frame_date = null;
var memoryusage_start = 0;
var GC_THRESH = 16 * 1024 * 1024;// 16MB
var capture_if;
var capture_process;

var http = null;

var options = JSON.parse(fs.readFileSync('config.json', 'utf8'));

async
	.waterfall([
		function(callback) {// exit sequence
			process.on('SIGINT', function() {
				console.log("exit process done");
				process.exit();
			});
			process.on('SIGUSR2', function() {
				if (agent.server) {
					agent.stop();
				} else {
					agent.start({
						port : 9999,
						bind_to : '192.168.3.103',
						ipc_port : 3333,
						verbose : true
					});
				}
			});
			callback(null);
		},
		function(callback) {
			console.log("init data stream");

			var rtcp_command_id = 0;
			var active_frame = null;
			var startTime = new Date();
			var num_of_frame = 0;
			var fps = 0;

			function rtp_passthrough(pack_list) {
				rtp_rx_watcher.forEach(function(watcher) {
					if (watcher.active_frame_count < 5) {
						if (watcher.ws) {
							watcher.skip_count = 0;
							watcher.active_frame_count++;
							rtp.sendpacket(watcher.ws, pack_list, function(
								value) {
								if (rtp_rx_watcher
									&& rtp_rx_watcher[0] == watcher && value
									&& value.startsWith(UPSTREAM_DOMAIN)) {
									cmd2upstream_list.push(value
										.substr(UPSTREAM_DOMAIN.length));
								}
								watcher.active_frame_count--;
							});
						} else if (watcher.conn) {
							rtp.sendpacket(watcher.conn, pack_list);
						}
					} else if (watcher.skip_count > 1000) {// 100sec
						// remove
						console.log("timeout remove rtp rx watcher:"
							+ watcher.ip);
						removeArray(rtp_rx_watcher, watcher);
					} else {
						watcher.skip_count++;
					}
				});
			}

			// image from upstream
			rtp
				.set_callback(9004, function(pack) {
					if (pack.GetPayloadType() == 110) {
						var data_len = pack.GetPacketLength();
						var header_len = pack.GetHeaderLength();
						var data = pack.GetPacketData();
						if (!active_frame) {
							if ((data[header_len] == 0xFF && data[header_len + 1] == 0xD8)
								|| (data[header_len] == 0x4E && data[header_len + 1] == 0x41)) { // SOI
								active_frame = [];
								// console.log("new_frame");
							}
						}
						if (active_frame) {
							active_frame.push(data);
							if ((data[data_len - 2] == 0xFF && data[data_len - 1] == 0xD9)
								|| (data[data_len - 2] == 0x4C && data[data_len - 1] == 0x55)) { // EOI
								// image to downstream
								rtp_passthrough(active_frame);
								active_frame = null;
								// console.log("active_frame");
								num_of_frame++;
								var endTime = new Date();
								var diff_sec = (endTime - startTime) / 1000;
								var frame_sec = (((fps != 0) ? 1.0 / fps : 0) * 0.9 + diff_sec * 0.1);
								fps = (frame_sec != 0) ? 1.0 / frame_sec : 0;
								// console.log("fps : " + fps);

								startTime = new Date();
							}
						}
					}
				});
			// cmd from downstream
			rtcp.set_callback(function(pack) {
				if (pack.GetPayloadType() == 101) {
					var cmd = pack.GetPacketData().toString('ascii', pack
						.GetHeaderLength());
					var split = cmd.split('\"');
					var id = split[1];
					var value = split[3];
					plugin_host.send_command(value);
					if (options.debug >= 5) {
						console.log("cmd got :" + cmd);
					}
				}
			});
			// cmd to upstream
			setInterval(function() {
				if (cmd2upstream_list.length) {
					var value = cmd2upstream_list.shift();
					var cmd = "<picam360:command id=\"" + rtcp_command_id
						+ "\" value=\"" + value + "\" />";
					var pack = rtcp.build_packet(new Buffer(cmd, 'ascii'), 101);
					rtcp.sendpacket(pack, 9005, "127.0.0.1");

					rtcp_command_id++;
				}
			}, 20);
			// status to downstream
			setInterval(function() {
				var pack_list = [];
				for ( var name in statuses) {
					if (statuses[name]) {
						var value = statuses[name]();
						var status = "<picam360:status name=\"" + name
							+ "\" value=\"" + value + "\" />";
						var pack = rtp
							.build_packet(new Buffer(status, 'ascii'), 100);
						pack_list.push(pack);
					}
				}
				if (pack_list.length > 0) {
					rtp_passthrough(pack_list);
				}
			}, 1000);
			callback(null);
		},
		function(callback) {// gc
			console.log("gc");
			var disk_free = 0;
			setInterval(function() {
				disk.check('/tmp', function(err, info) {
					disk_free = info.available;
				});
			}, 1000);
			setInterval(function() {
				if (global.gc && os.freemem() < GC_THRESH) {
					console.log("gc : free=" + os.freemem() + " usage="
						+ process.memoryUsage().rss);
					console.log("disk_free=" + disk_free);
					global.gc();
				}
			}, 100);
			callback(null);
		},
		function(callback) {// start up websocket server
			console.log("websocket server starting up");
			var app = require('express')();
			http = require('http').Server(app);
			app
				.get('/img/*.jpeg', function(req, res) {
					var url = req.url.split("?")[0];
					var query = req.url.split("?")[1];
					var filepath = 'userdata/' + url.split("/")[2];
					console.log(url);
					console.log(query);
					console.log(filepath);
					fs
						.readFile(filepath, function(err, data) {
							if (err) {
								res.writeHead(404);
								res.end();
								console.log("404");
							} else {
								res
									.writeHead(200, {
										'Content-Type' : 'image/jpeg',
										'Content-Length' : data.length,
										'Cache-Control' : 'private, no-cache, no-store, must-revalidate',
										'Expires' : '-1',
										'Pragma' : 'no-cache',
									});
								res.end(data);
								console.log("200");
							}
						});
				});
			app.get('/img/*.mp4', function(req, res) {
				var url = req.url.split("?")[0];
				var query = req.url.split("?")[1];
				var filepath = 'userdata/' + url.split("/")[2];
				console.log(url);
				console.log(query);
				console.log(filepath);
				fs.readFile(filepath, function(err, data) {
					if (err) {
						res.writeHead(404);
						res.end();
						console.log("404");
					} else {
						var range = req.headers.range // bytes=0-1
						if (!range) {
							res.writeHead(200, {
								"Content-Type" : "video/mp4",
								"X-UA-Compatible" : "IE=edge;chrome=1",
								'Content-Length' : data.length
							});
							res.end(data)
						} else {
							var total = data.length;
							var split = range.split(/[-=]/);
							var ini = +split[1];
							var end = split[2] ? +split[2] : total - 1;
							var chunkSize = end - ini + 1;
							res.writeHead(206, {
								"Content-Range" : "bytes " + ini + "-" + end
									+ "/" + total,
								"Accept-Ranges" : "bytes",
								"Content-Length" : chunkSize,
								"Content-Type" : "video/mp4",
							})
							res.end(data.slice(ini, chunkSize + ini))
						}
					}
				});
			});
			app.use(express.static('www'));// this need be set
			http.listen(9001, function() {
				console.log('listening on *:9001');
			});
			callback(null);
		},
		function(callback) {
			// websocket
			var io = require("socket.io").listen(http);
			io.sockets
				.on("connection", function(socket) {
					var ip = socket.request.headers['x-forwarded-for']
						|| socket.request.connection.remoteAddress;
					if (rtp_rx_watcher.length >= 2) {// exceed client
						console.log("exceeded_num_of_clients:" + ip);
						socket.emit("custom_error", "exceeded_num_of_clients");
						return;
					}
					var watcher = {
						ip : ip,
						ws : socket,
						active_frame_count : 0,
						skip_count : 0
					};
					rtp_rx_watcher.push(watcher);
					rtcp.add_websocket(socket);
					console.log("add rtp rx watcher:" + watcher.ip);
					socket.on("connected", function() {
					});
					socket.on("snap", function(callback) {
						var filename = moment().format('YYYYMMDD_hhmmss')
							+ '.jpeg';
						var cmd = CAPTURE_DOMAIN + 'snap -0 -o /tmp/'
							+ filename;
						plugin_host.send_command(cmd);
						console.log(cmd);
						watchFile('/tmp/' + filename, function() {
							console.log(filename + ' saved.');
							var rm_cmd = 'mv' + ' /tmp/' + filename
								+ ' userdata/' + filename;
							var cmd = rm_cmd;
							console.log(cmd);
							child_process.exec(cmd, function() {
								callback(filename);
							});
						});
					});
					socket.on("start_record", function(duration) {
						if (is_recording)
							return;
						var cmd = CAPTURE_DOMAIN
							+ 'start_record -0 -o /tmp/movie.h264';
						plugin_host.send_command(cmd);
						console.log(cmd);
						// console.log("camera
						// recording
						// start
						// duration="
						// +
						// duration);
						is_recording = true;
						frame_duration = duration;
						last_frame_date = null;
					});
					socket
						.on("stop_record", function(callback) {
							is_recording = false;
							var cmd = CAPTURE_DOMAIN + 'stop_record -0';
							plugin_host.send_command(cmd);
							console.log(cmd);
							var filename = moment().format('YYYYMMDD_hhmmss')
								+ '.mp4';
							var ffmpeg_cmd = 'ffmpeg -y -r 10 -i /tmp/movie.h264 -c:v copy userdata/'
								+ filename;
							var delh264_cmd = 'rm /tmp/movie.h264';
							var cmd = ffmpeg_cmd + ' ; ' + delh264_cmd;
							console.log(cmd);
							child_process.exec(cmd, function() {
								callback(filename);
							});
						});
					socket
						.on("snap_e", function(callback) {
							var filename = moment().format('YYYYMMDD_hhmmss')
								+ '.jpeg';
							var cmd = CAPTURE_DOMAIN
								+ 'snap -E -W 3072 -H 1536 -o /tmp/' + filename;
							plugin_host.send_command(cmd);
							console.log(cmd);
							watchFile('/tmp/' + filename, function() {
								console.log(filename + ' saved.');
								var exiftool_cmd = 'exiftool -ProjectionType="equirectangular" -o'
									+ ' userdata/'
									+ filename
									+ ' /tmp/'
									+ filename;
								var rm_cmd = 'rm' + ' /tmp/' + filename;
								var cmd = exiftool_cmd + ' && ' + rm_cmd;
								console.log(cmd);
								child_process.exec(cmd, function() {
									callback(filename);
								});
							});
						});
					socket
						.on("start_record_e", function(duration) {
							if (is_recording)
								return;
							duration = (duration == null) ? 0 : duration;
							var cmd = 'start_record -E -W 1024 -H 512 -o /tmp/movie.h264\n';
							capture_if.write(cmd);
							console.log(cmd);
							// console.log("camera
							// recording
							// start
							// duration="
							// +
							// duration);
							is_recording = true;
							frame_duration = duration;
							last_frame_date = null;
						});
					socket
						.on("stop_record_e", function(callback) {
							is_recording = false;
							capture_if.write('stop_record\n');
							console.log("camera recording stop");
							var filename = moment().format('YYYYMMDD_hhmmss')
								+ '.mp4';
							var ffmpeg_cmd = 'ffmpeg -y -r 5 -i /tmp/movie.h264 -c:v copy userdata/'
								+ filename;
							var delh264_cmd = 'rm /tmp/movie.h264';
							var spatialmedia_cmd = 'python submodules/spatial-media/spatialmedia -i userdata/'
								+ filename + ' /tmp/vr.mp4';
							// var cmd =
							// ffmpeg_cmd
							// + ' && '
							// +
							// delh264_cmd
							// + ' && '
							// +
							// spatialmedia_cmd;
							var cmd = ffmpeg_cmd + ' ; ' + delh264_cmd;
							console.log(cmd);
							child_process.exec(cmd, function() {
								callback(filename);
							});
						});
					socket.on("disconnect", function() {
						console.log("remove rtp rx watcher:" + watcher.ip);
						removeArray(rtp_rx_watcher, watcher);
					});
					socket.on("error", function(event) {
						console.log("error : " + event);
					});
				});
			callback(null);
		},
		function(callback) {
			// wrtc
			if (options["wrtc_enabled"]) {
				var P2P_API_KEY = "v8df88o1y4zbmx6r";
				global.Blob = "blob";
				global.File = "file";
				global.WebSocket = require("ws");
				global.window = require("wrtc");
				global.window.postMessage = function(message, origin) {
					console.log(message);
				};
				var uuid = options["wrtc_uuid"] || uuidv1();
				console.log("\n\n\n");
				console.log("webrtc uuid : " + uuid);
				console.log("\n\n\n");
				var Peer = require("peerjs");
				var peer = new Peer(uuid, {
					key : P2P_API_KEY,
					debug : options.debug || 0
				});

				peer.on('connection', function(conn) {
					console.log("\n\n\n");
					console.log("p2p connection established as upstream.");
					console.log("\n\n\n");

					var watcher = {
						ip : "conn",
						conn : conn,
						active_frame_count : 0,
						skip_count : 0
					};
					rtp_rx_watcher.push(watcher);
					rtcp.add_peerconnection(conn);
				});
			}
			callback(null);
		},
		function(callback) {
			// plugin host
			function rtp_sendpacket(pack_list) {
				rtp_rx_watcher.forEach(function(watcher) {
					if (watcher.active_frame_count < 5) {
						if (watcher.ws) {
							watcher.skip_count = 0;
							watcher.active_frame_count++;
							rtp.sendpacket(watcher.ws, pack_list, function(
								value) {
								if (rtp_rx_watcher
									&& rtp_rx_watcher[0] == watcher && value
									&& value.startsWith(UPSTREAM_DOMAIN)) {
									cmd2upstream_list.push(value
										.substr(UPSTREAM_DOMAIN.length));
								}
								watcher.active_frame_count--;
							});
						} else if (watcher.conn) {
							rtp.sendpacket(watcher.conn, pack_list);
						}
					} else if (watcher.skip_count > 1000) {// 100sec
						// remove
						console.log("timeout remove rtp rx watcher:"
							+ watcher.ip);
						removeArray(rtp_rx_watcher, watcher);
					} else {
						watcher.skip_count++;
					}
				});
			}
			// cmd handling
			function command_handler(value) {
				var split = value.split(' ');
				if (split[0] == "snap") {
					var filename = moment().format('YYYYMMDD_hhmmss') + '.jpeg';
					var filepath = 'userdata/' + filename;
					var cmd = CAPTURE_DOMAIN + 'snap -0 -o /tmp/' + filename;
					plugin_host.send_command(cmd);
					console.log(cmd);
					watchFile('/tmp/' + filename, function() {
						console.log(filename + ' saved.');
						var rm_cmd = 'mv' + ' /tmp/' + filename + ' '
							+ filepath;
						var cmd = rm_cmd;
						console.log(cmd);
						child_process
							.exec(cmd, function() {
								// callback(filename);
								fs
									.readFile(filepath, function(err, data) {
										if (err) {
											console.log("not found :"
												+ filepath);
										} else {
											rtp_sendpacket(build_packet(data, PT_FILE));
											console.log("send :" + filepath);
										}
									});
							});
					});
				} else if (split[0] == "start_record") {
					if (is_recording)
						return;
					var cmd = CAPTURE_DOMAIN
						+ 'start_record -0 -o /tmp/movie.h264';
					plugin_host.send_command(cmd);
					console.log(cmd);
					is_recording = true;
					frame_duration = duration;
					last_frame_date = null;
				} else if (split[0] == "stop_record") {
					is_recording = false;
					var cmd = CAPTURE_DOMAIN + 'stop_record -0';
					plugin_host.send_command(cmd);
					console.log(cmd);
					var filename = moment().format('YYYYMMDD_hhmmss') + '.mp4';
					var filepath = 'userdata/' + filename;
					var ffmpeg_cmd = 'ffmpeg -y -r 10 -i /tmp/movie.h264 -c:v copy '
						+ filepath;
					var delh264_cmd = 'rm /tmp/movie.h264';
					var cmd = ffmpeg_cmd + ' ; ' + delh264_cmd;
					console.log(cmd);
					child_process.exec(cmd, function() {
						fs.readFile(filepath, function(err, data) {
							if (err) {
								console.log("not found :" + filepath);
							} else {
								rtp_sendpacket(build_packet(data, PT_FILE));
								console.log("send :" + filepath);
							}
						});
					});
				}
			}
			setInterval(function() {
				if (cmd_list.length) {
					var value = cmd_list.shift();
					command_handler(value);
				}
			}, 20);
			plugin_host.send_command = function(value) {
				if (value.startsWith(UPSTREAM_DOMAIN)) {
					cmd2upstream_list
						.push(value.substr(UPSTREAM_DOMAIN.length));
				} else {
					cmd_list.push(value);
				}
			};
			plugin_host.add_watch = function(name, callback) {
				watches[name] = callback;
			};
			plugin_host.add_status = function(name, callback) {
				statuses[name] = callback;
			};

			plugin_host.add_status("is_recording", function() {
				return is_recording;
			});

			callback(null);
		}], function(err, result) {
	});
