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
var rtp_rx_watcher = [];
var primary_socket = null;

var recording = false;
var framecount = 0;
var frame_duration = 0;
var last_frame_date = null;
var memoryusage_start = 0;
var GC_THRESH = 16 * 1024 * 1024;// 16MB
var capture_if;
var capture_process;
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

			var UPSTREAM_DOMAIN = "upstream.";
			var cmd2upstream_list = [];
			var rtcp_command_id = 0;
			var active_frame = null;
			var startTime = new Date();
			var num_of_frame = 0;
			var fps = 0;

			rtp
				.set_callback(9004, function(pack) {
					if (pack.GetPayloadType() == 110) {
						var data_len = pack.GetPacketLength();
						var header_len = pack.GetHeaderLength();
						var data = pack.GetPacketData();
						if (!active_frame) {
							if (data[header_len] == 0xFF
								&& data[header_len + 1] == 0xD8) { // SOI
								active_frame = [];
								// console.log("new_frame");
							}
						}
						if (active_frame) {
							active_frame.push(data);
							if (data[data_len - 2] == 0xFF
								&& data[data_len - 1] == 0xD9) { // EOI
								rtp_rx_watcher
									.forEach(function(watcher) {
										if (watcher.active_frame_count < 5) {
											watcher.skip_count = 0;
											watcher.active_frame_count++;
											rtp
												.sendpacket(watcher.ws, active_frame, function(
													value) {
													if (watcher.ws == primary_socket
														&& value
														&& value
															.startsWith(UPSTREAM_DOMAIN)) {
														cmd2upstream_list
															.push(value
																.substr(UPSTREAM_DOMAIN.length));
													}
													watcher.active_frame_count--;
												});
										} else {
											watcher.skip_count++;
										}
									});
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
			rtcp.set_callback(function(pack) {
				if (pack.GetPayloadType() == 101) {
					var cmd = pack.GetPacketData().toString('ascii', pack
						.GetHeaderLength());
					var split = cmd.split('\"');
					var id = split[1];
					var value = split[3];
					if (value.startsWith(UPSTREAM_DOMAIN)) {
						cmd2upstream_list.push(value
							.substr(UPSTREAM_DOMAIN.length));
					}
				}
			});
			setInterval(function() {
				if (cmd2upstream_list.length) {
					var value = cmd2upstream_list.shift();
					var cmd = "<picam360:command id=\"" + rtcp_command_id
						+ "\" value=\"" + value + "\" />";
					rtcp
						.sendpacket(new Buffer(cmd, 'ascii'), 101, 9005, "127.0.0.1");

					rtcp_command_id++;
				}
			}, 20);
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
			var http = require('http').Server(app);
			var io = require("socket.io").listen(http);
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
			// after all dynamic
			// files
			function clone(src) {
				var dst = {}
				for ( var k in src) {
					dst[k] = src[k];
				}
				return dst;
			}
			io.sockets
				.on("connection", function(socket) {
					var watcher = {
						ws : socket,
						active_frame_count : 0,
						skip_count : 0
					};
					primary_socket = socket;
					rtp_rx_watcher.push(watcher);
					rtcp.add_websocket(socket);
					console.log("add rtp rx watcher");
					socket.on("connected", function() {
					});
					socket
						.on("snap", function(callback) {
							var filename = moment().format('YYYYMMDD_hhmmss')
								+ '.jpeg';
							var cmd = 'snap -E -W 3072 -H 1536 -o /tmp/'
								+ filename + '\n';
							capture_if.write(cmd);
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
						.on("startRecord", function(duration) {
							if (recording)
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
							recording = true;
							frame_duration = duration;
							last_frame_date = null;
						});
					socket
						.on("stopRecord", function(callback) {
							recording = false;
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
					});
				});
			http.listen(9001, function() {
				console.log('listening on *:9001');
			});
			callback(null);
		}], function(err, result) {
	});