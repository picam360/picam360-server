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

var SIGNALING_HOST = "peer.picam360.com";
var SIGNALING_PORT = 443;
var SIGNALING_SECURE = true;

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
var filerequest_list = [];

var upstream_next_frame_id = 0;
var is_recording = false;
var frame_duration = 0;
var last_frame_date = null;
var memoryusage_start = 0;
var GC_THRESH = 16 * 1024 * 1024;// 16MB
var capture_if;
var capture_process;
var request_call = "";

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

			rtp.add_watcher = function(conn) {
				var ip;
				if (conn.peerConnection) { // webrtc
					ip = " via webrtc";
				} else {
					ip = (conn.request.headers['x-forwarded-for'] || conn.request.connection.remoteAddress)
						+ " via websocket";
				}
				if (rtp_rx_watcher.length >= 2) {// exceed client
					console.log("exceeded_num_of_clients : " + ip);
					// TODO:need to send error
					// conn.emit("custom_error", "exceeded_num_of_clients");
					return;
				} else {
					console.log("connection opend : " + ip);
				}

				var watcher = {
					ip : ip,
					conn : conn,
					frame_queue : [],
					fps : 5,
					ttl : 1.0,
					min_ttl : 1.0,
					frame_num : 0,
					tmp_num : 0,
					tmp_time : 0,
					frame_id : upstream_next_frame_id,
					timeout : false
				};
				conn.on('data', function(data) {
					watcher.timeout = false;
				});

				rtcp.add_connection(conn);
				rtp_rx_watcher.push(watcher);

				plugin_host
					.send_command(UPSTREAM_DOMAIN
						+ "create_frame -P -w 640 -h 640 -s h264 -f 5 -k 800", conn);

				watcher.timer = setInterval(function() {
					if (watcher.timeout) {
						console.log("timeout");
						if (conn.peerConnection) { // webrtc
							conn.close();
						} else {
							conn.disconnect(true);
						}
					} else {
						watcher.timeout = true;
					}
				}, 10000);
			}

			rtp.remove_watcher = function(conn) {
				for (var i = rtp_rx_watcher.length - 1; i >= 0; i--) {
					if (rtp_rx_watcher[i].conn === conn) {
						console.log("connection closed : "
							+ rtp_rx_watcher[i].ip);
						clearInterval(rtp_rx_watcher[i].timer);

						plugin_host
							.send_command(UPSTREAM_DOMAIN + "delete_frame -i "
								+ rtp_rx_watcher[i].frame_id, conn);
						rtp_rx_watcher.splice(i, 1);
					}
				}
			}

			rtp.get_frame_id = function(conn) {
				for (var i = rtp_rx_watcher.length - 1; i >= 0; i--) {
					if (rtp_rx_watcher[i].conn === conn) {
						return rtp_rx_watcher[i].frame_id;
					}
				}
			}

			rtp.get_conn = function(frame_id) {
				for (var i = rtp_rx_watcher.length - 1; i >= 0; i--) {
					if (rtp_rx_watcher[i].frame_id == frame_id) {
						return rtp_rx_watcher[i].conn;
					}
				}
			}

			rtp._sendpacket = function(pack_list, conn) {
				for (var i = rtp_rx_watcher.length - 1; i >= 0; i--) {
					var watcher = rtp_rx_watcher[i];
					if (conn && conn != watcher.conn) {
						continue;
					}
					rtp.sendpacket(watcher.conn, pack_list);
				}
			}

			rtp.push_frame_queue = function(server_key, conn) {
				var now = new Date().getTime();
				for (var i = rtp_rx_watcher.length - 1; i >= 0; i--) {
					var watcher = rtp_rx_watcher[i];
					if (watcher.conn === conn) {
						// to minimize ttl
						// and miximize fps
						var target_fps = watcher.fps + 1;
						var target_ttl = (watcher.ttl + watcher.min_ttl) / 2;
						var limit = Math.ceil(target_fps * target_ttl);
						if (watcher.frame_queue.length > limit) {
							return false;
						} else {
							watcher.frame_queue.push({
								server_key : server_key,
								base_time : now
							});
							return true;
						}
					}
				}
				return false;
			}

			rtp.pop_frame_queue = function(server_key, conn) {
				var now = new Date().getTime();
				for (var i = rtp_rx_watcher.length - 1; i >= 0; i--) {
					var watcher = rtp_rx_watcher[i];
					if (watcher.conn === conn) {
						for (var j = 0; j < watcher.frame_queue.length; j++) {
							if (watcher.frame_queue[j].server_key == server_key) {
								watcher.frame_num++;
								{// ttl
									var value = (now - watcher.frame_queue[j].base_time) / 1000.0;
									watcher.ttl = watcher.ttl * 0.9 + value
										* 0.1;
									if (watcher.ttl < watcher.min_ttl) {
										watcher.min_ttl = watcher.ttl;
									}
								}
								{// fps
									if (watcher.tmp_time == 0) {
										watcher.tmp_time = now;
									} else if (now - watcher.tmp_time > 200) {
										var value = (watcher.frame_num - watcher.tmp_num)
											* 1000 / (now - watcher.tmp_time);
										watcher.fps = watcher.fps * 0.9 + value
											* 0.1;
										watcher.tmp_num = watcher.frame_num;
										watcher.tmp_time = now;
									}
								}
								watcher.frame_queue = watcher.frame_queue
									.slice(j + 1);
								break;
							}
						}
					}
				}
				return false;
			}

			// image from upstream
			rtp
				.set_callback(9004, function(pack) {
					if (pack.GetPayloadType() == PT_STATUS) {
						var data_len = pack.GetPacketLength();
						var header_len = pack.GetHeaderLength();
						var data = pack.GetPacketData();
						var start = 0;
						var start_code = '<'.charCodeAt(0);
						var end_code = '>'.charCodeAt(0);
						for (var j = header_len; j < data_len; j++) {
							if (data[j] == start_code) {
								start = j;
							} else if (data[j] == end_code) {
								var str = String.fromCharCode.apply("", data
									.subarray(start, j + 1), 0);
								var split = str.split(' ');
								var name;
								var value;
								for (var i = 0; i < split.length; i++) {
									var separator = (/[=,\"]/);
									var _split = split[i].split(separator);
									if (_split[0] == "name") {
										name = _split[2];
									} else if (_split[0] == "value") {
										value = _split[2];
									}
								}
								if (name && watches[name]) {
									watches[name](value);
								}
							}
						}
					} else if (pack.GetPayloadType() == PT_CAM_BASE) {
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
								var conn;
								var server_key;
								if (data[data_len - 2] == 0x4C
									&& data[data_len - 1] == 0x55) {
									if ((active_frame[0][header_len + 2 + 4] & 0x1f) == 6) {// sei
										var str = String.fromCharCode
											.apply("", active_frame[0]
												.subarray(header_len + 2 + 4), 0);
										var split = str.split(' ');
										for (var i = 0; i < split.length; i++) {
											var separator = (/[=,\"]/);
											var _split = split[i]
												.split(separator);
											if (_split[0] == "frame_id") {
												conn = rtp.get_conn(_split[2]);
											} else if (_split[0] == "server_key") {
												server_key = _split[2];
											}
										}
									}
								}
								// image to downstream
								var need_to_send = true;
								if (server_key) {
									need_to_send = rtp
										.push_frame_queue(server_key, conn);
								}
								if (need_to_send) {
									rtp._sendpacket(active_frame, conn);
								}
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
			rtcp.set_callback(function(pack, conn) {
				if (pack.GetPayloadType() == PT_CMD) {
					var cmd = pack.GetPacketData().toString('ascii', pack
						.GetHeaderLength());
					var split = cmd.split('\"');
					var id = split[1];
					var value = split[3];
					plugin_host.send_command(value, conn);
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
					var pack = rtcp
						.build_packet(new Buffer(cmd, 'ascii'), PT_CMD);
					rtcp.sendpacket(pack, 9005, "127.0.0.1");

					rtcp_command_id++;
				}
			}, 20);
			// status to downstream
			setInterval(function() {
				var pack_list = [];
				for ( var name in statuses) {
					if (statuses[name]) {
						var ret = statuses[name]();
						if (!ret.succeeded) {
							continue;
						}
						var value = ret.value;
						var status = "<picam360:status name=\"" + name
							+ "\" value=\"" + value + "\" />";
						var pack = rtp
							.build_packet(new Buffer(status, 'ascii'), 100);
						pack_list.push(pack);
					}
				}
				if (pack_list.length > 0) {
					rtp._sendpacket(pack_list);
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
			io.sockets.on("connection", function(socket) {
				rtp.add_watcher(socket);
				socket.on("connected", function() {
				});
				socket.on("disconnect", function() {
					rtp.remove_watcher(socket);
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
				global.window.evt_listener = [];
				global.window.postMessage = function(message, origin) {
					console.log(message);
					var event = {
						source : global.window,
						data : message,
					};
					if (global.window.evt_listener["message"]) {
						global.window.evt_listener["message"].forEach(function(
							callback) {
							callback(event);
						});
					}
				};
				global.window.addEventListener = function(name, callback, bln) {
					if (!global.window.evt_listener[name]) {
						global.window.evt_listener[name] = [];
					}
					global.window.evt_listener[name].push(callback);
				}
				var uuid = options["wrtc_uuid"] || uuidv1();
				console.log("\n\n\n");
				console.log("webrtc uuid : " + uuid);
				console.log("\n\n\n");
				var Peer = require("peerjs");
				var peer = new Peer(uuid, {
					host : SIGNALING_HOST,
					port : SIGNALING_PORT,
					secure : SIGNALING_SECURE,
					key : P2P_API_KEY,
					debug : options.debug || 0
				});

				peer.on('connection', function(conn) {
					rtp.add_watcher(conn);
					conn.on('close', function() {
						rtp.remove_watcher(conn);
					});
				});
				peer.on('error', function(err) {
					if (err.type == "network") {// disconnect
						setTimeout(function() {
							peer.reconnect();
						}, 1000);
					}
				});
			}
			callback(null);
		},
		function(callback) {
			// plugin host
			// cmd handling
			function command_handler(value, conn) {
				var split = value.split(' ');
				if (split[0] == "get_file") {
					filerequest_list.push({
						filename : split[1],
						key : split[2],
						conn : conn
					});
				} else if (split[0] == "set_view_quaternion") {
					var id = rtp.get_frame_id(conn);
					if (id) {
						var cmd = CAPTURE_DOMAIN + value + " id=" + id;
						plugin_host.send_command(cmd, conn);

						var server_key;
						var split = value.split(' ');
						for (var i = 0; i < split.length; i++) {
							var separator = (/[=,\"]/);
							var _split = split[i].split(separator);
							if (_split[0] == "server_key") {
								server_key = _split[1];
								rtp.pop_frame_queue(server_key, conn);
							}
						}
					}
				} else if (split[0] == "snap") {
					var filename = moment().format('YYYYMMDD_hhmmss') + '.jpeg';
					var filepath = 'userdata/' + filename;
					var cmd = CAPTURE_DOMAIN + 'snap -0 -o /tmp/' + filename;
					plugin_host.send_command(cmd, conn);
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
											rtp
												._sendpacket(build_packet(data, PT_FILE), conn);
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
					plugin_host.send_command(cmd, conn);
					console.log(cmd);
					is_recording = true;
					frame_duration = duration;
					last_frame_date = null;
				} else if (split[0] == "stop_record") {
					is_recording = false;
					var cmd = CAPTURE_DOMAIN + 'stop_record -0';
					plugin_host.send_command(cmd, conn);
					console.log(cmd);
					var filename = moment().format('YYYYMMDD_hhmmss') + '.mp4';
					var filepath = 'userdata/' + filename;
					var ffmpeg_cmd = 'ffmpeg -y -r 10 -i /tmp/movie.h264 -c:v copy '
						+ filepath;
					var delh264_cmd = 'rm /tmp/movie.h264';
					var cmd = ffmpeg_cmd + ' ; ' + delh264_cmd;
					console.log(cmd);
					child_process
						.exec(cmd, function() {
							fs
								.readFile(filepath, function(err, data) {
									if (err) {
										console.log("not found :" + filepath);
									} else {
										rtp
											._sendpacket(build_packet(data, PT_FILE), conn);
										console.log("send :" + filepath);
									}
								});
						});
				} else if (split[0] == "request_call") {
					request_call = split[1];
				}
			}
			function filerequest_handler(filename, key, conn) {
				fs.readFile("www/" + filename, function(err, data) {
					var header_str;
					if (err) {
						var header_str = "<picam360:file name=\"" + filename
							+ "\" key=\"" + key + "\" status=\"404\" />";
						data = new Buffer(0);
						console.log("unknown :" + filename + ":" + key);
					} else {
						var header_str = "<picam360:file name=\"" + filename
							+ "\" key=\"" + key
							+ "\" status=\"200\" seq=\"0\" eof=\"true\" />";
					}
					var header = new Buffer(header_str, 'ascii');
					var len = 2 + header.length + data.length;
					var buffer = new Buffer(len);
					buffer.writeUInt16BE(header.length, 0);
					header.copy(buffer, 2);
					data.copy(buffer, 2 + header.length);
					var pack = rtp.build_packet(buffer, PT_FILE);
					rtp.sendpacket(conn, pack);
				});
			}
			setInterval(function() {
				if (cmd_list.length) {
					var cmd = cmd_list.shift();
					command_handler(cmd.value, cmd.conn);
				}
				if (filerequest_list.length) {
					var filerequest = filerequest_list.shift();
					filerequest_handler(filerequest.filename, filerequest.key, filerequest.conn);
				}
			}, 20);
			plugin_host.send_command = function(value, conn) {
				if (value.startsWith(UPSTREAM_DOMAIN)) {
					cmd2upstream_list
						.push(value.substr(UPSTREAM_DOMAIN.length));
				} else {
					cmd_list.push({
						value : value,
						conn : conn
					});
				}
			};
			plugin_host.add_watch = function(name, callback) {
				watches[name] = callback;
			};
			plugin_host.add_status = function(name, callback) {
				statuses[name] = callback;
			};

			plugin_host.add_watch(UPSTREAM_DOMAIN + "next_frame_id", function(
				value) {
				upstream_next_frame_id = value;
			});

			plugin_host.add_status("is_recording", function() {
				return {
					succeeded : true,
					value : is_recording
				};
			});

			plugin_host.add_status("request_call", function() {
				return {
					succeeded : request_call != "",
					value : request_call
				};
			});

			plugin_host.add_status("p2p_num_of_members", function() {
				var value = 0;
				for (var i = 0; i < rtp_rx_watcher.length; i++) {
					var watcher = rtp_rx_watcher[i];
					if (watcher.conn.peerConnection) { // webrtc
						value++;
					}
				}
				return {
					succeeded : true,
					value : value
				};
			});

			// delete all frame
			plugin_host.send_command(UPSTREAM_DOMAIN + "delete_frame -i *");

			callback(null);
		}], function(err, result) {
	});
