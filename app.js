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
var xmlhttprequest = require('xmlhttprequest');

var UPSTREAM_DOMAIN = "upstream.";
var SERVER_DOMAIN = "";
var CAPTURE_DOMAIN = UPSTREAM_DOMAIN;
var DRIVER_DOMAIN = UPSTREAM_DOMAIN + UPSTREAM_DOMAIN;
var PT_STATUS = 100;
var PT_CMD = 101;
var PT_FILE = 102;
var PT_CAM_BASE = 110;
var PT_AUDIO_BASE = 120;

var SIGNALING_HOST = "peer.picam360.com";
// var SIGNALING_HOST = "test-peer-server.herokuapp.com";
var SIGNALING_PORT = 443;
var SIGNALING_SECURE = true;

var data_host = "";
var PING_TO_DATA_HOST_INTERVAL = 10000;

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
var plugins = [];
var rtp_rx_watcher = [];
var cmd2upstream_list = [];
var cmd_list = [];
var watches = [];
var statuses = [];
var filerequest_list = [];

var upstream_next_frame_id = 0;
var upstream_info = "";
var upstream_menu = "";
var upstream_quaternion = [0, 0, 0, 1.0];
var upstream_north = 0;

var is_recording = false;
var memoryusage_start = 0;
var GC_THRESH = 16 * 1024 * 1024;// 16MB
var capture_if;
var capture_process;
var m_request_call = "";

var http = null;

var options = [];

async
	.waterfall([
		function(callback) {// argv
			var conf_filepath = 'config.json';
			for (var i = 0; i < process.argv.length; i++) {
				if (process.argv[i] == "-c") {
					conf_filepath = process.argv[i + 1];
					i++;
				}
			}
			var tmp_conf_filepath = "/tmp/picam360-server.conf.json";
			var cmd = "grep -v -e '^\s*#' " + conf_filepath + " > " + tmp_conf_filepath;
			child_process.exec(cmd, function() {
				console.log("load config file : " + conf_filepath);
				options = JSON.parse(fs.readFileSync(tmp_conf_filepath, 'utf8'));
				callback(null);
			});
		},
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

			rtp.send_error = function(conn, err) {
				setTimeout(function() {
					var name = "error";
					var value = err;
					var status = "<picam360:status name=\"" + name
						+ "\" value=\"" + value + "\" />";
					var pack = rtp
						.build_packet(new Buffer(status, 'ascii'), PT_STATUS);
					rtp.sendpacket(conn, pack);
				}, 1000);
			}

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
					rtp.send_error(conn, "exceeded_num_of_clients");
					return;
				} else {
					console.log("connection opend : " + ip);
				}

				var watcher = {
					ip : ip,
					conn : conn,
					frame_queue : [],
					fps : 5,
					latency : 1.0,
					min_latency : 1.0,
					frame_num : 0,
					tmp_num : 0,
					tmp_latency : 0,
					tmp_time : 0,
					frame_id : upstream_next_frame_id,
					timeout : false,
					is_init : false
				};
				var init_con = function() {
					watcher.is_init = true;
					var create_frame_cmd = sprintf("create_frame -m %s -w %d -h %d -s %s -f %d", options.frame_mode
						|| "WINDOW", options.frame_width || 512, options.frame_height || 512, options.frame_encode
						|| "h264", options.frame_fps || 5);
					if (options.frame_bitrate) {
						create_frame_cmd += " -k " + options.frame_bitrate;
					}
					console.log(create_frame_cmd);
					plugin_host
						.send_command(UPSTREAM_DOMAIN + create_frame_cmd, conn);

					rtcp.add_connection(conn);
					rtp_rx_watcher.push(watcher);

					watcher.timer = setInterval(function() {
						if (watcher.timeout) {
							console.log("timeout");
							rtp.remove_watcher(conn);
							if (conn.peerConnection) { // webrtc
								conn.close();
							} else {
								conn.disconnect(true);
							}
						} else {
							watcher.timeout = true;
						}
					}, 60000);
				};
				conn
					.on('data', function(data) {
						watcher.timeout = false;
						if (!watcher.is_init) {
							var pack = rtcp.PacketHeader(data);
							if (pack.GetPayloadType() == PT_CMD) {
								var cmd = pack.GetPacketData()
									.toString('ascii', pack.GetHeaderLength());
								var split = cmd.split('\"');
								var id = split[1];
								var value = split[3].split(' ');
								if (value[0] == "ping") {
									var status = "<picam360:status name=\"pong\" value=\""
										+ value[1]
										+ " "
										+ new Date().getTime()
										+ "\" />";
									var pack = rtp
										.build_packet(new Buffer(status, 'ascii'), PT_STATUS);
									rtp.sendpacket(conn, pack);
									return;
								}
							}
							init_con();
						}
					});
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
				if (rtp.p2p_num_of_members() <= 1) { // reset request_call
					m_request_call = "";
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
						watcher.frame_queue.push({
							server_key : server_key,
							base_time : now
						});
					}
				}
			}

			rtp.pop_frame_queue = function(server_key, conn) {
				var now = new Date().getTime();
				for (var i = rtp_rx_watcher.length - 1; i >= 0; i--) {
					var watcher = rtp_rx_watcher[i];
					if (watcher.conn === conn) {
						for (var j = 0; j < watcher.frame_queue.length; j++) {
							if (watcher.frame_queue[j].server_key == server_key) {
								watcher.frame_num++;
								{// latency
									var value = (now - watcher.frame_queue[j].base_time) / 1000.0;
									watcher.tmp_latency += watcher.latency;
								}
								{// fps
									if (watcher.tmp_time == 0) {
										watcher.tmp_time = now;
									} else if (watcher.frame_num
										- watcher.tmp_num > 5) {
										var frame_num = watcher.frame_num
											- watcher.tmp_num;
										var fps = frame_num * 1000
											/ (now - watcher.tmp_time);
										watcher.fps = watcher.fps * 0.9 + fps
											* 0.1;
										var latency = watcher.tmp_latency
											/ frame_num;
										watcher.latency = watcher.latency * 0.9
											+ value * 0.1;
										if (watcher.latency < watcher.min_latency) {
											watcher.min_latency = watcher.latency;
										}
										watcher.tmp_num = watcher.frame_num;
										watcher.tmp_latency = 0;
										watcher.tmp_time = now;

										var target_latency = (watcher.latency + watcher.min_latency) / 2;
										var stack_fps = (watcher.frame_queue.length - (j + 1))
											/ target_latency;
										var target_fps;
										var offset = (options.latency_offset_msec || 200) / 1000;
										var step = options.fps_step || 1;
										if (watcher.latency > (watcher.min_latency + offset)
											* (1 + offset)) {
											target_fps = watcher.fps - step;
										} else if (stack_fps > watcher.fps
											* (1 + offset)) {
											target_fps = watcher.fps - step;
										} else {
											target_fps = watcher.fps + step;
										}
										target_fps = Math
											.min(Math.max(target_fps, 1), options.max_fps || 15);
										var cmd = UPSTREAM_DOMAIN
											+ "set_fps -i " + watcher.frame_id
											+ " -f " + target_fps;
										// console.log("fps:" + watcher.fps
										// + ",latency:" + watcher.latency
										// + ",min_latency:" +
										// watcher.min_latency
										// + ",stack_fps:" + stack_fps
										// + ",target_fps:" + target_fps);
										plugin_host
											.send_command(cmd, watcher.conn);
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

			rtp.p2p_num_of_members = function() {
				var value = 0;
				for (var i = 0; i < rtp_rx_watcher.length; i++) {
					var watcher = rtp_rx_watcher[i];
					if (watcher.conn.peerConnection) { // webrtc
						value++;
					}
				}
				return value;
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
								var name;
								var value;
								var last_spece = -1;
								var first_dq = -1;
								for (var i = 0; i < str.length; i++) {
									if (first_dq < 0 && str[i] == ' ') {
										last_spece = i;
									} else if (str[i] == '\"') {
										if (first_dq < 0) {
											first_dq = i;
										} else {
											var tag = str
												.slice(last_spece + 1, first_dq - 1);
											if (tag == "name") {
												name = UPSTREAM_DOMAIN
													+ str
														.slice(first_dq + 1, i);
											} else if (tag == "value") {
												value = str
													.slice(first_dq + 1, i);
											}
											last_spece = -1;
											first_dq = -1;
										}
									}
								}
								if (name && watches[name]) {
									watches[name](value);
								}
							}
						}
					} else if (pack.GetPayloadType() == PT_CAM_BASE) {
						if (options.latency_debug) {
							var latency = new Date().getTime() / 1000
								- (pack.GetTimestamp() + pack.GetSsrc() / 1E6);
							console.log("seq:" + pack.GetSequenceNumber()
								+ ":latency:" + latency);
						}
						var data_len = pack.GetPacketLength();
						var header_len = pack.GetHeaderLength();
						var data = pack.GetPacketData();
						// rtp._sendpacket([data]);
						// return;
						if (!active_frame) {
							if ((data[header_len] == 0xFF && data[header_len + 1] == 0xD8)
								|| (data[header_len] == 0x4E && data[header_len + 1] == 0x41)
								|| (data[header_len] == 0x48 && data[header_len + 1] == 0x45)) { // SOI
								active_frame = [];
								// console.log("new_frame");
							}
						}
						if (active_frame) {
							active_frame.push(data);
							var codec = null;
							if (data[data_len - 2] == 0xFF
								&& data[data_len - 1] == 0xD9) {
								codec = "MJPEG";
							} else if (data[data_len - 2] == 0x4C
								&& data[data_len - 1] == 0x55) {
								codec = "H264";
							} else if (data[data_len - 2] == 0x56
								&& data[data_len - 1] == 0x43) {
								codec = "H265";
							}
							if (codec) { // EOI
								var conn;
								var server_key;
								if (codec == "H264" || codec == "H265") {
									var sei = false;
									var nal_type = -1;
									if (codec == "H264") {
										nal_type = active_frame[0][header_len + 2 + 4] & 0x1f;
										if (nal_type == 6) {// sei
											sei = true;
										}
									} else if (codec == "H265") {
										nal_type = (active_frame[0][header_len + 2 + 4] & 0x7e) >> 1;
										if (nal_type == 40) {// sei
											sei = true;
										}
									}
									if (sei) {// sei
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

										if (options.debug == "stream") {
											var nal_len = 0;
											var _nal_len = 0;
											for (var i = 1; i < active_frame.length - 1; i++) {
												if (i == 1) {
													nal_len = active_frame[i][header_len + 0] << 24
														| active_frame[i][header_len + 1] << 16
														| active_frame[i][header_len + 2] << 8
														| active_frame[i][header_len + 3];
													_nal_len += active_frame[i].length - 4 - 12;

													// nal header 1byte
													if (codec == "H264") {
														nal_type = active_frame[i][header_len + 4] & 0x1f;
													} else if (codec == "H265") {
														nal_type = (active_frame[i][header_len + 4] & 0x7e) >> 1;
													}
												} else {
													_nal_len += active_frame[i].length - 12;
												}
											}
											console.log("nal_len:" + nal_len
												+ "," + _nal_len + ":nal_type:"
												+ nal_type);
										}
									}
								}
								// image to downstream
								if (server_key) {
									need_to_send = rtp
										.push_frame_queue(server_key, conn);
								}
								if (conn) {
									rtp._sendpacket(active_frame, conn);
								} else {
									console.log("warning : no conn");
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
					} else if (pack.GetPayloadType() == PT_AUDIO_BASE) {
						var data = pack.GetPacketData();
						rtp._sendpacket(data, null);
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
						var value = encodeURIComponent(ret.value);
						var status = "<picam360:status name=\"" + name
							+ "\" value=\"" + value + "\" />";
						var pack = rtp
							.build_packet(new Buffer(status, 'ascii'), PT_STATUS);
						pack_list.push(pack);
					}
				}
				if (pack_list.length > 0) {
					rtp._sendpacket(pack_list);
				}
			}, 200);
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
				var connect = function() {
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

					peer
						.on('open', function(id) {// connected to server
							if (peer.ping_started) {
								return;
							}
							peer.ping_started = true;

							var http = new xmlhttprequest.XMLHttpRequest();
							var protocol = SIGNALING_SECURE
								? 'https://'
								: 'http://';
							var url = protocol + SIGNALING_HOST + ':'
								+ SIGNALING_PORT + '/' + P2P_API_KEY
								+ '/data_host';

							console.log("IN getDataHost");

							http.open('get', url, true);
							http.onerror = function(e) {
								util.error('Error getDataHost', e);
							};
							http.onreadystatechange = function() {
								if (http.readyState !== 4) {
									return;
								}
								if (http.status !== 200) {
									http.onerror();
									return;
								}

								console
									.log("SUCCESS getDataHost %s", http.responseText);
								data_host = http.responseText;

								var _pingToDataHost = function() {
									if (data_host) {
										var http = new xmlhttprequest.XMLHttpRequest();
										var protocol = SIGNALING_SECURE
											? 'https://'
											: 'http://';
										var url = protocol + data_host;

										http.open('POST', url, true);
										http
											.setRequestHeader('Content-Type', 'application/json');
										http
											.setRequestHeader('Authorization', 'Token token=mzi9vncbbo');

										http.onerror = function(e) {
											util.error('Error _ping', e);
										};
										http.onreadystatechange = function() {
											if (http.readyState !== 4) {
												return;
											}
											if (http.status !== 200) {
												http.onerror();
												return;
											}

											console
												.log("RESPONSE _ping %s", http.responseText);
										};

										var data = JSON.stringify({
											method : "ping",
											app_key : uuid
										});
										console.log(data);
										http.send(data);
									}
								}

								_pingToDataHost();// do ping once first
								peer.ping_timer = setInterval(function() {
									_pingToDataHost();
								}, PING_TO_DATA_HOST_INTERVAL);
							};
							http.send(null);
						});

					peer.on('error', function(err) {
						if (err.type == "network") {// disconnect
							setTimeout(function() {
								peer.reconnect();
							}, 1000);
						} else if (err.type == "socket-error") {// internal
							// socket
							// error
							setTimeout(function() {
								connect();
							}, 1000);
						}
					});
				}
				connect();
			}
			callback(null);
		},
		function(callback) {
			// plugin host
			var m_view_quaternion = [0, 0, 0, 1.0];
			// cmd handling
			function command_handler(value, conn) {
				var split = value.split(' ');
				var domain = split[0].split('.');
				if (domain.length != 1 && domain[0] != "picam360_server") {
					// delegate to plugin
					for (var i = 0; i < plugins.length; i++) {
						if (plugins[i].name && plugins[i].name == domain[0]) {
							if (plugins[i].command_handler) {
								plugins[i].command_handler(value);
								break;
							}
						}
					}
					return;
				}
				if (split[0] == "ping") {
				} else if (split[0] == "get_file") {
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
							} else if (_split[0] == "quat") {
								m_view_quaternion = [parseFloat(_split[1]),
									parseFloat(_split[2]),
									parseFloat(_split[3]),
									parseFloat(_split[4])];
							}
						}
					}
				} else if (split[0] == "snap") {
					var id = rtp.get_frame_id(conn);
					if (id) {
						var key = split[1];
						var filename = moment().format('YYYYMMDD_hhmmss')
							+ '.jpeg';
						var filepath = 'userdata/' + filename;
						var cmd = CAPTURE_DOMAIN + 'set_mode -i ' + id
							+ ' -m WINDOW';
						plugin_host.send_command(cmd, conn);
						cmd = CAPTURE_DOMAIN + 'snap -i ' + id + ' -o /tmp/'
							+ filename;
						plugin_host.send_command(cmd, conn);
						console.log(cmd);
						watchFile('/tmp/' + filename, function() {
							var cmd = CAPTURE_DOMAIN + 'set_mode -i ' + id
								+ ' -m ' + (options.frame_mode || "WINDOW");
							plugin_host.send_command(cmd, conn);

							console.log(filename + ' saved.');
							var rm_cmd = 'mv' + ' /tmp/' + filename + ' '
								+ filepath;
							var cmd = rm_cmd;
							console.log(cmd);
							child_process.exec(cmd, function() {
								// callback(filename);
								fs.readFile(filepath, function(err, data) {
									if (err) {
										console.log("not found :" + filepath);
									} else {
										send_file(filename, key, conn, data);
										console.log("send :" + filepath);
									}
								});
							});
						});
					}
				} else if (split[0] == "start_record") {
					if (is_recording)
						return;
					var id = rtp.get_frame_id(conn);
					if (id) {
						var cmd = CAPTURE_DOMAIN + 'set_mode -i ' + id
							+ ' -m WINDOW';
						plugin_host.send_command(cmd, conn);

						cmd = CAPTURE_DOMAIN + 'start_record -i ' + id
							+ ' -o /tmp/movie.h264';
						plugin_host.send_command(cmd, conn);
						console.log(cmd);
						is_recording = true;
					}
				} else if (split[0] == "stop_record") {
					var id = rtp.get_frame_id(conn);
					if (id) {
						is_recording = false;

						var key = split[1];
						var cmd = CAPTURE_DOMAIN + 'stop_record -i ' + id;
						plugin_host.send_command(cmd, conn);
						console.log(cmd);
						var filename = moment().format('YYYYMMDD_hhmmss')
							+ '.mp4';
						var filepath = 'userdata/' + filename;
						var ffmpeg_cmd = 'ffmpeg -y -r 10 -i /tmp/movie.h264 -c:v copy '
							+ filepath;
						var delh264_cmd = 'rm /tmp/movie.h264';
						var cmd = ffmpeg_cmd + ' ; ' + delh264_cmd;
						console.log(cmd);
						child_process.exec(cmd, function() {
							var cmd = CAPTURE_DOMAIN + 'set_mode -i ' + id
								+ ' -m ' + (options.frame_mode || "WINDOW");
							plugin_host.send_command(cmd, conn);
							fs.readFile(filepath, function(err, data) {
								if (err) {
									console.log("not found :" + filepath);
								} else {
									send_file(filename, key, conn, data);
									console.log("send :" + filepath);
								}
							});
						});
					}
				} else if (split[0] == "request_call") {
					m_request_call = split[1];
				}
			}

			function send_file(filename, key, conn, data) {
				var chunksize = 63 * 1024;
				var length;
				for (var i = 0, seq = 0; i < data.length; i += length, seq++) {
					var eof;
					if (i + chunksize >= data.length) {
						eof = true;
						length = data.length - i;
					} else {
						eof = false;
						length = chunksize;
					}
					var header_str = sprintf("<picam360:file name=\"%s\" key=\"%s\" status=\"200\" seq=\"%d\" eof=\"%s\" />", filename, key, seq, eof
						.toString());
					var header = new Buffer(header_str, 'ascii');
					var len = 2 + header.length + length;
					var buffer = new Buffer(len);
					buffer.writeUInt16BE(header.length, 0);
					header.copy(buffer, 2);
					data.copy(buffer, 2 + header.length, i, i + length);
					var pack = rtp.build_packet(buffer, PT_FILE);
					rtp.sendpacket(conn, pack);
				}
			}
			function filerequest_handler(filename, key, conn) {
				fs.readFile("www/" + filename, function(err, data) {
					if (err) {
						var header_str = "<picam360:file name=\"" + filename
							+ "\" key=\"" + key + "\" status=\"404\" />";
						data = new Buffer(0);
						console.log("unknown :" + filename + ":" + key);
						var header = new Buffer(header_str, 'ascii');
						var len = 2 + header.length;
						var buffer = new Buffer(len);
						buffer.writeUInt16BE(header.length, 0);
						header.copy(buffer, 2);
						var pack = rtp.build_packet(buffer, PT_FILE);
						rtp.sendpacket(conn, pack);
					} else {
						send_file(filename, key, conn, data);
					}
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
			if (options.aws_iot_enabled) {
				var interval_s = options.aws_iot_interval_s || 60;
				var awsIot = require('aws-iot-device-sdk');
				var device = awsIot
					.device({
						keyPath : 'certs/aws_iot/'
							+ options.aws_iot_private_key,
						certPath : 'certs/aws_iot/'
							+ options.aws_iot_certificate,
						caPath : 'certs/aws_iot/VeriSign-Class 3-Public-Primary-Certification-Authority-G5.pem',
						host : options.aws_iot_host,
					});

				device.on('connect', function() {
					console.log('Connected to AWS IOT.');

					setInterval(function() {
						for (var i = 0; i < plugins.length; i++) {
							if (plugins[i].aws_iot_handler) {
								plugins[i].aws_iot_handler(device);
							}
						}
					}, interval_s * 1000);
				});
			}
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
			plugin_host.get_vehicle_quaternion = function() {
				return upstream_quaternion;
			};
			plugin_host.get_vehicle_north = function() {
				return upstream_north;
			};
			plugin_host.get_view_quaternion = function() {
				return m_view_quaternion;
			};
			plugin_host.add_watch = function(name, callback) {
				watches[name] = callback;
			};
			plugin_host.add_status = function(name, callback) {
				statuses[name] = callback;
			};

			plugin_host.add_watch(UPSTREAM_DOMAIN + "next_frame_id", function(
				value) {
				if (upstream_next_frame_id != value) {
					upstream_next_frame_id = value;
					console.log("next_frame_id updted : " + value);
				}
			});

			plugin_host.add_watch(UPSTREAM_DOMAIN + "quaternion", function(
				value) {
				var separator = (/[,]/);
				var split = value.split(separator);
				upstream_quaternion = [parseFloat(split[0]),
					parseFloat(split[1]), parseFloat(split[2]),
					parseFloat(split[3])];
			});

			plugin_host.add_watch(UPSTREAM_DOMAIN + "north", function(value) {
				upstream_north = value;
			});

			plugin_host.add_watch(UPSTREAM_DOMAIN + "info", function(value) {
				upstream_info = value;
			});

			plugin_host.add_watch(UPSTREAM_DOMAIN + "menu", function(value) {
				upstream_menu = value;
			});

			plugin_host.add_status("is_recording", function() {
				return {
					succeeded : true,
					value : is_recording
				};
			});

			plugin_host.add_status("request_call", function() {
				return {
					succeeded : m_request_call != "",
					value : m_request_call
				};
			});

			plugin_host.add_status("p2p_num_of_members", function() {
				return {
					succeeded : true,
					value : rtp.p2p_num_of_members()
				};
			});

			plugin_host.add_status("info", function() {
				return {
					succeeded : upstream_info != "",
					value : upstream_info
				};
			});

			plugin_host.add_status("menu", function() {
				return {
					succeeded : upstream_menu != "",
					value : upstream_menu
				};
			});

			// delete all frame
			plugin_host.send_command(UPSTREAM_DOMAIN + "delete_frame -i *");

			callback(null);
		},
		function(callback) {
			// load plugin
			if (options["plugin_paths"]) {
				for ( var k in options["plugin_paths"]) {
					var plugin_path = options["plugin_paths"][k];
					console.log("loading... " + plugin_path);
					var plugin = require("./" + plugin_path)
						.create_plugin(plugin_host);
					plugins.push(plugin);
				}
				for (var i = 0; i < plugins.length; i++) {
					if (plugins[i].init_options) {
						plugins[i].init_options(options);
					}
				}
			}
			callback(null);
		}], function(err, result) {
	});
