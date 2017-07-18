process.chdir(__dirname);

var os = require('os');
var disk = require('diskusage');
var child_process = require('child_process');
var async = require('async');
var fs = require("fs");
var express = require('express');
var moment = require("moment");
var sprintf = require('sprintf-js').sprintf;
var dgram = require("dgram");

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

var recording = false;
var framecount = 0;
var frame_duration = 0;
var last_frame_date = null;
var memoryusage_start = 0;
var GC_THRESH = 16 * 1024 * 1024;// 16MB

var capture_if;
var capture_process;
async
		.waterfall(
				[
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
						function(callback) {// connect to picam-capture
							console.log("connect to picam-capture");

							var rtp_rx = dgram.createSocket("udp4");

							rtp_rx.on("error", function(err) {
								console.log("server error:\n" + err.stack);
								server.close();
							});

							var marker = 0;
							var packet = null;
							var xmp = false;
							var xmp_len = 0;
							var xmp_pos = 0;
							rtp_rx.on("message", function(buff, rinfo) {
								var data_len = buff.length;
								for (var i = 0; i < data_len; i++) {
									if (xmp) {
										if (xmp_pos == 2) {
											xmp_len = buff[i];
											xmp_pos++;
										} else if (xmp_pos == 3) {
											xmp_len += buff[i] << 8;
											xmp_pos++;
										} else if (xmp_pos == 4) {
											if (buff[i] == 0x72 ) {// r
												xmp_pos++;
											} else {
												xmp = false;
											}
										} else if (xmp_pos == 5) {
											if (buff[i] == 0x74) { // t
												xmp_pos++;
											} else {
												xmp = false;
											}
										} else if (xmp_pos == 6) {
											if (buff[i] == 0x70 ) {// p
												xmp_pos++;
											} else {
												xmp = false;
											}
										} else if (xmp_pos == 7) { // rtp_header
											if (buff[i] == 0x00) { // \0
												xmp_pos++;
											} else {
												xmp = false;
											}
										} else {
											if (xmp_pos == 8) {
												pack = new Buffer(xmp_len - 8);
											}
											if (i + (xmp_len - xmp_pos) <= data_len) {
												buff.copy(pack, xmp_pos - 8, i,
														i + (xmp_len - xmp_pos));
												i += xmp_len - xmp_pos - 1;
												xmp_pos = xmp_len;
												console.log("packet : " + xmp_len);
// pack->LoadHeader();
// if (lg_callback && lg_load_fd < 0) {
// lg_callback(pack->GetPayloadData(),
// pack->GetPayloadLength(),
// pack->GetPayloadType(),
// pack->GetSequenceNumber());
// }
// if (lg_record_fd > 0) {
// pthread_mutex_lock(&lg_record_packet_queue_mlock);
// lg_record_packet_queue.push_back(pack);
// mrevent_trigger(&lg_record_packet_ready);
// pthread_mutex_unlock(&lg_record_packet_queue_mlock);
// } else {
// delete pack;
// }
												pack = null;
												xmp = false;
											} else {
												var rest_in_buff = data_len - i;
												buff.copy(pack, xmp_pos - 8, i,
														i + rest_in_buff);
												i = data_len - 1;
												xmp_pos += rest_in_buff;
											}
										}
									} else {
										if (marker) {
											marker = 0;
											if (buff[i] == 0xE1) { // xmp
												xmp = true;
												xmp_pos = 2;
												xmp_len = 0;
											}
										} else if (buff[i] == 0xFF) {
											marker = 1;
										}
									}
								}
							});

							rtp_rx.on("listening",
									function() {
										var address = rtp_rx.address();
										console.log("server listening "
												+ address.address + ":"
												+ address.port);
									});

							rtp_rx.bind(9004);

							var rtcp_tx = dgram.createSocket("udp4");

							rtcp_tx.on("error", function(err) {
								console.log("server error:\n" + err.stack);
								rtcp_tx.close();
							});

							// rtcp_tx.bind(9005);

							callback(null);
						},
						function(callback) {// cam
							console.log("camera instance");

							var disk_free = 0;
							setInterval(function() {
								disk.check('/tmp', function(err, info) {
									disk_free = info.available;
								});
							}, 1000);
							setInterval(function() {
								if (global.gc && os.freemem() < GC_THRESH) {
									console.log("gc : free=" + os.freemem()
											+ " usage="
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
									.get(
											'/img/*.jpeg',
											function(req, res) {
												var url = req.url.split("?")[0];
												var query = req.url.split("?")[1];
												var filepath = 'userdata/'
														+ url.split("/")[2];
												console.log(url);
												console.log(query);
												console.log(filepath);
												fs
														.readFile(
																filepath,
																function(err,
																		data) {
																	if (err) {
																		res
																				.writeHead(404);
																		res
																				.end();
																		console
																				.log("404");
																	} else {
																		res
																				.writeHead(
																						200,
																						{
																							'Content-Type' : 'image/jpeg',
																							'Content-Length' : data.length,
																							'Cache-Control' : 'private, no-cache, no-store, must-revalidate',
																							'Expires' : '-1',
																							'Pragma' : 'no-cache',
																						});
																		res
																				.end(data);
																		console
																				.log("200");
																	}
																});
											});

							app
									.get(
											'/img/*.mp4',
											function(req, res) {
												var url = req.url.split("?")[0];
												var query = req.url.split("?")[1];
												var filepath = 'userdata/'
														+ url.split("/")[2];
												console.log(url);
												console.log(query);
												console.log(filepath);
												fs
														.readFile(
																filepath,
																function(err,
																		data) {
																	if (err) {
																		res
																				.writeHead(404);
																		res
																				.end();
																		console
																				.log("404");
																	} else {
																		var range = req.headers.range // bytes=0-1
																		if (!range) {
																			res
																					.writeHead(
																							200,
																							{
																								"Content-Type" : "video/mp4",
																								"X-UA-Compatible" : "IE=edge;chrome=1",
																								'Content-Length' : data.length
																							});
																			res
																					.end(data)
																		} else {
																			var total = data.length;
																			var split = range
																					.split(/[-=]/);
																			var ini = +split[1];
																			var end = split[2] ? +split[2]
																					: total - 1;
																			var chunkSize = end
																					- ini
																					+ 1;
																			res
																					.writeHead(
																							206,
																							{
																								"Content-Range" : "bytes "
																										+ ini
																										+ "-"
																										+ end
																										+ "/"
																										+ total,
																								"Accept-Ranges" : "bytes",
																								"Content-Length" : chunkSize,
																								"Content-Type" : "video/mp4",
																							})
																			res
																					.end(data
																							.slice(
																									ini,
																									chunkSize
																											+ ini))
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
									.on(
											"connection",
											function(socket) {

												socket.on("connected",
														function() {
														});

												socket
														.on(
																"snap",
																function(
																		callback) {
																	var filename = moment()
																			.format(
																					'YYYYMMDD_hhmmss')
																			+ '.jpeg';
																	var cmd = 'snap -E -W 3072 -H 1536 -o /tmp/'
																			+ filename
																			+ '\n';
																	capture_if
																			.write(cmd);
																	console
																			.log(cmd);
																	watchFile(
																			'/tmp/'
																					+ filename,
																			function() {
																				console
																						.log(filename
																								+ ' saved.');
																				var exiftool_cmd = 'exiftool -ProjectionType="equirectangular" -o'
																						+ ' userdata/'
																						+ filename
																						+ ' /tmp/'
																						+ filename;
																				var rm_cmd = 'rm'
																						+ ' /tmp/'
																						+ filename;
																				var cmd = exiftool_cmd
																						+ ' && '
																						+ rm_cmd;
																				console
																						.log(cmd);
																				child_process
																						.exec(
																								cmd,
																								function() {
																									callback(filename);
																								});
																			});
																});

												socket
														.on(
																"startRecord",
																function(
																		duration) {
																	if (recording)
																		return;
																	duration = (duration == null) ? 0
																			: duration;
																	var cmd = 'start_record -E -W 1024 -H 512 -o /tmp/movie.h264\n';
																	capture_if
																			.write(cmd);
																	console
																			.log(cmd);
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
														.on(
																"stopRecord",
																function(
																		callback) {
																	recording = false;
																	capture_if
																			.write('stop_record\n');
																	console
																			.log("camera recording stop");
																	var filename = moment()
																			.format(
																					'YYYYMMDD_hhmmss')
																			+ '.mp4';
																	var ffmpeg_cmd = 'ffmpeg -y -r 5 -i /tmp/movie.h264 -c:v copy userdata/'
																			+ filename;
																	var delh264_cmd = 'rm /tmp/movie.h264';
																	var spatialmedia_cmd = 'python submodules/spatial-media/spatialmedia -i userdata/'
																			+ filename
																			+ ' /tmp/vr.mp4';
																	// var cmd =
																	// ffmpeg_cmd
																	// + ' && '
																	// +
																	// delh264_cmd
																	// + ' && '
																	// +
																	// spatialmedia_cmd;
																	var cmd = ffmpeg_cmd
																			+ ' ; '
																			+ delh264_cmd;
																	console
																			.log(cmd);
																	child_process
																			.exec(
																					cmd,
																					function() {
																						callback(filename);
																					});
																});

												socket.on("disconnect",
														function() {
														});

											});

							http.listen(9001, function() {
								console.log('listening on *:9001');
							});

							callback(null);
						} ], function(err, result) {
				});