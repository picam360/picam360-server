var dgram = require("dgram");

var sequencenumber = 0;
var timestamp = 0;
var csrc = 0;

function PacketHeader(pack) {
	var packetlength = pack.length;
	var payloadtype = pack.readUInt8(1) & 0x7F;
	var sequencenumber = pack.readUInt16BE(2);
	var timestamp = pack.readUInt32BE(4);
	var ssrc = pack.readUInt32BE(8);
	var self = {
		GetSequenceNumber : function() {
			return sequencenumber;
		},
		GetTimestamp : function() {
			return timestamp;
		},
		GetSsrc : function() {
			return ssrc;
		},
		GetPacketData : function() {
			return pack;
		},
		GetPacketLength : function() {
			return packetlength;
		},
		GetHeaderLength : function() {
			return 12;
		},
		GetPayloadType : function() {
			return payloadtype;
		},
		GetPayloadLength : function() {
			return packetlength - self.GetHeaderLength();
		},
		GetPayload : function() {
			return new Uint8Array(pack, self.GetHeaderLength(), self
				.GetPayloadLength());
		}
	};
	return self;
}

function build_packet(data, pt) {
	var header_len = 12;
	var pack = new Buffer(header_len + data.length);
	pack.writeUInt8(0, 0);
	pack.writeUInt8(pt & 0x7F, 1);
	pack.writeUInt16BE(sequencenumber, 2);
	pack.writeUInt32BE(timestamp, 4);
	pack.writeUInt32BE(csrc, 8);
	data.copy(pack, header_len);

	sequencenumber++;
	if (sequencenumber >= (1 << 16)) {
		sequencenumber = 0;
	}

	return pack;
}

function set_callback(port, callback) {

	var rtp_rx = dgram.createSocket("udp4");

	rtp_rx.on("error", function(err) {
		console.log("server error:\n" + err.stack);
		server.close();
	});

	var marker = 0;
	var pack = null;
	var xmp = false;
	var xmp_len = 0;
	var xmp_pos = 0;
	rtp_rx
		.on("message", function(buff, rinfo) {
			var data_len = buff.length;
			for (var i = 0; i < data_len; i++) {
				if (xmp) {
					if (xmp_pos == 2) {
						xmp_len = buff[i] << 8;//network(big) endian
						xmp_pos++;
					} else if (xmp_pos == 3) {
						xmp_len += buff[i] << 0;//network(big) endian
						xmp_pos++;
					} else if (xmp_pos == 4) {
						if (buff[i] == 0x72) {// r
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
						if (buff[i] == 0x70) {// p
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
							buff.copy(pack, xmp_pos - 8, i, i
								+ (xmp_len - xmp_pos));
							i += xmp_len - xmp_pos - 1;
							xmp_pos = xmp_len;

							callback(PacketHeader(pack));

							pack = null;
							xmp = false;
						} else {
							var rest_in_buff = data_len - i;
							buff.copy(pack, xmp_pos - 8, i, i + rest_in_buff);
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

	rtp_rx
		.on("listening", function() {
			var address = rtp_rx.address();
			console.log("server listening " + address.address + ":"
				+ address.port);
		});

	rtp_rx.bind(port);
}

// @_packet : Buffer
function sendpacket(stream, packets) {
	if (!Array.isArray(packets)) {
		stream.send(packets);
		return;
	}
	for (var i = 0; i < packets.length; i++) {
		stream.send(packets[i]);
	}
}

exports.PacketHeader = PacketHeader;
exports.set_callback = set_callback;
exports.build_packet = build_packet;
exports.sendpacket = sendpacket;
