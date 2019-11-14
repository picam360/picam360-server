var dgram = require("dgram");
var rtcp_tx = dgram.createSocket("udp4");
var sequencenumber = 0;
var timestamp = 0;
var csrc = 0;
var callback;

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
		}
	};
	return self;
}

function build_packet(data, pt) {
	var raw_header_len = 8;
	var header_len = 12;
	var pack = new Buffer(raw_header_len + header_len + data.length);
	pack[0] = 0xFF;
	pack[1] = 0xE1;
	pack[2] = (pack.length >> 8) & 0xFF;//network(big) endian
	pack[3] = (pack.length >> 0) & 0xFF;//network(big) endian
	pack[4] = 0x72; // r
	pack[5] = 0x74; // t
	pack[6] = 0x70; // p
	pack[7] = 0x00; // \0
	pack.writeUInt8(0, raw_header_len + 0);
	pack.writeUInt8(pt & 0x7F, raw_header_len + 1);
	pack.writeUInt16BE(sequencenumber, raw_header_len + 2);
	pack.writeUInt32BE(timestamp, raw_header_len + 4);
	pack.writeUInt32BE(csrc, raw_header_len + 8);
	data.copy(pack, raw_header_len + header_len);

	sequencenumber++;
	if (sequencenumber >= (1 << 16)) {
		sequencenumber = 0;
	}

	return pack;
}

function set_callback(_callback) {
	callback = _callback;
}

function add_connection(ws) {
	if (!ws) {
		return;
	}
	ws.on("data", function(buff) {
		if (callback) {
			if (buff.constructor.name != "Buffer") {
				buff = new Buffer(buff);
			}
			callback(PacketHeader(buff), ws);
		}
	});
}

// @_packet : Buffer
function sendpacket(pack, port, ip) {
	rtcp_tx.send(pack, 0, pack.length, port, ip);
}

exports.PacketHeader = PacketHeader;
exports.set_callback = set_callback;
exports.add_connection = add_connection;
exports.build_packet = build_packet;
exports.sendpacket = sendpacket;
