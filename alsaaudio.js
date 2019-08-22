'use strict';
const AlsaCapture = require("alsa-capture");

const { RTCAudioSource } = require('wrtc').nonstandard;

class RTCAudioSourceAlsa {
  constructor(options = {}) {
    options = {
      channelCount: 1,
      sampleRate: 48000,
      ...options
    };
  // default values for the options object

    var sources = {};
    const {
      device,
      channelCount,
      sampleRate
    } = options;
    
    const bitsPerSample = 16;
    const numberOfFrames = sampleRate / 100;
    
    const captureInstance = new AlsaCapture({
        channels: channelCount,
        debug: false,
        device: device?device:"default",
        format: "S16_LE",
        periodSize: numberOfFrames,
        periodTime: undefined,
        rate: sampleRate,
       });

	 // data is an ArrayBuffer
	 // Buffer size = numChannels * formatByteSize * periodSize
	 // Example: 2 Bytes (AlsaFormat.S16_LE) * 2 (numChannels) * 32
		// (periodSize) = 128 Bytes
	 captureInstance.on("audio", (_samples) => {
		const samples = new Int16Array(_samples.buffer, 0, _samples.byteLength / 2);
		if(true){//swap
	        for (let i = 0; i < numberOfFrames; i++) {
	        	var tmp = samples[i * channelCount + 0];
	        	samples[i * channelCount + 0] = samples[i * channelCount + 1];
	        	samples[i * channelCount + 1] = tmp;
	        }
		}
	    const data = {
	        samples,
	        sampleRate,
	        bitsPerSample,
	        channelCount,
	        numberOfFrames
	      };
	    for (var uuid in sources) {
	    	sources[uuid].onData(data);
	    }
	 });

    this.close = () => {
        captureInstance.close();
    };

    this.createTrack = (uuid) => {
      const source = new RTCAudioSource();
      sources[uuid] = source;
      return source.createTrack();
    };

    this.deleteTrack = (uuid) => {
      delete sources[uuid];
      return;
    };
  }
}

module.exports = RTCAudioSourceAlsa;