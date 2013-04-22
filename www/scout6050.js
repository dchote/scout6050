	var queue = new Mosquitto();
	
	var url = 'ws://' + window.location.host + '/mqtt';
	
	var topic = 'dchote/scout6050';
	var controlTopic = 'dchote/scout6050-control';
	
	var position = null;
	var packets = 0;
	
	var nodes = new Object();
	var analyser;
	var isPlaying;
	
	var frequency = 500;
	var volume = 0.5
	var pan = 0.5;
	
	$(document).ready(function() {
		try {
			nodes.context = new (window.AudioContext || window.webkitAudioContext);
			
			analyser = new WavyJones(nodes.context, 'oscilloscope');
			analyser.connect(nodes.context.destination);
			analyser.lineColor = '#3A3A49';
			analyser.lineThickness = 1;
			
			nodes.oscillator = nodes.context.createOscillator();
			nodes.panner = nodes.context.createPanner();
			nodes.volume = nodes.context.createGainNode();

			updateAudioState();
			
			nodes.oscillator.connect(nodes.panner);
			nodes.panner.connect(nodes.volume);
			nodes.volume.connect(analyser);
			
		} catch (e) {
			alert('No web audio oscillator support in this browser');
		}
		
		queue.connect(url, 10);
		
		queue.onconnect = function(rc) {
			if (rc == 0) {
				queue.subscribe(topic, 0);
				
				$('#connectionDescription').html('Connected with topic "' + topic + '"');
			}
        };
		
		queue.onmessage = function(topic, payload, qos) {
			if (payload.length < 3) {
				return;
			}
			
			try {
				var responsePacket = jQuery.parseJSON(payload);

				if (responsePacket.y && responsePacket.p) {
					updatePosition(responsePacket);
				} else if (responsePacket.payload == 'pong') {
					var currentDate = new Date;
					var currentTime = currentDate.getTime();
					var pingTime = new Date(responsePacket.timestamp);

					$('#latency').html = (currentTime - pingTime);
				} else if (responsePacket.payload == 'health') {
					$('#batteryVoltage').html(responsePacket.batteryVoltage);
					$('#temperature').html(responsePacket.temperature);
				}
			} catch (e) {
				console.log(e);
			}
			
		};
			
	});
	
	
	function updatePosition(newPosition) {
		position = newPosition;
		
		packets++;
		
		audioStateFromPosition(position);
		updateAudioState();
		
		$('#yawValue').html(position.y);
		$('#pitchValue').html(position.p);
		$('#rollValue').html(position.r);
		
		$('#totalPackets').html(packets);
	}
	
	function audioStateFromPosition(position) {
		frequency = ((position.y + 180) * 2).toFixed(0);
		volume = (((position.p + 180) / 100) * 0.4).toFixed(2);
		pan = (((position.r + 180) / 100) * 0.4).toFixed(2);
	}
	
	function updateAudioState() {
		try {
			nodes.oscillator.frequency.value = frequency;
			nodes.panner.setPosition(pan, 0, 0);
			nodes.volume.gain.value = volume;
			
			$('#frequencyValue').html(frequency + 'hz');
			$('#panValue').html(pan);
			$('#volumeValue').html(volume);
		} catch (e) {}
	}
	
	function drawAnalyser() {
		
	}
	
	function stop() {
		isPlaying = false;		
		nodes.oscillator.noteOff(0);
	}

	function play() {
		isPlaying = true;
		nodes.oscillator.noteOn(0);
	}