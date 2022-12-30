from labdevices.functiongenerator import FunctionGenerator, FunctionGeneratorWaveform, FunctionGeneratorModulation
from labdevices.exceptions import CommunicationError_ProtocolViolation, CommunicationError_Timeout, CommunicationError_NotConnected

from time import sleep

import serial
import atexit
import struct

class FY6900Serial(FunctionGenerator):
	def __init__(
		self,

		serialPort = "/dev/ttyU0",
		serialCommandDelay = 0.1,
		debug = False,

		commandRetries = 3
	):
		super().__init__(
			nchannels = 2,
			freqrange = ( float(0.0), 100.0e6 ),
			amplituderange = ( 0.0, 24.0 ),
			offsetrange = ( -24.0, 24.0 ),

			arbitraryWaveforms = True,
			arbitraryWaveformLength = (8192, 8192),
			arbitraryWaveformMinMax = (0, 16383),
			arbitraryNormalizeInDriver = True,
			hasFrequencyCounter = True,

			supportedWaveforms = [
				FunctionGeneratorWaveform.SINE,
				FunctionGeneratorWaveform.SQUARE,
				FunctionGeneratorWaveform.RECTANGLE,
				FunctionGeneratorWaveform.TRAPEZOID,
				FunctionGeneratorWaveform.CMOS,
				FunctionGeneratorWaveform.ADJPULSE,
				FunctionGeneratorWaveform.DC,
				FunctionGeneratorWaveform.TRGL,
				FunctionGeneratorWaveform.RAMP,
				FunctionGeneratorWaveform.NEGRAMP,
				FunctionGeneratorWaveform.STAIRTRGL,
				FunctionGeneratorWaveform.STARSTEP,
				FunctionGeneratorWaveform.NEGSTAIR,
				FunctionGeneratorWaveform.POSEXP,
				FunctionGeneratorWaveform.NEGEXP,
				FunctionGeneratorWaveform.PFALLEXP,
				FunctionGeneratorWaveform.NFALLEXP,
				FunctionGeneratorWaveform.POSLOG,
				FunctionGeneratorWaveform.NEGLOG,
				FunctionGeneratorWaveform.PFALLLOG,
				FunctionGeneratorWaveform.NFALLLOG,
				FunctionGeneratorWaveform.PFULLWAV,
				FunctionGeneratorWaveform.NFULLWAV,
				FunctionGeneratorWaveform.PHALFWAV,
				FunctionGeneratorWaveform.NHALFWAV,
				FunctionGeneratorWaveform.SINCPULSE,
				FunctionGeneratorWaveform.IMPULSE,
				FunctionGeneratorWaveform.AM,
				FunctionGeneratorWaveform.FM,
				FunctionGeneratorWaveform.CHIRP,
				FunctionGeneratorWaveform.WHITENOISE,
				FunctionGeneratorWaveform.LORENTZPULSE,
				FunctionGeneratorWaveform.ECGSIMULATION
			],

			supportedTriggerModes = [],

			supportedModulations = [
				FunctionGeneratorModulation.NONE,
				FunctionGeneratorModulation.ASK,
				FunctionGeneratorModulation.FSK,
				FunctionGeneratorModulation.PSK,
				FunctionGeneratorModulation.TRIGGER,
				FunctionGeneratorModulation.AM,
				FunctionGeneratorModulation.FM,
				FunctionGeneratorModulation.PM
			],

			commandRetries = commandRetries
		)

		self._portName = serialPort
		self._serial_command_delay = serialCommandDelay
		self._port = None
		self._usedConnect = False
		self._debug = debug

		atexit.register(self.__close)

	# Context handling
	def __enter__(self):
		if self._usedConnect:
			raise ValueError("Cannot use context management (with) on connected port")
		self._connect()
		self._usesContext = True
		return self

	def __exit__(self, exc_type, exc_val, exc_tb):
		self.__close()
		self._usesContext = False

	def __close(self):
		atexit.unregister(self.__close)
		if (self._port is not None) and (self._portName is not None):
			self._off()
			self._port.close()
			self._port = None

	# Connect / Disconnect handler
	def _connect(self):
		if (self._port is None) and (self._portName is not None):
			self._port = serial.Serial(self._portName, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
			self._initialRequests()
		return True
	def _disconnect(self):
		if self._port is not None:
			self.__close()
		return True

	# Internal transmit and receive routines

	def _sendCommand_NoReply(self, cmd):
		if self._debug:
			print(f"FG6900> {cmd}")
		sleep(self._serial_command_delay)
		self._port.write(f"{cmd}\n".encode('ascii'))

	def _sendCommand(self, cmd, binary = False):
		# First clear input buffer ...
		oldtimeout = self._port.timeout
		self._port.timeout = 0
		while True:
			c = self._port.read(1)
			if len(c) <= 0:
				break
		self._port.timeout = oldtimeout

		# Now  transmit command
		if self._debug:
			print(f"FG6900> {cmd}")
		sleep(self._serial_command_delay)
		self._port.write(f"{cmd}\n".encode('ascii'))

		res = []
		while True:
			if not binary:
				c = self._port.read(1).decode('ascii')
			else:
				c = self._port.read(1)

			if len(c) <= 0:
				if self._debug:
					print("[FG6900] Timeout")
				raise CommunicationError_Timeout()
			if ord(c) == 0x0a:
				break
			res.append(c)

		if not binary:
			reply = "".join(res)
		else:
			reply = res

		if self._debug:
			print(f"FG6900< {reply}")

		return reply

	def _off(self):
		try:
			self._set_channel_enabled(0, False)
		except:
			pass

		try:
			self._set_channel_enabled(1, False)
		except:
			pass

		return True

	def _initialRequests(self):
		# Identify and set to a well known state

		self._identification_string = self._id()
		if not self._identification_string.startswith("FY6900-"):
			raise CommunicationError_ProtocolViolation(f"Unknown device identification {self._identification_string} received")

		parts = self._identification_string.split("-")
		try:
			maxfrq = int(parts[1][:-1])
			if self._debug:
				print(f"[FY6900] Maximum frequency {maxfrq} MHz")
			self._maxfrq = maxfrq * 10e6
		except:
			raise CommunicationError_ProtocolViolation(f"Failed to parse maximum frequency in device identification string {self._identification_string}")

		self._serialnumber = self._serial()

	def _id(self):
		res = self._sendCommand("UMO")
		if res is None:
			raise CommunicationError_ProtocolViolation("Invalid response to UMO received")
		res = res.strip()
		return res

	def _serial(self):
		res = self._sendCommand("UID")
		if res is None:
			raise CommunicationError_ProtocolViolation("Invalid response to UMO received")
		res = res.strip()
		return res

	def _set_channel_waveform(self, channel = None, waveform = None, arbitrary = None):
		if (channel is None) or ((waveform is None) and (arbitrary is None)):
			raise ValueError("Missing waveform or channel specification")

		if (waveform is not None) and (arbitrary is not None):
			raise ValueError("Either predefined waveform or arbitrary channel has to be specified, not both")

		if (not isinstance(waveform, FunctionGeneratorWaveform)) and (not isinstance(arbitrary, int)):
			raise ValueError("The waveform has to be specified via FunctionGeneratorWaveform enum")

		waveformNumberMap = {
			FunctionGeneratorWaveform.SINE          : 0,
			FunctionGeneratorWaveform.SQUARE        : 1,
			FunctionGeneratorWaveform.RECTANGLE     : 2,
			FunctionGeneratorWaveform.TRAPEZOID     : 3,
			FunctionGeneratorWaveform.CMOS          : 4,
			FunctionGeneratorWaveform.ADJPULSE      : 5,
			FunctionGeneratorWaveform.DC            : 6,
			FunctionGeneratorWaveform.TRGL          : 7,
			FunctionGeneratorWaveform.RAMP          : 8,
			FunctionGeneratorWaveform.NEGRAMP       : 9,
			FunctionGeneratorWaveform.STAIRTRGL     : 10,
			FunctionGeneratorWaveform.STARSTEP      : 11,
			FunctionGeneratorWaveform.NEGSTAIR      : 12,
			FunctionGeneratorWaveform.POSEXP        : 13,
			FunctionGeneratorWaveform.NEGEXP        : 14,
			FunctionGeneratorWaveform.PFALLEXP      : 15,
			FunctionGeneratorWaveform.NFALLEXP      : 16,
			FunctionGeneratorWaveform.POSLOG        : 17,
			FunctionGeneratorWaveform.NEGLOG        : 18,
			FunctionGeneratorWaveform.PFALLLOG      : 19,
			FunctionGeneratorWaveform.NFALLLOG      : 20,
			FunctionGeneratorWaveform.PFULLWAV      : 21,
			FunctionGeneratorWaveform.NFULLWAV      : 22,
			FunctionGeneratorWaveform.PHALFWAV      : 23,
			FunctionGeneratorWaveform.NHALFWAV      : 24,
			FunctionGeneratorWaveform.SINCPULSE     : 30,
			FunctionGeneratorWaveform.IMPULSE       : 31,
			FunctionGeneratorWaveform.AM            : 33,
			FunctionGeneratorWaveform.FM            : 34,
			FunctionGeneratorWaveform.CHIRP         : 35,
			FunctionGeneratorWaveform.WHITENOISE    : 27,
			FunctionGeneratorWaveform.LORENTZPULSE  : 25,
			FunctionGeneratorWaveform.ECGSIMULATION : 28
		}

		if not isinstance(arbitrary, int):
			if waveform not in waveformNumberMap:
				raise ValueError("The supplied waveform is not supported by the FY6900 directly")
		else:
			if (arbitrary < 0) or (arbitrary >= 64):
				raise ValueError("The supplied arbitrary waveform index is invalid. The FY6900 supports 64 slots (0-63)")

		if (channel == 0) and (not isinstance(arbitrary, int)):
			self._sendCommand(f"WMW{waveformNumberMap[waveform]}")
		elif (channel == 1) and (not isinstance(arbitrary, int)):
			self._sendCommand(f"WFW{waveformNumberMap[waveform]}")
		elif (channel == 0) and (isinstance(arbitrary, int)):
			self._sendCommand(f"WMW{36 + arbitrary:02.0f}")
		elif (channel == 1) and (isinstance(arbitrary, int)):
			self._sendCommand(f"WFW{36 + arbitrary:02.0f}")
		else:
			raise ValueError(f"Channel number {channel} is not supported")

		return True

	def _get_channel_waveform(self, channel = None):
		if channel is None:
			raise ValueError("Channel not supplied")

		waveformNumberMap = {
			 0 : FunctionGeneratorWaveform.SINE,
			 1 : FunctionGeneratorWaveform.SQUARE,
			 2 : FunctionGeneratorWaveform.RECTANGLE,
			 3 : FunctionGeneratorWaveform.TRAPEZOID,
			 4 : FunctionGeneratorWaveform.CMOS,
			 5 : FunctionGeneratorWaveform.ADJPULSE,
			 6 : FunctionGeneratorWaveform.DC,
			 7 : FunctionGeneratorWaveform.TRGL,
			 8 : FunctionGeneratorWaveform.RAMP,
			 9 : FunctionGeneratorWaveform.NEGRAMP,
			10 : FunctionGeneratorWaveform.STAIRTRGL,
			11 : FunctionGeneratorWaveform.STARSTEP,
			12 : FunctionGeneratorWaveform.NEGSTAIR,
			13 : FunctionGeneratorWaveform.POSEXP,
			14 : FunctionGeneratorWaveform.NEGEXP,
			15 : FunctionGeneratorWaveform.PFALLEXP,
			16 : FunctionGeneratorWaveform.NFALLEXP,
			17 : FunctionGeneratorWaveform.POSLOG,
			18 : FunctionGeneratorWaveform.NEGLOG,
			19 : FunctionGeneratorWaveform.PFALLLOG,
			20 : FunctionGeneratorWaveform.NFALLLOG,
			21 : FunctionGeneratorWaveform.PFULLWAV,
			22 : FunctionGeneratorWaveform.NFULLWAV,
			23 : FunctionGeneratorWaveform.PHALFWAV,
			24 : FunctionGeneratorWaveform.NHALFWAV,
			30 : FunctionGeneratorWaveform.SINCPULSE,
			31 : FunctionGeneratorWaveform.IMPULSE,
			33 : FunctionGeneratorWaveform.AM,
			34 : FunctionGeneratorWaveform.FM,
			35 : FunctionGeneratorWaveform.CHIRP,
			27 : FunctionGeneratorWaveform.WHITENOISE,
			25 : FunctionGeneratorWaveform.LORENTZPULSE,
			28 : FunctionGeneratorWaveform.ECGSIMULATION
		}

		if channel == 0:
			wv = self._sendCommand("RMW")
		elif channel == 1:
			wv = self._sendCommand("RFW")
		else:
			raise ValueError(f"Channel number {channel} is invalid")

		try:
			wv = int(wv)
		except:
			raise CommunicationError_ProtocolViolation(f"Received waveform {wv} is not parsable")

		if wv not in waveformNumberMap:
			raise CommunicationError_ProtocolViolation(f"Received waveform number {wv} unknown to library")
		return waveformNumberMap[wv]

	def _get_channel_frequency(self, channel = None):
		if channel == 0:
			res = self._sendCommand("RMF")
		elif channel == 1:
			res = self._sendCommand("RFF")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		try:
			res = float(res)
		except:
			raise CommunicationError_ProtocolViolation(f"Failed to parse frequency response {res}")

		# Frequency is transmitted in micro Hz ...

		if (res < 0) or (res > self._maxfrq):
			raise CommunicationError_ProtocolViolation(f"Received frequency {res} Hz is out of range of supported frequency of this device")

		return res

	def _set_channel_frequency(self, channel = None, frequency = None):
		if (float(frequency) < 0) or (float(frequency) > self._maxfrq):
			raise ValueError(f"Requested frequency {frequency} is out of supported frequency range of this device")

		if channel == 0:
			self._sendCommand(f"WMF{float(frequency):08.6f}")
		elif channel == 1:
			self._sendCommand(f"WFF{float(frequency):08.6f}")

		return True

	def _get_channel_amplitude(self, channel = None):
		if channel == 0:
			res = self._sendCommand("RMA")
		elif channel == 1:
			res = self._sendCommand("RFA")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		try:
			res = float(res) / 10000.0
		except:
			raise CommunicationError_ProtocolViolation(f"Failed to parse amplitude response {res}")

		if (res < 0) or (res > 20):
			raise CommunicationError_ProtocolViolation(f"Reported amplitude {res} is out of range from 0 to 20V")

		return res

	def _set_channel_amplitude(self, channel = None, amplitude = None):
		if (float(amplitude) < 0) or (float(amplitude) > 20):
			raise ValueError(f"Requested amplitude {amplitude} is out of range [0;20]")

		if channel == 0:
			self._sendCommand(f"WMA{float(amplitude):0.5f}")
		elif channel == 1:
			self._sendCommand(f"WFA{float(amplitude):0.5f}")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		return True

	def _get_channel_offset(self, channel = None):
		if channel == 0:
			res = self._sendCommand("RMO")
		elif channel == 1:
			res = self._sendCommand("RFO")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		try:
			res = float(res)
		except:
			raise CommunicationError_ProtocolViolation(f"Failed to parse offset response {res}")

		if res > 10000:
			res = -1.0 * (4294967296 - res) / 1000.0
		else:
			res = res / 1000.0

		if (res < -20) or (res > 20):
			raise CommunicationError_ProtocolViolation(f"Reported offset {res} is out of range from -20 to 20V")

		return res

	def _set_channel_offset(self, channel = None, offset = None):
		if (float(offset) < -20) or (float(offset) > 20):
			raise ValueError(f"Requested offset {offset} is out of range [0;20]")

		if channel == 0:
			self._sendCommand(f"WMO{float(offset):0.5f}")
		elif channel == 1:
			self._sendCommand(f"WFO{float(offset):0.5f}")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		return True

	def _get_channel_duty(self, channel = None):
		if channel == 0:
			res = self._sendCommand("RMD")
		elif channel == 1:
			res = self._sendCommand("RFD")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		try:
			res = float(res) / 1000.0
		except:
			raise CommunicationError_ProtocolViolation(f"Received duty cycle {res} is out of range [0;100]")

		return res

	def _set_channel_duty(self, channel = None, duty = None):
		if (float(duty) < 0) or (float(duty) > 100):
			raise ValueError("Duty cycle has to be in range of [0;99.999] percent")

		# We cannot go to real 100% with this device
		if duty > 99.999:
			duty = 99.999

		if channel == 0:
			self._sendCommand(f"WMD{float(duty):2.3f}")
		elif channel == 1:
			self._sendCommand(f"WFD{float(duty):2.3f}")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		return True

	def _get_channel_phase(self, channel = None):
		if channel == 0:
			res = self._sendCommand("RMP")
		elif channel == 1:
			res = self._sendCommand("RFP")
		else:
			raise ValueError(f"Channel {channel} is invalid")

		try:
			res = float(res) / 1000.0
		except:
			raise CommunicationError_ProtocolViolation(f"Received phase angle {res} is out of range [0;360]")

		if (res < 0) or (res > 360):
			raise CommunicationError_ProtocolViolation(f"Received phase {res} is not in valid range [0;360]")

		return res

	def _set_channel_phase(self, channel = None, phase = None):
		phase = float(phase)

		while phase < 0:
			phase = phase + 360.0
		while phase > 359.999:
			phase = phase - 360.0

		if channel == 0:
			self._sendCommand(f"WMP{float(phase):3.3f}")
		elif channel == 1:
			self._sendCommand(f"WFP{float(phase):3.3f}")
		else:
			raise ValueError(f"Channel {channel} is not a supported channel number")

	def _is_channel_enabled(self, channel = None):
		if channel == 0:
			res = self._sendCommand("RMN")
		elif channel == 1:
			res = self._sendCommand("RFN")
		else:
			raise ValueError(f"Channel {channel} is not a supported channel number")

		try:
			res = int(res)
		except:
			raise CommunicationError_ProtocolViolation(f"Response {res} to RMN/RFN is not known")

		if res == 0:
			return False
		elif res == 255:
			return True
		else:
			raise CommunicationError_ProtocolViolation(f"Response {res} to RMN/RFN is not know")


	def _set_channel_enabled(self, channel = None, enable = None):
		enable = bool(enable)

		if (channel == 0) and enable:
			self._sendCommand("WMN1")
		elif (channel == 0) and not enable:
			self._sendCommand("WMN0")
		elif (channel == 1) and enable:
			self._sendCommand("WFN1")
		elif (channel == 1) and not enable:
			self._sendCommand("WFN0")

		return True

	# Currently not implemented:
	#	Modulation configuration
	#	Measurement (Counter)
	#	Sweeping
	#	Synchronization mode
	#	Buzzer on/off
	#	Uplink mode and status

	# User defined arbitrary waveforms

	def _upload_waveform(self, slot, wavedata, normalize = False):
		slot = int(slot)
		if (slot < 0) or (slot > 63):
			raise ValueError("The FY6900 only offers 64 slots for arbitrary waveforms")

		if len(wavedata) != 8192:
			raise ValueError("Waveform data for FY6900 is required to be exactly 8192 samples long")

		if normalize:
			mi = min(wavedata)
			mx = max(wavedata)
			rng = mx - mi
			if rng == 0:
				rng = 1
			for i in range(len(wavedata)):
				wavedata[i] = ((wavedata[i] - mi) / rng) * 16383

		if (min(wavedata) < 0) or (max(wavedata) > 16383):
			raise ValueError("The 14 bit DDS DAC requires values from 0 to 16383")

		# Now transmit into the given channel ...
		resp = self._sendCommand(f"DDS_WAVE{(slot+1):02.0f}\n")
		if resp != "W":
			raise CommunicationError_ProtocolViolation(f"Failed to upload DDS wave into slot {slot}, unexpected response {resp}")

		# Transmit data ...
		for b in wavedata:
			# Encode as binary structure with words (16 bits) in big endian
			b = struct.pack('>H', int(b))
			self._port.write(b)

		# Send some dummy command that will timeout anyways ...
		for i in range(3):
			try:
				self._id()
			except CommunicationError_Timeout:
				pass

		return True
