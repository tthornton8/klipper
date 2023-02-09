import time

class SonicBedLevel:
	cmd_VIBRATE_help = ("Vibrate the extruder with the frequency set in the config.")
	cmd_STOP_VIBRATE_help = ("Stop vibrating the extruder")

	def __init__(self, config):
		self.printer = config.get_printer()
		self.freq = config.getfloat('freq', 50., above=0.)
		self.incr = config.getfloat('incr', 0.1, above=0.)
		self.dt   = config.getfloat('dt',   3, above=0.)
		self.accel_per_hz = config.getfloat('accel_per_hz', 75., above=0.)

		self.gcode = self.printer.lookup_object('gcode')
		self.gcode.register_command("VIBRATE_EXTRUDER",
					self.cmd_VIBRATE,
					desc=self.cmd_VIBRATE_help)
		self.gcode.register_command("STOP_VIBRATE_EXTRUDER",
					self.cmd_STOP_VIBRATE,
					desc=self.cmd_STOP_VIBRATE_help)

		self.vibr = False

		pins = self.printer.lookup_object("pins")

	def cmd_VIBRATE(self, gcmd):
		toolhead = self.printer.lookup_object('toolhead')

		systime = self.printer.get_reactor().monotonic()
		toolhead_info = toolhead.get_status(systime)
		old_max_accel = toolhead_info['max_accel']
		old_max_accel_to_decel = toolhead_info['max_accel_to_decel']
		max_accel = self.freq * self.accel_per_hz
		self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    max_accel, max_accel))

		self.vibr = True
		t = time.time()
		while self.vibr:
			X, Y, Z, E = toolhead.get_position()

			t_seg = .25 / self.freq
			accel = self.accel_per_hz * self.freq
			max_v = accel * t_seg
			toolhead.move([X, Y, Z, E + self.incr], max_v)
			toolhead.move([X, Y, Z, E], max_v)

			self.incr *= -1

			if time.time() - t > self.dt:
				self.vibr = False

		self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    old_max_accel, old_max_accel_to_decel))

	def cmd_STOP_VIBRATE(self, gcmd):
		self.vibr = False

	

def load_config(config):
    return SonicBedLevel(config)