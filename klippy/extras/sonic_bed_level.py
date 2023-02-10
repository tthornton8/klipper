import time

class SonicBedLevel:
	cmd_VIBRATE_help = ("Vibrate the extruder with the frequency set in the config.")
	cmd_STOP_VIBRATE_help = ("Stop vibrating the extruder")

	def __init__(self, config):
		self.printer 	   = config.get_printer()
		self.freq 		   = config.getfloat('freq', 50., above=0.)
		self.incr 		   = config.getfloat('incr', 0.1, above=0.)
		self.dt   		   = config.getfloat('dt',   3, above=0.)
		self.accel_per_hz  = config.getfloat('accel_per_hz', 75., above=0.)
		self.dz            = config.getfloat('dz', 0.01, above=0.01)
		self.speed         = self.dz / self.dt 

		self.gcode = self.printer.lookup_object('gcode')
		self.gcode.register_command("VIBRATE_EXTRUDER",
					self.cmd_VIBRATE,
					desc=self.cmd_VIBRATE_help)
		self.gcode.register_command("STOP_VIBRATE_EXTRUDER",
					self.cmd_STOP_VIBRATE,
					desc=self.cmd_STOP_VIBRATE_help)

		self.vibr = False

	def cmd_VIBRATE(self, gcmd):
		toolhead = self.printer.lookup_object('toolhead')

		systime = self.printer.get_reactor().monotonic()
		toolhead_info = toolhead.get_status(systime)
		self.old_max_accel = toolhead_info['max_accel']
		self.old_max_accel_to_decel = toolhead_info['max_accel_to_decel']
		max_accel = self.freq * self.accel_per_hz
		self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    max_accel, max_accel))

		self._generate_moves(toolhead)

		self.vibr = True
		t = time.time()
		while self.vibr:
			self._vibrate_move(toolhead)

			if time.time() - t > self.dt:
				self.vibr = False
			
			touch = self._check_touch()
			if touch:
				self.vibr = False
				self._get_pos()	
				self.gcode.respond_info( str(self.last_kinematics_pos) )

		self.gcode.respond_info( "Finished Probe" )
		self._reset_accel()

	def _generate_moves(self, toolhead):
		X, Y, Z, E = toolhead.get_position()

		t_seg   = 1./self.freq
		t_total = self.dz/self.speed

		self.z_move = list(range(Z, Z - self.dz, self.dz/t_total))                   # monotonically increasing z step
		self.e_move = [E + self.incr * ( (-1)**i ) for i in range(self.dt // t_seg)] # oscillating extruder moves over the entire vibrate time
		self.x_move = [X for _ in self.e_move]										 # constant x
		self.y_move = [Y for _ in self.e_move]									     # constant y

	def _check_touch(self):
		return False

	def _get_pos(self, toolhead):
		toolhead_pos = toolhead.get_position()
		toolhead.flush_step_generation()
		kin = toolhead.get_kinematics()
		kin_spos = {s.get_name(): s.get_commanded_position()
					for s in kin.get_steppers()}
		kin_pos = kin.calc_position(kin_spos)
		self.last_toolhead_pos = toolhead_pos
		self.last_kinematics_pos = kin_pos

	def _vibrate_move(self, toolhead):
		X, Y, Z, E = self.x_move.pop(0), self.y_move.pop(0), self.z_move.pop(0), self.e_move.pop(0)
		toolhead.manual_move([X, Y, Z, E], self.speed)

		if not len(self.e_move):
			self.vibr = False

	def _reset_accel(self):
		self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    self.old_max_accel, self.old_max_accel_to_decel))

	def cmd_STOP_VIBRATE(self, gcmd):
		self.vibr = False

	

def load_config(config):
    return SonicBedLevel(config)