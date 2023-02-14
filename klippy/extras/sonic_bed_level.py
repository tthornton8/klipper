import time
import math
from numpy import arange

class SonicBedLevel:
	cmd_VIBRATE_help = ("Vibrate the extruder with the frequency set in the config.")
	cmd_STOP_VIBRATE_help = ("Stop vibrating the extruder")

	def __init__(self, config):
		self.printer 	   = config.get_printer()
		self.freq 		   = config.getfloat('freq', 50., above=0.)
		self.incr 		   = config.getfloat('incr', 0.1, above=0.)
		self.dt   		   = config.getfloat('dt',   3, above=0.)
		self.accel_per_hz  = config.getfloat('accel_per_hz', 75., above=0.)
		self.dz            = config.getfloat('dz', 0.01)
		self.probe_pin     = config.get('probe_pin')
		self.speed         = self.dz / self.dt 
		self.check_dt 	   = config.getfloat('check_dt', 0.1, above=0.0) 

		self.gcode = self.printer.lookup_object('gcode')
		self.gcode.register_command("VIBRATE_EXTRUDER",
					self.cmd_VIBRATE,
					desc=self.cmd_VIBRATE_help)
		self.gcode.register_command("STOP_VIBRATE_EXTRUDER",
					self.cmd_STOP_VIBRATE,
					desc=self.cmd_STOP_VIBRATE_help)

		self.vibr = False

		ppins = self.printer.lookup_object('pins')
		pin_params = ppins.lookup_pin(self.probe_pin, can_invert=True, can_pullup=True)
		mcu = pin_params['chip']
		self.mcu_endstop = mcu.setup_pin('endstop', pin_params)

	def cmd_VIBRATE(self, gcmd, all_endstop_trigger=None):
		self.toolhead = self.printer.lookup_object('toolhead')

		systime = self.printer.get_reactor().monotonic()
		toolhead_info = self.toolhead.get_status(systime)
		self.old_max_accel = toolhead_info['max_accel']
		self.old_max_accel_to_decel = toolhead_info['max_accel_to_decel']
		max_accel = self.freq * self.accel_per_hz
		self.all_endstop_trigger = all_endstop_trigger
		self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    max_accel, max_accel))

		self._generate_moves()

		self.vibr = True
		t = time.time()
		self.i = 0
		dti = 0
		touch = False
		while self.vibr:
			self._vibrate_move()
			
			dt = time.time() - t
			
			if (dt-dti) >  self.check_dt: 
				touch = self._check_touch()
				dti += self.check_dt                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
			if touch:
				self.vibr = False
				self._get_pos()	
				self.gcode.respond_info( "Endstop hit: " + str(self.last_kinematics_pos) )

			self.i += 1

		self.gcode.respond_info( "Finished Probe" )
		self._reset_accel()

	def _generate_moves(self):
		X, Y, Z, E = self.toolhead.get_position()

		t_seg   = 1. / self.freq
		n_steps = int(self.dt // t_seg)
		accel = self.accel_per_hz * self.freq
		self.max_v = accel * t_seg

		z_move = list(arange(Z, Z + self.dz, self.dz/n_steps))                   # monotonically increasing z step
		e_move = [E + self.incr * ( (-1)**i ) for i in range(n_steps)]          # oscillating extruder moves over the entire vibrate time
		x_move = [X for _ in e_move]										 # constant x
		y_move = [Y for _ in e_move]									     # constant y

		self.xyze_move = list(zip(x_move, y_move, z_move, e_move))

		self.gcode.respond_info( "n_steps = " +str(n_steps))
		self.gcode.respond_info( "t_seg = " +str(t_seg))
		self.gcode.respond_info( "speed = " +str(self.speed))

		with open('/home/pi/klipper/moves.csv', 'w+') as f:
			for z, e, x, y in self.xyze_move:
				f.write("G1 X{0} Y{1} Z{2} E{3} F{4} \n".format(x, y, z, e, self.speed))

	def _check_touch(self):
		print_time = self.toolhead.get_last_move_time()
		res = self.mcu_endstop.query_endstop(print_time)

		return res

	def _get_pos(self):
		toolhead_pos = self.toolhead.get_position()
		self.toolhead.flush_step_generation()
		kin = self.toolhead.get_kinematics()
		kin_spos = {s.get_name(): s.get_commanded_position()
					for s in kin.get_steppers()}
		kin_pos = kin.calc_position(kin_spos)
		self.last_toolhead_pos = toolhead_pos
		self.last_kinematics_pos = kin_pos

	def _vibrate_move(self):
		if self.i >= len(self.xyze_move):
			self.vibr = False
			return

		X, Y, Z, E = self.xyze_move[self.i]

		# if self.all_endstop_trigger is not None:
		# 	self.toolhead.drip_move([X, Y, Z, E], self.speed, self.all_endstop_trigger)
		# else:
		self.toolhead.move([X, Y, Z, E], self.speed)


	def _reset_accel(self):
		self.gcode.run_script_from_command(
                "SET_VELOCITY_LIMIT ACCEL=%.3f ACCEL_TO_DECEL=%.3f" % (
                    self.old_max_accel, self.old_max_accel_to_decel))

	def cmd_STOP_VIBRATE(self, gcmd):
		self.vibr = False

	

def load_config(config):
    return SonicBedLevel(config)