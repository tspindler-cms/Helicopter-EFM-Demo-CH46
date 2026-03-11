-- FM_Config.lua for CH-46D Sea Knight with EFM
-- MOI and suspension must match vwv_ch46d_base.lua (wheeled tricycle gear)
-- Collision names must match CH-46D 3D model - verify in ModelViewer if suspension fails

-- Wheeled oleo strut suspension for CH-46D (tricycle: nose + 2 main)
-- CH-46D: lead_stock_main=0.38m, lead_stock_support=0.28m, M_nominal~9435 kg
function wheeledSuspension(collisionLine, argAmortizer, strokeM)
	return {
		self_attitude = false,
		yaw_limit = math.rad(0.0),

		amortizer_min_length = 0.0,
		amortizer_max_length = strokeM,
		amortizer_basic_length = strokeM,
		amortizer_spring_force_factor = 150000.0,   -- stiffer for ~9t aircraft
		amortizer_spring_force_factor_rate = 1,
		amortizer_static_force = 25000.0,          -- ~2500 kg per strut
		amortizer_reduce_length = strokeM,
		amortizer_direct_damper_force_factor = 15000,
		amortizer_back_damper_force_factor = 8000,

		wheel_radius = 0.25,                       -- ~0.5m diameter
		wheel_static_friction_factor = 0.8,
		wheel_side_friction_factor = 0.6,
		wheel_roll_friction_factor = 0.02,
		wheel_glide_friction_factor = 0.8,
		wheel_damage_force_factor = 500.0,
		wheel_damage_speedX = 80,
		wheel_damage_delta_speedX = 10.0,

		arg_post = -1,
		arg_amortizer = argAmortizer,
		arg_wheel_yaw = -1,
		arg_wheel_rotation = -1,
		damage_element = 83,
		collision_shell_name = collisionLine,
	}
end

EFM = {
	-- CH-46D from vwv_ch46d_base.lua: {Rl, Yw, Ptch, POI} [kg*m^2]
	-- Wrong MOI causes violent oscillations/explosions on spawn
	center_of_mass = {0, 0, 0},
	moment_of_inertia = {24500, 185000, 172000, -2500},

	-- CH-46D tricycle: 0=nose (arg 2), 1=left main (arg 6), 2=right main (arg 4)
	-- Collision names: try Line_WHEEL_* or WHEEL_* - verify in ModelViewer if unstable
	suspension = {
		wheeledSuspension("Line_WHEEL_F", 2, 0.28),   -- nose, 0.28m stroke
		wheeledSuspension("Line_WHEEL_L", 6, 0.38),   -- left main, 0.38m
		wheeledSuspension("Line_WHEEL_R", 4, 0.38),   -- right main, 0.38m
	},
	disable_built_in_oxygen_system = false,
}
