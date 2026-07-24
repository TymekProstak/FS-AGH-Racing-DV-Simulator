# Complete vehicle dynamics state. Numeric fields use SI units.

float32 time
int32 step_number

float32 x
float32 y
float32 yaw
float32 yaw_rate

float32 vx
float32 vy
float32 ax
float32 ay

float32 torque
float32 torque_left
float32 torque_right
float32 torque_request

float32 omega_rl
float32 omega_rr
float32 omega_fl
float32 omega_fr

float32 rack_angle
float32 delta_left
float32 delta_rigth
float32 rack_angle_request

float32 fx_fl
float32 fx_fr
float32 fx_rl
float32 fx_rr

float32 fy_fl
float32 fy_fr
float32 fy_rl
float32 fy_rr

float32 fz_fl
float32 fz_fr
float32 fz_rl
float32 fz_rr

float32 slip_angle_fl
float32 slip_angle_fr
float32 slip_angle_rl
float32 slip_angle_rr
float32 slip_angle_body

float32 kappa_fl
float32 kappa_fr
float32 kappa_rl
float32 kappa_rr

float32 total_drag
float32 total_downforce
float32 Power_total

float32 step_dt
