import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        kp = 5
        ki = 0.5
        kd = 0.5
        #mn = 0.
        #mx = 0.2
        mn = decel_limit
        mx = accel_limit
        self.throttle_controller = PID(kp,ki,kd,mn,mx)
        
        tau = 3
        ts = 1
        self.vel_lpf = LowPassFilter(tau,ts)
        
        self.steer_lpf = LowPassFilter(tau = 3, ts = 1)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
        

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        if not dbw_enabled:
            self.throttle_controller.reset()
            tau = 3
            ts = 1
            self.vel_lpf = LowPassFilter(tau,ts)         
            self.steer_lpf = LowPassFilter(tau, ts)
            self.last_time = rospy.get_time()
            return 0., 0., 0,
            
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time     
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering = self.steer_lpf.filt(steering)
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        throttle = self.vel_lpf.filt(throttle)

        brake = 0
        
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY)*self.wheel_radius
            
        return throttle, brake, steering

