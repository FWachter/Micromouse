/* Implementation file for state model class name
 * Author: Frederick Wachter - wachterfreddy@gmail.com
 * Created: 2017-05-20
 */

#include <math.h>

#include "StateModel.h"

StateModel::StateModel(double x, double y, double theta, double vt, double wt) {
	this->_state[0] = x;
	this->_state[1] = y;
	this->_state[2] = theta;
	this->_state[3] = vt;
	this->_state[4] = wt;
}

StateModel::setControl(double u1, double u2) {
	this->_control[0] = u1;
	this->_control[1] = u2;
}

StateModel::setNoise(double v1, double v2) {
	this->_noise[0] = v1;
	this->_noise[1] = v2;
}

StateModel::setRobotConstants(double time_constant, double volts_to_accel, double wheel_radius, 
		double gear_ratio, double wheels_to_cg, double wheel_distance, double cg_to_sensors, 
		double sensor_distance) {
	this->_robot_constants[0] = time_constant;
	this->_robot_constants[1] = volts_to_accel;
	this->_robot_constants[2] = wheel_radius;
	this->_robot_constants[3] = gear_ratio;
	this->_robot_constants[4] = wheels_to_cg;
	this->_robot_constants[5] = wheel_distance;
	this->_robot_constants[6] = cg_to_sensors;
	this->_robot_constants[7] = sensor_distance;
}

void StateModel::updateState(void) {
    _state_dot[0] = _state[3]*cos(_state[2]) - _robot_constants[4]*_state[4]*sin(_state[2]);
    _state_dot[1] = _state[3]*sin(_state[2]) + _robot_constants[4]*_state[4]*cos(_state[2]);
    _state_dot[2] = _state[4];
    _state_dot[3] = -_robot_constants[0]*_state[3] + _robot_constants[1]*_robot_constants[3]*
    		_robot_constants[2]/2.0*(_control[0] + _noise[0]);
    _state_dot[4] = -_robot_constants[0]*_state[4] + _robot_constants[1]*_robot_constants[3]*
    		_robot_constants[2]/_robot_constants[5]*(_control[1] + _noise[1]);
}
