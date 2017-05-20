/* Header file for state model class name
 * Author: Frederick Wachter - wachterfreddy@gmail.com
 * Created: 2017-05-20
 */

#ifndef MICROMOUSE_STATE_MODEL
#define MICROMOUSE_STATE_MODEL

class StateModel {
public:

	// Constructors
	StateModel(double, double, double, double, double);
	/* Constructor to set initial states
	 * Author: Frederick Wachter
	 * Created: 2017-05-20
	 */

	void setControl(double, double);
	/* Sets control parameters
	 * Author: Frederick Wachter
	 * Created: 2017-05-20
	 */

	void setNoise(double, double);
	/* Sets noise parameters
	 * Author: Frederick Wachter
	 * Created: 2017-05-20
	 */

	void setRobotConstants(double, double, double, double, double, double, double, double);
	/* Sets all constants for the robot model
	 * Author: Frederick Wachter
	 * Created: 2017-05-20
	 */

	void updateState();
	/* Update the states
	 * Author: Frederick Wachter
	 * Created: 2017-05-20
	 */

private:
	_states[5]
	_states_dot[5]
	_control[2]
	_noise[2]
	_robot_constants[8]

};

#endif


