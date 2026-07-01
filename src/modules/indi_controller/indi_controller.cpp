#include "indi_controller.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <string.h>

// constructor
IndiController::IndiController() :
	ModuleParams(nullptr)
{
	_omega_prev.setZero();

	// initialize 12Hz Outer Loop Filters
    for (int i=0; i<3; i++) _filter_accel[i].set_cutoff_frequency(300.0f, 12.0f);
    _filter_f_mot.set_cutoff_frequency(300.0f, 12.0f);

    // initialize 6Hz Inner Loop Filters
    for (int i=0; i<3; i++) _filter_alpha[i].set_cutoff_frequency(300.0f, 6.0f);
    for (int i=0; i<3; i++) _filter_tau_mot[i].set_cutoff_frequency(300.0f, 6.0f);
}

// main loop
void IndiController::run(){
	PX4_INFO("INDI Controller module started");

	// pulls values from memory
	updateParams();

	float mass = _param_indi_mass.get();
	float l    = _param_indi_arm_len.get();
    float k    = _param_indi_ctau.get();   // Torque constant
    float c_T  = _param_indi_ct.get();     // Thrust coefficient
    matrix::Vector3f J(_param_indi_inertia_x.get(), _param_indi_inertia_y.get(), _param_indi_inertia_z.get());

	PX4_INFO("INDI Params Loaded -> Mass: %.2f kg, Arm: %.2f m", (double)mass, (double)l);

	const float dt = 0.003333f;

	// run while module is active
	while (!should_exit()){

		debug_key_value_s debug_msg;

		// Listen to MAVROS degub message
		if (_debug_key_value_sub.update(&debug_msg)){

			// key == px4 mode
			if (strncmp(debug_msg.key, "px4_mode", 10) == 0){

				int requested_mode = (int)debug_msg.value;

				// new mode requested
				if (requested_mode != _current_px4_mode){
					PX4_INFO("INDI Controller changing to mode %d", requested_mode);
					_current_px4_mode = requested_mode;
				}


				// sending ACK back
				debug_vect_s ack_msg{};
				strncpy(ack_msg.name, "mode_ack", 10);
				ack_msg.x = (float)_current_px4_mode;
					ack_msg.timestamp = hrt_absolute_time();

				// publish to uORB
				_debug_vect_pub.publish(ack_msg);




			}
		}


		// get sensor and telemetry data

		if (_sensor_gyro_sub.updated()){
			_sensor_gyro_sub.copy(&_gyro_data);
		}

		if (_sensor_accel_sub.updated()){
			_sensor_accel_sub.copy(&_accel_data);
		}

		if (_esc_status_sub.updated()){
			_esc_status_sub.copy(&_esc_data);
		}

		// calculate angular acceleration
		matrix::Vector3f omega(_gyro_data.x, _gyro_data.y, _gyro_data.z);
		matrix::Vector3f dot_omega = (omega - _omega_prev) / dt;
		_omega_prev = omega;


		// calculate raw motor thrust and torque from RPM

		float f_mot_raw = 0.0f;
		matrix::Vector3f tau_mot_raw(0.0f, 0.0f, 0.0f);
		float f_i[4] = {0.0f};

		for (int i = 0; i < 4; i++){
			float rpm = _esc_data.esc[i].esc_rpm;
			f_i[i] = c_T * (rpm * rpm);
			f_mot_raw += f_i[i];	// sum for total thrust
		}

		// Map individual forces to moments for PX4 Standard Quad-X
        // Roll (+ is right wing down): Left motors (1, 2) add roll, Right motors (0, 3) reduce roll.
        tau_mot_raw(0) = l * (-f_i[0] + f_i[1] + f_i[2] - f_i[3]);

        // Pitch (+ is nose up): Front motors (0, 2) add pitch, Rear motors (1, 3) reduce pitch.
        tau_mot_raw(1) = l * (f_i[0]  - f_i[1] + f_i[2] - f_i[3]);

        // Yaw (+ is nose right): CCW motors (0, 1) add yaw, CW motors (2, 3) reduce yaw.
        tau_mot_raw(2) = k * (f_i[0]  + f_i[1] - f_i[2] - f_i[3]);



		// apply filters
		// 12 Hz Outer Loop Filter
        float f_mot_filt = _filter_f_mot.apply(f_mot_raw);

        // 6 Hz Inner Loop Filter
        matrix::Vector3f dot_omega_filt(
            _filter_alpha[0].apply(dot_omega(0)),
            _filter_alpha[1].apply(dot_omega(1)),
            _filter_alpha[2].apply(dot_omega(2))
        );



		// print debug data
        static int print_counter = 0;
        if (print_counter++ % 300 == 0) {
            PX4_INFO("INDI Debug | f_mot_filt: %.2f N | dot_omega_x: %.2f",
                     (double)f_mot_filt,
                     (double)dot_omega_filt(0));
        }


		// sleep for 3.33 ms -> ~300 Hz loop
		px4_usleep(3333);
	}

	PX4_INFO("INDI Controller module exiting");

}


// allocate resources
int IndiController::task_spawn(int argc, char *argv[]){
	_task_id = px4_task_spawn_cmd("indi_controller", 	//name of the thread
								SCHED_DEFAULT, 			// scheduling algorithm
								SCHED_PRIORITY_ATTITUDE_CONTROL,	// priority - critical
								2000,
								(px4_main_t)&run_trampoline,
								(char *const *)argv);
	if (_task_id < 0){
		_task_id = -1;
		return -errno;
	}
	return 0;
}

// build the module
IndiController *IndiController::instantiate(int argc, char *argv[]){
	return new IndiController();
}

int IndiController::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int IndiController::print_usage(const char *reason)
{
    if (reason) { PX4_WARN("%s\n", reason); }
    PRINT_MODULE_DESCRIPTION("Custom INDI Controller Module");
    PRINT_MODULE_USAGE_NAME("indi_controller", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}


int indi_controller_main(int argc, char *argv[])
{
    return IndiController::main(argc, argv);
}
