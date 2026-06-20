#include "indi_controller.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <string.h>

// constructor
IndiController::IndiController() :
	ModuleParams(nullptr)
{
	// init code
}

// main loop
void IndiController::run(){
	PX4_INFO("INDI Controller module started");

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


					// sending ACK back
					debug_key_value_s ack_msg{};
					strncpy(ack_msg.key, "mode_ack", 10);
					ack_msg.value = (float)_current_px4_mode;
					ack_msg.timestamp = hrt_absolute_time();

					// publish to uORB
					_debug_key_value_pub.publish(ack_msg);


				}

			}
		}

		// MATH GOES HERE


		// sleep for 4 ms
		px4_usleep(4000);
	}

	PX4_INFO("INDI Controller module exiting");

}


// allocate resources
int IndiController::task_spawn(int argc, char *argv[]){
	_task_id = px4_task_spawn_cmd("indi_controller", 	//name of the thread
								SCHED_DEFAULT, 			// scheduling algorithm
								SCHED_PRIORITY_ATTITUDE_CONTROL,	// priority - critical
								2000,								// <-- Changed semicolon to comma
								(px4_main_t)&run_trampoline,
								(char *const *)argv);				// <-- Fixed the closing parenthesis and semicolon
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


// ---------------------------------------------------------
// Main entry point for the module
// ---------------------------------------------------------
int indi_controller_main(int argc, char *argv[])
{
    return IndiController::main(argc, argv);
}
