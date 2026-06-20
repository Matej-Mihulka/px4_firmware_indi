#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>


// this is so it is compiled such that OS written in C can understand it
extern "C" __EXPORT int indi_controller_main(int argc, char *argv[]);

// inherit stuff from ModuleBase
class IndiController : public ModuleBase<IndiController>, public ModuleParams
{
public:
	IndiController();
	~IndiController() = default;

	static int task_spawn(int argc, char *argv[]);

	static IndiController *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	void run() override;

private:
	// listening to MAVROS/ROS
	uORB::Subscription _debug_key_value_sub{ORB_ID(debug_key_value)};

	uORB::Publication<debug_key_value_s> _debug_key_value_pub{ORB_ID(debug_key_value)};

	int _current_px4_mode{0};
};

