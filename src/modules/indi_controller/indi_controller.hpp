#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/esc_status.h>
#include <matrix/math.hpp>
#include <mathlib/math/filter/LowPassFilter2p.hpp>


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
	uORB::Publication<debug_vect_s> _debug_vect_pub{ORB_ID(debug_vect)};
	int _current_px4_mode{0};

	// IMU and Motor
	uORB::Subscription _sensor_gyro_sub{ORB_ID(sensor_gyro)};
	uORB::Subscription _sensor_accel_sub{ORB_ID(sensor_accel)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};

	// Data structures
	sensor_gyro_s _gyro_data{};
	sensor_accel_s _accel_data{};
	esc_status_s _esc_data{};

	// previous omega
	matrix::Vector3f _omega_prev{0.0f, 0.0f, 0.0f};

	// PX4 parameters
	DEFINE_PARAMETERS(
        (ParamFloat<px4::params::INDI_MASS>) _param_indi_mass,
        (ParamFloat<px4::params::INDI_INERTIA_X>) _param_indi_inertia_x,
        (ParamFloat<px4::params::INDI_INERTIA_Y>) _param_indi_inertia_y,
        (ParamFloat<px4::params::INDI_INERTIA_Z>) _param_indi_inertia_z,
        (ParamFloat<px4::params::INDI_ARM_LEN>) _param_indi_arm_len,
        (ParamFloat<px4::params::INDI_CTAU>) _param_indi_ctau,
        (ParamFloat<px4::params::INDI_CT>) _param_indi_ct
    )

	//filters
	math::LowPassFilter2p<float> _filter_accel[3];     // 12 Hz Outer Loop
    math::LowPassFilter2p<float> _filter_f_mot;        // 12 Hz Outer Loop
    math::LowPassFilter2p<float> _filter_alpha[3];     // 6 Hz Inner Loop
    math::LowPassFilter2p<float> _filter_tau_mot[3];   // 6 Hz Inner Loop

};

