#ifndef __TILT_SCAN_TYPES_HPP__
#define __TILT_SCAN_TYPES_HPP__

#include <base/Pose.hpp>
#include <base/samples/Joints.hpp>
#include <stdint.h>

namespace tilt_scan
{
	struct Configuration
	{
		Configuration()
		: mode(CONTINUOUS_SWEEPING),
		max_lines(200),
		sweep_angle_min(-1.0),
		sweep_angle_max(1.0),
		sweep_velocity_up(2.0),
		sweep_velocity_down(0.5),
		sweep_servo_name("dynamixel"),
		sweep_back_and_forth(false)
		{}

		enum Mode {
			CONTINUOUS_SWEEPING,
			TRIGGERED_SWEEPING,
		};

		/** current mode */
		Mode mode;

		/** maximum number of lines until the scan is considered complete */
		int max_lines;

		/** minimum angle value to use when sweeping in rad */
		float sweep_angle_min;

		/** maximum angle to use when sweeping in rad */
		float sweep_angle_max;

		/** sweep velocity up in rad/s */
		float sweep_velocity_up;

		/** sweep velocity down in rad/s, by default the point cloud is only generated while sweeping down */
		float sweep_velocity_down;

		/** name of the servo to sweep */
		std::string sweep_servo_name;

		/** also take a pointcloud while sweeping up */
		bool sweep_back_and_forth;
	};
}

#endif