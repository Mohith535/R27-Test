#ifndef DRIVE_H
#define DRIVE_H
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/*****************************************************************************
 * Rover parameters
 *
 * These were previously duplicated inside src/drive.c. They live here so the
 * simulator and every caller agree on one set of values.
 ****************************************************************************/

#define PI_F 3.14159265358979323846f

#define WHEEL_RADIUS 0.15f
#define WHEEL_SEPARATION 0.77f
#define MAX_LINEAR_VELOCITY 1.0f
#define MAX_ANGULAR_VELOCITY 2.0f
#define MAX_WHEEL_VELOCITY 10.0f
#define HEADING_GAIN 1.25f

#define TARGET_TOLERANCE 0.10f
#define DRIVE_DT_SECONDS 0.02f
#define MAX_DRIVE_STEPS 6000

/*
 * Latitude and longitude are normalized local simulation coordinates measured
 * in metres. Latitude is the north axis and longitude is the east axis. The
 * differential-drive rover is planar, so altitude is received but not changed.
 */
struct coordinate {
 float latitude;
 float longitude;
 float altitude;
};

/* Heading is in radians: zero points east and positive rotation is CCW. */
struct rover_state {
 struct coordinate position;
 float heading_rad;
};

struct wheel_velocity {
 float left;
 float right;
};

enum drive_status {
 DRIVE_REACHED_TARGET = 0,
 DRIVE_INVALID_INPUT = -1,
 DRIVE_INVALID_COMMAND = -2,
 DRIVE_MAX_STEPS_EXCEEDED = -3
};

/*
 * The helpers this header used to declare (clampf, normlize_angle,
 * coordinate_is_finite, rover_is_valid, limit_wheel_velocities,
 * apply_wheel_velocities) are internal to src/drive.c and stay static there.
 * Declaring them "static" in a shared header meant every translation unit that
 * included drive.h promised a definition it never provided.
 */

enum drive_status drive_to_target(struct rover_state *rover,const struct coordinate *target);


#endif
