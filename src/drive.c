/*
 * Differential-drive recruitment task
 *
 * The communication and decoding stages provide a target coordinate. Implement
 * drive_to_target() so the simulated differential-drive rover reaches the
 * target using valid left and right wheel velocities.
 */

#include <math.h>
#include <stdbool.h>

#include "drive.h"

/* Provided simulator helpers. Candidates should not modify these functions. */
static float normalize_angle(float angle);
static bool apply_wheel_velocities(struct rover_state *rover,
                                   struct wheel_velocity velocity);

/* Helpers added for this task. */
static float clampf(float value, float min, float max);
static float wrap_heading_error(float angle);
static bool coordinate_is_finite(const struct coordinate *coordinate);
static bool rover_is_valid(const struct rover_state *rover);
static struct wheel_velocity limit_wheel_velocities(struct wheel_velocity velocity);

/*
 * Candidate task
 * --------------
 * A proportional heading controller run on the simulator's own fixed timestep.
 *
 * Each iteration:
 *   1. measure the vector from the rover to the target and stop if it is
 *      already inside TARGET_TOLERANCE;
 *   2. turn the bearing to the target into a heading error folded onto
 *      [-PI, PI), so a target behind the rover is a short turn one way rather
 *      than an almost-full turn the other;
 *   3. ask for an angular velocity proportional to that error, and a forward
 *      velocity that is cut back both by how far off the heading still is and
 *      by how little distance is left;
 *   4. convert that (v, w) pair into the two wheel velocities the simulator
 *      expects, scale them if they exceed what the wheels can do, and step.
 *
 * The loop is bounded by MAX_DRIVE_STEPS, so the function always terminates.
 */
enum drive_status drive_to_target(struct rover_state *rover,
                                  const struct coordinate *target) {
  if ((rover == NULL) || (target == NULL)) {
    return DRIVE_INVALID_INPUT;
  }

  if (!rover_is_valid(rover) || !coordinate_is_finite(target)) {
    return DRIVE_INVALID_INPUT;
  }

  for (int step = 0; step < MAX_DRIVE_STEPS; step++) {
    /* Latitude is north, longitude is east. */
    const float north_error = target->latitude - rover->position.latitude;
    const float east_error = target->longitude - rover->position.longitude;
    const float distance = hypotf(north_error, east_error);

    if (distance <= TARGET_TOLERANCE) {
      return DRIVE_REACHED_TARGET;
    }

    /* Heading zero points east, so the bearing is atan2(north, east). */
    const float desired_heading = atan2f(north_error, east_error);
    const float heading_error =
        wrap_heading_error(desired_heading - rover->heading_rad);

    const float angular_velocity =
        clampf(HEADING_GAIN * heading_error, -MAX_ANGULAR_VELOCITY,
               MAX_ANGULAR_VELOCITY);

    /*
     * Only drive forward to the extent the rover already points at the target.
     * cos() of the heading error is 1 when aimed at it, 0 at ninety degrees
     * off, and is floored at 0 so the rover never reverses away while turning.
     * Capping the speed at the remaining distance stops it overshooting the
     * tolerance band on the last few steps.
     */
    const float forward_gate = clampf(cosf(heading_error), 0.0f, 1.0f);
    const float linear_velocity =
        fminf(MAX_LINEAR_VELOCITY, distance) * forward_gate;

    /*
     * Inverse kinematics of a differential drive. apply_wheel_velocities()
     * reads these back as v = R(wl+wr)/2 and w = R(wr-wl)/L, so inverting it:
     *   wheel = (v +/- w*L/2) / R
     */
    const float half_track = angular_velocity * WHEEL_SEPARATION / 2.0f;
    struct wheel_velocity velocity;
    velocity.left = (linear_velocity - half_track) / WHEEL_RADIUS;
    velocity.right = (linear_velocity + half_track) / WHEEL_RADIUS;

    velocity = limit_wheel_velocities(velocity);

    if (!apply_wheel_velocities(rover, velocity)) {
      return DRIVE_INVALID_COMMAND;
    }
  }

  return DRIVE_MAX_STEPS_EXCEEDED;
}

static float clampf(float value, float min, float max) {
  if (value < min) {
    return min;
  }
  if (value > max) {
    return max;
  }
  return value;
}

/*
 * Folds a heading error onto [-PI, PI).
 *
 * This is deliberately not normalize_angle(). That one keeps the half-open end
 * at +PI, which leaves an exact about-turn sitting on the boundary where the
 * sign of the error - and so the direction the rover spins - depends on
 * rounding. Folding to [-PI, PI) makes an exact 180 degree target always turn
 * the same way instead of dithering between the two equally short arcs.
 */
static float wrap_heading_error(float angle) {
  while (angle >= PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

static bool coordinate_is_finite(const struct coordinate *coordinate) {
  if (coordinate == NULL) {
    return false;
  }
  return isfinite(coordinate->latitude) && isfinite(coordinate->longitude) &&
         isfinite(coordinate->altitude);
}

static bool rover_is_valid(const struct rover_state *rover) {
  if (rover == NULL) {
    return false;
  }
  return coordinate_is_finite(&rover->position) && isfinite(rover->heading_rad);
}

/*
 * Keeps both wheels inside MAX_WHEEL_VELOCITY, which apply_wheel_velocities()
 * rejects outright rather than saturating.
 *
 * Both wheels are scaled by the same factor. Clamping them independently would
 * change the ratio between them, and that ratio is what sets the turn radius,
 * so the rover would quietly curve differently from what the controller asked
 * for exactly when it is turning hardest.
 */
static struct wheel_velocity limit_wheel_velocities(struct wheel_velocity velocity) {
  const float largest = fmaxf(fabsf(velocity.left), fabsf(velocity.right));

  if (largest > MAX_WHEEL_VELOCITY) {
    const float scale = MAX_WHEEL_VELOCITY / largest;
    velocity.left *= scale;
    velocity.right *= scale;

    /* Guard against the scaled value landing a rounding step above the limit
     * and being rejected by apply_wheel_velocities(). */
    velocity.left = clampf(velocity.left, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
    velocity.right = clampf(velocity.right, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
  }

  return velocity;
}

static float normalize_angle(float angle) {
  while (angle > PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

static bool apply_wheel_velocities(struct rover_state *rover,
                                   struct wheel_velocity velocity) {
  if (!isfinite(velocity.left) || !isfinite(velocity.right) ||
      fabsf(velocity.left) > MAX_WHEEL_VELOCITY ||
      fabsf(velocity.right) > MAX_WHEEL_VELOCITY) {
    return false;
  }

  const float linear_velocity =
      WHEEL_RADIUS * (velocity.left + velocity.right) / 2.0f;
  const float angular_velocity =
      WHEEL_RADIUS * (velocity.right - velocity.left) / WHEEL_SEPARATION;

  rover->heading_rad = normalize_angle(
      rover->heading_rad + angular_velocity * DRIVE_DT_SECONDS);
  rover->position.longitude +=
      linear_velocity * cosf(rover->heading_rad) * DRIVE_DT_SECONDS;
  rover->position.latitude +=
      linear_velocity * sinf(rover->heading_rad) * DRIVE_DT_SECONDS;

  return true;
}
