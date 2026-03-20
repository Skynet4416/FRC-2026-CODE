// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.vts.request;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import lombok.Builder;

@Builder(toBuilder = true)
public class PathRequestSegment {
  /** The waypoints that make up the path request. */
  @Builder.Default public final List<PathWaypoint> waypoints = new ArrayList<>();

  /**
   * Do not allow the robot velocity to exceed the specified value. Negative to disable constraint.
   * Measured in meters per second.
   */
  @Builder.Default public final double maxVelocity = -1.0;

  /**
   * Do not allow the robot acceleration to exceed the specified value. Negative to disable
   * constraint. Measured in meters per second squared.
   */
  @Builder.Default public final double maxAcceleration = -1.0;

  /**
   * Do not allow the robot angular velocity to exceed the specified value. Negative to disable
   * constraint. Measured in radians.
   */
  @Builder.Default public final double maxAngularVelocity = -1.0;

  /** Forces the the robot to face its front or back to a given point, within a given tolerance */
  @Builder.Default public final PointTarget pointAt = null;

  /**
   * Keep the robot within a rectangular lane defined by the start and end waypoints, where the
   * value is the allowed distance from the line directly connecting these waypoints. Negative to
   * disable. Measured in meters.
   */
  @Builder.Default public final double keepInLaneWidth = -1.0;

  /** Keep the robot (including bumpers) out of the areas specified by the circles. */
  @Builder.Default public final List<KeepOutCircle> keepOutCircles = new ArrayList<>();

  public static record PointTarget(Translation2d target, Rotation2d tolerance, Boolean flip) {}

  public static record KeepOutCircle(Translation2d center, double radius) {}
}
