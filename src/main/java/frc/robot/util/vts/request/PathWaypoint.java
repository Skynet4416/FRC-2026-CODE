// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.vts.request;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.Builder;

@Builder(toBuilder = true)
public class PathWaypoint {
  /** The translation of the waypoint. Required. */
  public final Translation2d translation;

  /** The rotation of the waypoint. Optional (can be null). */
  @Builder.Default public final Rotation2d rotation = null;

  /** The number of samples to use between this waypoint and the <b>next</b> waypoint. */
  @Builder.Default public final int intervals = -1;

  /**
   * Whether the robot velocity must be zero at this waypoint. Note that this property may be
   * overriden for start and end waypoints (see {@link
   * frc.robot.util.vts.request.PathRequest#stopAtStart stopAtStart} and {@link
   * frc.robot.util.vts.request.PathRequest#stopAtEnd stopAtEnd}).
   */
  @Builder.Default public final boolean stopped = false;

  /** Construct a waypoint from a Pose2d. */
  public static PathWaypointBuilder from(Pose2d pose) {
    return PathWaypoint.builder().translation(pose.getTranslation()).rotation(pose.getRotation());
  }

  /** Construct a waypoint from a Translation2d. */
  public static PathWaypointBuilder from(Translation2d translation) {
    return PathWaypoint.builder().translation(translation);
  }
}
