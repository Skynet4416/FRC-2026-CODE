// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.vts.request;

import frc.robot.util.vts.request.PathRequest.PathRequestBuilder;
import frc.robot.util.vts.request.PathRequestSegment.PathRequestSegmentBuilder;
import java.util.ArrayList;
import java.util.List;

public class PathRequestHelpers {
  /** Add segments to a path request. */
  public static PathRequestBuilder segments(
      PathRequestBuilder builder, PathRequestSegment... segments) {
    List<PathRequestSegment> list = new ArrayList<>(builder.build().segments);
    for (var segment : segments) {
      list.add(segment);
    }
    return builder.segments(list);
  }

  public static PathRequestBuilder segments(
      PathRequestBuilder builder, List<PathRequestSegment> segments) {
    List<PathRequestSegment> list = new ArrayList<>(builder.build().segments);
    for (var segment : segments) {
      list.add(segment);
    }
    return builder.segments(list);
  }

  /** Add waypoints to a path request. */
  public static PathRequestSegmentBuilder waypoints(
      PathRequestSegmentBuilder builder, PathWaypoint... waypoints) {
    List<PathWaypoint> list = new ArrayList<>(waypoints.length);
    for (var waypoint : waypoints) {
      list.add(waypoint);
    }
    return builder.waypoints(list);
  }

  /** Constraint the path request to a straight line. */
  public static PathRequestSegmentBuilder straightLine(PathRequestSegmentBuilder builder) {
    return builder.keepInLaneWidth(0.01);
  }

  private PathRequestHelpers() {}
}
