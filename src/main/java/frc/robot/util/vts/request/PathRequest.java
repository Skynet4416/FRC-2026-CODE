// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.vts.request;

import java.util.ArrayList;
import java.util.List;
import lombok.Builder;

@Builder(toBuilder = true)
public class PathRequest {
  /**
   * The segments that make up the path request. <b>Constraints on segment N are applied from the
   * last waypoint of segment N-1 to the last waypoint of segment N.</b>
   */
  @Builder.Default public final List<PathRequestSegment> segments = new ArrayList<>();

  /** Always mark the first waypoint in the first segment as stopped. */
  @Builder.Default public final boolean stopAtStart = true;

  /** Always mark the last waypoint in the last segment as stopped. */
  @Builder.Default public final boolean stopAtEnd = true;

  /** Approximately control how much elapses between each sample */
  @Builder.Default public final double targetDt = 0.05;

  /** Creates a new builder instance with the current state copied. */
  public PathRequestBuilder clone() {
    PathRequestBuilder newBuilder =
        PathRequest.builder()
            .segments(new ArrayList<>(this.segments))
            .stopAtStart(this.stopAtStart)
            .stopAtEnd(this.stopAtEnd);
    return newBuilder;
  }
}
