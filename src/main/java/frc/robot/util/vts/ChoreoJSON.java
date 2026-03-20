// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.vts;

import java.util.List;
import java.util.Map;
import lombok.Builder;

/*
 * A template to create JSON files through Jackson ObjectMapper
 */
class ChoreoJSON {
  @Builder
  public record ChorFileRoot(
      String name,
      int version,
      String hashcode,
      String type,
      Variable variables,
      RobotConfig config,
      List<Object> generationFeatures,
      Codegen codegen) {}

  @Builder
  public record RobotConfig(
      ModuleConfig frontLeft,
      ModuleConfig backLeft,
      ExpVal mass,
      ExpVal inertia,
      ExpVal gearing,
      ExpVal radius,
      ExpVal vmax,
      ExpVal tmax,
      ExpVal cof,
      BumperConfig bumper,
      ExpVal differentialTrackWidth) {}

  @Builder
  public record ModuleConfig(ExpVal x, ExpVal y) {}

  @Builder
  public record BumperConfig(ExpVal front, ExpVal side, ExpVal back) {}

  @Builder
  public record Variable(Map<Object, Object> expressions, Map<Object, Object> poses) {}

  @Builder
  public record Codegen(String root, boolean genVars, boolean genTrajData, boolean useChoreoLib) {}

  @Builder
  public record Waypoint(
      Object x,
      Object y,
      Object heading,
      int intervals,
      boolean split,
      boolean fixTranslation,
      boolean fixHeading,
      boolean overrideIntervals) {}

  @Builder
  public record Constraint(Object from, Object to, Map<String, Object> data, boolean enabled) {}

  @Builder
  public record Sample(
      double t,
      double x,
      double y,
      double heading,
      double vx,
      double vy,
      double omega,
      double ax,
      double ay,
      double alpha,
      double[] fx,
      double[] fy) {}

  @Builder
  public record TrajFileRoot(
      String name,
      int version,
      Snapshot snapshot,
      Params params,
      TrajectorySection trajectory,
      List<Integer> events) {}

  @Builder
  public record Snapshot(List<Waypoint> waypoints, List<Constraint> constraints, double targetDt) {}

  @Builder
  public record Params(List<Waypoint> waypoints, List<Constraint> constraints, ExpVal targetDt) {}

  @Builder
  public record TrajectorySection(
      RobotConfig config,
      String sampleType,
      List<Double> waypoints,
      List<Sample> samples,
      List<Integer> splits) {}

  @Builder
  public record ExpVal(String exp, double val) {}
}
