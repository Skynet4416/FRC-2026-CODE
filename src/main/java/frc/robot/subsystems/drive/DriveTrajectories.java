// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.AutoFieldConstants.*;
import frc.robot.FieldConstants;
import frc.robot.util.geometry.GeomUtil;
import frc.robot.util.vts.request.PathRequest;
import frc.robot.util.vts.request.PathRequest.PathRequestBuilder;
import frc.robot.util.vts.request.PathRequestHelpers;
import frc.robot.util.vts.request.PathRequestSegment;
import frc.robot.util.vts.request.PathRequestSegment.PointTarget;
import frc.robot.util.vts.request.PathWaypoint;
import java.util.HashMap;
import java.util.Map;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({PathRequestHelpers.class, GeomUtil.class})
public class DriveTrajectories {
  public static final Map<String, PathRequest> paths = new HashMap<>();
  public static final PointTarget hubTarget =
      new PointTarget(
          FieldConstants.Hub.innerCenterPoint.toTranslation2d(), Rotation2d.fromDegrees(10), true);
  public static final double substantialAimUntilXDepot =
      FieldConstants.Depot.rightCorner.getX() + DriveConstants.fullWidthX / 2.0 + 0.15;
  public static final double substantialAimUntilXOutpost =
      FieldConstants.Outpost.centerPoint.getX() + DriveConstants.fullWidthX / 2.0 + 0.85;

  static {
    paths.put(
        // Tests
        "Example",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(3.0, 2.0, Rotation2d.kPi))
                            .stopped(true)
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(new Pose2d(3.0, 5.0, Rotation2d.kPi)).build())
                    .straightLine()
                    .build())
            .build());

    paths.put(
        "DriveForward1m",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(1.0, 0.0, Rotation2d.kZero))
                            .stopped(true)
                            .build())
                    .build())
            .build());

    paths.put(
        "DriveForward1mWhileTurn90",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(1.0, 0.0, Rotation2d.kCW_90deg))
                            .stopped(true)
                            .build())
                    .build())
            .build());

    paths.put(
        "DriveForward1mWhileTurn90VelLim",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(1.0, 0.0, Rotation2d.kCW_90deg))
                            .stopped(true)
                            .build())
                    .maxVelocity(0.5)
                    .build())
            .build());

    // Start --> Neutral Zone
    paths.put(
        "bumpLeftInnerToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting bump pose
                        PathWaypoint.from(
                                new Pose2d(Bump.leftInner.translation(), Rotation2d.kZero))
                            .build(),
                        // Just after bump
                        PathWaypoint.from(
                                new Pose2d(Bump.leftOuter.translation(), Rotation2d.kZero))
                            .build(),
                        // Approaching fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(-1.5, 0.0)),
                                    Rotation2d.fromDegrees(-45)))
                            .build(),
                        // At fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(-1.0, 0.0)),
                                    Rotation2d.fromDegrees(-45)))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpRightInnerToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting bump pose
                        PathWaypoint.from(
                                new Pose2d(Bump.rightInner.translation(), Rotation2d.kZero))
                            .build(),
                        // Just after bump
                        PathWaypoint.from(
                                new Pose2d(Bump.rightOuter.translation(), Rotation2d.kZero))
                            .build(),
                        // Approaching fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.rightCenter.plus(
                                        new Translation2d(-1.5, 0.0)),
                                    Rotation2d.fromDegrees(45)))
                            .build(),
                        // At fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.rightCenter.plus(
                                        new Translation2d(-1.0, 0.0)),
                                    Rotation2d.fromDegrees(45)))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    // Launch --> Neutral Zone
    paths.put(
        "launchLeftBumpToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(Launch.leftBump).build(),
                        // Just before bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.kZero))
                            .build(),
                        // Beginning of fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.kZero))
                            .build())
                    .build())
            .stopAtStart(false)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchRightBumpToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(Launch.rightBump).build(),
                        // Just before bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.kZero))
                            .build(),
                        // Beginning of fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.kZero))
                            .build())
                    .build())
            .stopAtStart(false)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftTowerToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(Launch.leftTower).build(),
                        // Just before bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.kZero))
                            .build(),
                        // Beginning of fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.kZero))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchRightTowerToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(Launch.rightTower).build(),
                        // Just before bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.kZero))
                            .build(),
                        // Beginning of fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.kZero))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    // Climb Trajectories
    paths.put(
        "launchRightTowerToClimbRight",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.rightTower).build(),
                        PathWaypoint.from(Climb.rightOffset).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Climb.right).build())
                    .keepInLaneWidth(0.008)
                    .maxVelocity(0.5)
                    .maxAngularVelocity(0.01)
                    .build())
            .build());

    paths.put(
        "launchLeftTowerToClimbLeft",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.leftTower).build(),
                        PathWaypoint.from(Climb.leftOffset).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Climb.left).build())
                    .keepInLaneWidth(0.008)
                    .maxVelocity(0.5)
                    .maxAngularVelocity(0.01)
                    .build())
            .build());

    // Home Depot Salesman trajectories
    PathRequestBuilder depotLeftToRight =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.leftThrough.translation(), Rotation2d.fromDegrees(-105)))
                            .build())
                    .keepInLaneWidth(0.3)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.rightThrough.translation(), Rotation2d.fromDegrees(-105)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .keepInLaneWidth(0.05)
                    .build());

    PathRequestBuilder trenchLeftStartToTower =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftStart.translation(), Rotation2d.fromDegrees(-90)))
                            .build())
                    .build())
            .segments(depotLeftToRight.build().segments);

    PathRequestBuilder bumpLeftInnerToTower =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(
                                new Pose2d(
                                    Bump.leftInner.translation(), Rotation2d.fromDegrees(-90)))
                            .build())
                    .build())
            .segments(depotLeftToRight.build().segments);

    PathRequestBuilder towerLeftThroughToOutpostLeftIntake =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Before tower
                        PathWaypoint.from(
                                new Pose2d(
                                    Tower.leftThrough.translation(), Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through tower
                        PathWaypoint.from(
                                new Pose2d(
                                    Tower.rightThrough.translation(), Rotation2d.fromDegrees(-90)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost
                        PathWaypoint.from(Outpost.leftIntake).build())
                    .build());

    PathRequestBuilder towerLeftThroughToOutpostLeftIntakeAround =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Before Tower Intermediate
                        PathWaypoint.from(new Translation2d(0.75, 5.0)).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outside tower left
                        PathWaypoint.from(Tower.leftOutside.translation()).build())
                    .keepInLaneWidth(0.15)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outside tower right
                        PathWaypoint.from(Tower.rightOutside.translation()).build())
                    .pointAt(hubTarget)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.Outpost.centerPoint.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.25, 0.0)),
                                    Rotation2d.kPi))
                            .build())
                    .build());

    paths.put("trenchLeftStartToTower", trenchLeftStartToTower.build());

    paths.put("bumpLeftInnerToTower", bumpLeftInnerToTower.build());

    paths.put(
        "trenchLeftStartToOutpostLeftIntake",
        trenchLeftStartToTower
            .build()
            .clone()
            .segments(towerLeftThroughToOutpostLeftIntake.build().segments)
            .build());

    paths.put(
        "trenchLeftStartToOutpostLeftIntakeAround",
        trenchLeftStartToTower
            .build()
            .clone()
            .segments(towerLeftThroughToOutpostLeftIntakeAround.build().segments)
            .build());

    paths.put(
        "bumpLeftInnerToOutpostLeftIntake",
        bumpLeftInnerToTower
            .build()
            .clone()
            .segments(towerLeftThroughToOutpostLeftIntake.build().segments)
            .build());

    paths.put(
        "bumpLeftInnerToOutpostLeftIntakeAround",
        bumpLeftInnerToTower
            .build()
            .clone()
            .segments(towerLeftThroughToOutpostLeftIntakeAround.build().segments)
            .build());

    // Lowe's Hardware Salesman trajectories
    PathRequestBuilder trenchRightStartToOutpostFrontIntake =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(
                                new Pose2d(Trench.rightStart.translation(), Rotation2d.kPi))
                            .build(),
                        // Outpost
                        PathWaypoint.from(Outpost.frontIntake).build())
                    .keepInLaneWidth(0.15)
                    .build());

    PathRequestBuilder bumpRightInnerToOutpostFrontIntake =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(new Pose2d(Bump.rightInner.translation(), Rotation2d.kPi))
                            .build(),
                        // Outpost
                        PathWaypoint.from(Outpost.frontIntake).build())
                    .keepInLaneWidth(0.15)
                    .build());

    PathRequestBuilder depotRightToLeft =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.rightThrough.translation(), Rotation2d.fromDegrees(105)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.leftThrough.translation(), Rotation2d.fromDegrees(105)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .keepInLaneWidth(0.05)
                    .build());

    paths.put("trenchRightStartToOutpostFrontIntake", trenchRightStartToOutpostFrontIntake.build());

    paths.put("bumpRightInnerToOutpostFrontIntake", bumpRightInnerToOutpostFrontIntake.build());

    paths.put(
        "outpostFrontIntakeToDepot",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost
                        PathWaypoint.from(Outpost.frontIntake).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outside tower right
                        PathWaypoint.from(
                                new Pose2d(
                                    Tower.rightThrough.translation(), Rotation2d.fromDegrees(90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through tower
                        PathWaypoint.from(
                                new Pose2d(
                                    Tower.leftThrough.translation(), Rotation2d.fromDegrees(90)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .build())
            .segments(depotRightToLeft.build().segments)
            .build());

    paths.put(
        "outpostFrontIntakeToDepotAround",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost
                        PathWaypoint.from(Outpost.frontIntake).build(),
                        // Outside tower right
                        PathWaypoint.from(Tower.rightOutside.translation()).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outside tower left
                        PathWaypoint.from(Tower.leftOutside.translation()).build())
                    .pointAt(hubTarget)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // After Tower Intermediate
                        PathWaypoint.from(new Translation2d(0.75, 5.0)).build())
                    .keepInLaneWidth(0.05)
                    .build())
            .segments(depotRightToLeft.build().segments)
            .build());

    // Substantial Salesman trajectories
    paths.put(
        "launchLeftBumpThroughDepotAndBack",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(Launch.leftBump).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch translation
                        PathWaypoint.from(Launch.leftBump.getTranslation()).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot offset translation
                        PathWaypoint.from(
                                new Translation2d(
                                    substantialAimUntilXDepot,
                                    FieldConstants.Depot.rightCorner.getY()
                                        - DriveConstants.fullWidthX / 2.0))
                            .build())
                    .pointAt(hubTarget)
                    .maxVelocity(0.6)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.rightThrough.translation(), Rotation2d.fromDegrees(105)))
                            .build())
                    .build())
            // Intake through depot
            .segments(depotRightToLeft.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot translation
                        PathWaypoint.from(Depot.leftThrough.translation()).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Curve around end of depot
                        PathWaypoint.from(
                                Depot.leftThrough.translation().plus(new Translation2d(0.2, 0.25)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Launch left bump translation
                        PathWaypoint.from(Launch.leftBump.getTranslation()).build())
                    .pointAt(hubTarget)
                    .maxVelocity(0.8)
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "substantial_launchRightBumpToOutpostFrontIntake",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(
                                new Pose2d(
                                    Launch.rightBump.getTranslation(),
                                    hubTarget
                                        .target()
                                        .minus(Launch.rightBump.getTranslation())
                                        .getAngle()
                                        .plus(Rotation2d.kPi)))
                            .build(),

                        // Starting launch translation
                        PathWaypoint.from(Launch.rightBump.getTranslation()).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost offset translation
                        PathWaypoint.from(
                                new Translation2d(
                                    substantialAimUntilXOutpost,
                                    Outpost.frontIntake.getTranslation().getY()))
                            .build())
                    .pointAt(hubTarget)
                    .maxVelocity(0.7)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost front intake pose
                        PathWaypoint.from(Outpost.frontIntake).build())
                    .build())
            .build());

    paths.put(
        "substantial_outpostFrontIntakeToLaunchRightBump",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost front intake pose
                        PathWaypoint.from(Outpost.frontIntake).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost front intake translation
                        PathWaypoint.from(Outpost.frontIntake.getTranslation()).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost leaving offset translation
                        PathWaypoint.from(
                                new Translation2d(
                                    substantialAimUntilXOutpost,
                                    Outpost.frontIntake.getTranslation().getY()))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Launch right bump translation
                        PathWaypoint.from(Launch.rightBump.getTranslation()).build())
                    .pointAt(hubTarget)
                    .maxVelocity(0.8)
                    .build())
            .stopAtEnd(false)
            .build());
  }
}
