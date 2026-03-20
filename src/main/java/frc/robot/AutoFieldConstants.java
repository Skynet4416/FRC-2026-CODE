// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.LaunchCalculator;
import lombok.Builder;

public class AutoFieldConstants {
  public enum Area {
    DEPOT,
    TOWER,
    OUTPOST,
    START,
    NEUTRAL_ZONE,
    LAUNCH
  }

  @Builder
  public record Waypoint(Translation2d translation, Area area) {}

  public static class Trench {
    public static final Waypoint leftStart =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                    FieldConstants.LinesHorizontal.leftTrenchOpenEnd
                        + DriveConstants.fullWidthX / 2.0))
            .area(Area.START)
            .build();
    public static final Waypoint rightStart =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                    FieldConstants.LinesHorizontal.rightTrenchOpenStart
                        - DriveConstants.fullWidthX / 2.0))
            .area(Area.START)
            .build();
  }

  public static class Bump {
    public static final Waypoint leftInner =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                    FieldConstants.LinesHorizontal.leftBumpMiddle))
            .area(Area.START)
            .build();
    public static final Waypoint rightInner =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                    FieldConstants.LinesHorizontal.rightBumpMiddle))
            .area(Area.START)
            .build();
    public static final Waypoint leftOuter =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.LinesVertical.neutralZoneNear + DriveConstants.fullWidthX / 2.0,
                    FieldConstants.LinesHorizontal.leftBumpMiddle))
            .area(Area.NEUTRAL_ZONE)
            .build();
    public static final Waypoint rightOuter =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.LinesVertical.neutralZoneNear + DriveConstants.fullWidthX / 2.0,
                    FieldConstants.LinesHorizontal.rightBumpMiddle))
            .area(Area.NEUTRAL_ZONE)
            .build();
  }

  public static class Hub {
    public static final Waypoint centerStart =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullWidthX / 2.0,
                    FieldConstants.LinesHorizontal.center))
            .area(Area.START)
            .build();
  }

  public static class Depot {
    public static final Waypoint leftThrough =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.Depot.depth / 2.0 + 0.2,
                    FieldConstants.Depot.leftCorner.getY() + DriveConstants.fullWidthX / 2.0 + 0.1))
            .area(Area.DEPOT)
            .build();
    public static final Waypoint rightThrough =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.Depot.depth / 2.0 + 0.2,
                    FieldConstants.Depot.rightCorner.getY()
                        - DriveConstants.fullWidthX / 2.0
                        - 0.1))
            .area(Area.DEPOT)
            .build();
  }

  public static class Outpost {
    public static final Pose2d leftIntake =
        new Pose2d(
            FieldConstants.Outpost.centerPoint.plus(
                new Translation2d(DriveConstants.fullWidthX / 2 + 0.1, 0.25)),
            Rotation2d.fromDegrees(-90));

    public static final Pose2d frontIntake =
        new Pose2d(
            FieldConstants.Outpost.centerPoint.plus(
                new Translation2d(DriveConstants.fullWidthX / 2.0 + 0.25, 0.1)),
            Rotation2d.kPi);
  }

  public static class Tower {
    public static final Waypoint leftThrough =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.Tower.frontFaceX / 2.0,
                    FieldConstants.Tower.leftUpright.getY()
                        + DriveConstants.fullWidthX / 2.0
                        + 0.25))
            .area(Area.TOWER)
            .build();
    public static final Waypoint rightThrough =
        Waypoint.builder()
            .translation(
                new Translation2d(
                    FieldConstants.Tower.frontFaceX / 2.0,
                    FieldConstants.Tower.rightUpright.getY()
                        - DriveConstants.fullWidthX / 2.0
                        - 0.25))
            .area(Area.TOWER)
            .build();

    public static final Waypoint leftOutside =
        Waypoint.builder()
            .translation(
                FieldConstants.Tower.leftUpright.plus(
                    new Translation2d(
                        DriveConstants.fullWidthX / 2.0 + 0.3,
                        DriveConstants.fullWidthX / 2.0 + 0.3)))
            .area(Area.TOWER)
            .build();

    public static final Waypoint rightOutside =
        Waypoint.builder()
            .translation(
                FieldConstants.Tower.rightUpright.plus(
                    new Translation2d(
                        DriveConstants.fullWidthX / 2.0 + 0.3,
                        -DriveConstants.fullWidthX / 2.0 - 0.3)))
            .area(Area.TOWER)
            .build();
  }

  public static class Climb {
    public static final Pose2d right =
        new Pose2d(
            FieldConstants.Tower.rightUpright.plus(
                new Translation2d(0.0, -DriveConstants.fullWidthX / 2)),
            Rotation2d.kPi);

    public static final Pose2d rightOffset =
        new Pose2d(
            FieldConstants.Tower.rightUpright.plus(
                new Translation2d(0.0, -DriveConstants.fullWidthX / 2 - 0.6)),
            Rotation2d.kPi);

    public static final Pose2d left =
        new Pose2d(
            FieldConstants.Tower.leftUpright.plus(
                new Translation2d(0.0, +DriveConstants.fullWidthX / 2)),
            Rotation2d.kZero);

    public static final Pose2d leftOffset =
        new Pose2d(
            FieldConstants.Tower.leftUpright.plus(
                new Translation2d(0.0, +DriveConstants.fullWidthX / 2 + 0.6)),
            Rotation2d.kZero);
  }

  public static class Launch {
    public static final Pose2d leftTower =
        LaunchCalculator.getStationaryAimedPose(
            Climb.left.getTranslation().plus(new Translation2d(1.5, 0.5)), true);

    public static final Pose2d rightTower =
        LaunchCalculator.getStationaryAimedPose(
            Climb.right.getTranslation().plus(new Translation2d(1.5, -0.5)), true);

    public static Pose2d leftBump =
        LaunchCalculator.getStationaryAimedPose(
            new Translation2d(
                FieldConstants.LinesVertical.starting - 1,
                FieldConstants.LinesHorizontal.leftBumpMiddle),
            true);
    public static Pose2d rightBump =
        LaunchCalculator.getStationaryAimedPose(
            new Translation2d(
                FieldConstants.LinesVertical.starting - 1,
                FieldConstants.LinesHorizontal.rightBumpMiddle),
            true);
  }
}
