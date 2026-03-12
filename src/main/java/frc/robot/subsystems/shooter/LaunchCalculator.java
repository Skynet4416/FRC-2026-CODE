// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.LauncherConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.geometry.AllianceFlipUtil;
import frc.robot.util.geometry.Bounds;
import frc.robot.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class LaunchCalculator {
  private static LaunchCalculator instance;

  private double hoodAngleOffsetDeg = 0.0;
  // FUDGE FACTOR: Tune this down (e.g., 0.5 to 0.8) to stop over-compensating.
  private static final LoggedTunableNumber lookaheadScalar =
      new LoggedTunableNumber("LaunchCalculator/LookaheadScalar", 1.0);

  private final LoggedDashboardChooser<Boolean> forceCurrentSpeedsChooser;

  public double getHoodAngleOffsetDeg() {
    return hoodAngleOffsetDeg;
  }

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.4 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (1.5 / Constants.loopPeriodSecs));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  private LaunchCalculator() {
    forceCurrentSpeedsChooser = new LoggedDashboardChooser<>("Force Current Speeds");
    forceCurrentSpeedsChooser.addDefaultOption("No", false);
    forceCurrentSpeedsChooser.addOption("Yes", true);
  }

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double flywheelIdleSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Presets
  //   public static final double hubPresetDistance = 0.96;
  //   public static final double towerPresetDistance = 2.5;
  //   public static final double trenchPresetDistance = 3.03;
  //   public static final double outpostPresetDistance = 4.84;

  //   public static final LaunchPreset hubPreset;
  //   public static final LaunchPreset towerPreset;
  //   public static final LaunchPreset trenchPreset;
  //   public static final LaunchPreset outpostPreset;
  //   public static final LaunchPreset hoodMinPreset =
  //       new LaunchPreset(
  //           new LoggedTunableNumber(
  //               "LaunchCalculator/Presets/HoodMin/HoodAngle",
  //               Constants.Subsystems.Shooter.Hood.MIN_ANGLE_DEG),
  //           new LoggedTunableNumber("LaunchCalculator/Presets/HoodMin/FlywheelSpeed", 50));
  //   public static final LaunchPreset hoodMaxPreset =
  //       new LaunchPreset(
  //           new LoggedTunableNumber(
  //               "LaunchCalculator/Presets/HoodMax/HoodAngle",
  //               Constants.Subsystems.Shooter.Hood.MAX_ANGLE_DEG),
  //           new LoggedTunableNumber("LaunchCalculator/Presets/HoodMax/FlywheelSpeed", 50));

  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  private static final LoggedTunableNumber maxIdleSpeed =
      new LoggedTunableNumber("LaunchCalculator/MaxIdleSpeed", 2000);

  private static final double xPassTarget = Units.inchesToMeters(37);
  private static final double yPassTarget = Units.inchesToMeters(65);
  // Boxes of bad
  // Under tower
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  // Behind the hubs
  private static final Bounds nearHubBound =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(120),
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);
  private static final Bounds farHubBound =
      new Bounds(
          FieldConstants.LinesVertical.oppAllianceZone,
          FieldConstants.fieldLength,
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);

  static {
    minDistance = 0.9;
    maxDistance = 4.9;
    passingMinDistance = 0.0;
    passingMaxDistance = 12.0;
    phaseDelay = 0.03;

    hoodAngleMap.put(1.3, Rotation2d.fromDegrees(0.0));
    hoodAngleMap.put(1.7, Rotation2d.fromDegrees(5.0));
    hoodAngleMap.put(2.2, Rotation2d.fromDegrees(10.5));
    hoodAngleMap.put(2.7, Rotation2d.fromDegrees(13.0));
    hoodAngleMap.put(3.2, Rotation2d.fromDegrees(15.0));
    hoodAngleMap.put(3.7, Rotation2d.fromDegrees(18.0));
    hoodAngleMap.put(4.2, Rotation2d.fromDegrees(20.0));
    hoodAngleMap.put(4.7, Rotation2d.fromDegrees(22.0));
    hoodAngleMap.put(5.2, Rotation2d.fromDegrees(22.0));
    hoodAngleMap.put(5.7, Rotation2d.fromDegrees(23.0));

    flywheelSpeedMap.put(1.3, 3000.0);
    flywheelSpeedMap.put(1.7, 3100.0);
    flywheelSpeedMap.put(2.2, 3300.0);
    flywheelSpeedMap.put(2.7, 3500.0);
    flywheelSpeedMap.put(3.2, 3500.0);
    flywheelSpeedMap.put(3.7, 3700.0);
    flywheelSpeedMap.put(4.2, 3900.0);
    flywheelSpeedMap.put(4.7, 4000.0);
    flywheelSpeedMap.put(5.2, 5200.0);
    flywheelSpeedMap.put(5.7, 5500.0);

    timeOfFlightMap.put(1.3, 1.0);
    timeOfFlightMap.put(1.7, 1.0);
    timeOfFlightMap.put(2.2, 1.1);
    timeOfFlightMap.put(2.7, 1.17);
    timeOfFlightMap.put(3.2, 1.08);
    timeOfFlightMap.put(3.7, 1.07);
    timeOfFlightMap.put(4.2, 1.17);
    timeOfFlightMap.put(4.7, 1.12);
    timeOfFlightMap.put(5.2, 1.23);
    timeOfFlightMap.put(5.7, 1.25);

    passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(7.80, Rotation2d.fromDegrees(38.0));

    passingFlywheelSpeedMap.put(5.46, 5500.0);
    passingFlywheelSpeedMap.put(6.62, 5500.0);
    passingFlywheelSpeedMap.put(7.80, 5500.0);

    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);

    // hubPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Hub/HoodAngle",
    //             hoodAngleMap.get(hubPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Hub/FlywheelSpeed",
    //             flywheelSpeedMap.get(hubPresetDistance)));
    // towerPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Tower/HoodAngle",
    //             hoodAngleMap.get(towerPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Tower/FlywheelSpeed",
    //             flywheelSpeedMap.get(towerPresetDistance)));
    // trenchPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Trench/HoodAngle",
    //             hoodAngleMap.get(trenchPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Trench/FlywheelSpeed",
    //             flywheelSpeedMap.get(trenchPresetDistance)));
    // outpostPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Outpost/HoodAngle",
    //             hoodAngleMap.get(outpostPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Outpost/FlywheelSpeed",
    //             flywheelSpeedMap.get(outpostPresetDistance)));
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters() {
    boolean passing =
        AllianceFlipUtil.applyX(Drive.getInstance().getPose().getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = Drive.getInstance().getPose();
    ChassisSpeeds robotRelativeVelocity = Drive.getInstance().getChassisSpeeds();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target =
        passing
            ? getPassingTarget()
            : AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d launcherPosition = estimatedPose.transformBy(GeomUtil.toTransform2d(robotToLauncher));
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    var robotVelocity =
        Boolean.TRUE.equals(forceCurrentSpeedsChooser.get())
            ? Drive.getInstance().getFieldVelocity()
            : Drive.getInstance().getFieldSetpointVelocity();
    var robotAngle = Drive.getInstance().getRotation();
    ChassisSpeeds launcherVelocity =
        GeomUtil.transformVelocity(
            robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);

      // --- APPLY THE FUDGE FACTOR HERE ---
      double scaledToF = timeOfFlight * lookaheadScalar.get();

      double offsetX = launcherVelocity.vxMetersPerSecond * scaledToF;
      double offsetY = launcherVelocity.vyMetersPerSecond * scaledToF;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(GeomUtil.toTransform2d(robotToLauncher).inverse());
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians()
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    // Check if inside a box of bad
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);
    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

    double flywheelVelocity =
        passing
            ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
            : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && lookaheadLauncherToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadLauncherToTargetDistance
                    <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngle + Units.degreesToRadians(hoodAngleOffsetDeg),
            hoodVelocity,
            flywheelVelocity,
            MathUtil.clamp(flywheelVelocity, 0, maxIdleSpeed.get()),
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);
    Logger.recordOutput(
        "LaunchCalculator/RobotToLauncher",
        new edu.wpi.first.math.geometry.Pose3d(Drive.getInstance().getPose())
            .transformBy(robotToLauncher));

    return latestParameters;
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(
      Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToLauncher.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle.minus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public Translation2d getPassingTarget() {
    double flippedY = AllianceFlipUtil.apply(Drive.getInstance().getPose()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    // Fixed passing target
    Translation2d flippedGoalTranslation =
        AllianceFlipUtil.apply(
            new Translation2d(
                xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));

    return flippedGoalTranslation;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    // Calculate target
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation,
        getDriveAngleWithLauncherOffset(GeomUtil.toPose2d(robotTranslation), target));
  }

  /** Adjusts the hood angle offset up or down the specified amount. */
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }
}
