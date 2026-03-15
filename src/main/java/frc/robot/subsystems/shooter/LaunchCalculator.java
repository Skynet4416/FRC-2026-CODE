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

public class LaunchCalculator {
  private static LaunchCalculator instance;

  private double hoodAngleOffsetDeg = 0.0;

  public double getHoodAngleOffsetDeg() {
    return hoodAngleOffsetDeg;
  }

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.4 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (1.5 / Constants.loopPeriodSecs));

  private double lastHoodAngle = Double.NaN;
  private Rotation2d lastDriveAngle;

  // --- ADDED SOTM SOLVER ---
  private final ShotCalculator shotCalc;

  private LaunchCalculator() {
    // Configure the SOTM solver with your robot's specific measurements
    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = robotToLauncher.getX(); // -0.229m
    config.launcherOffsetY = robotToLauncher.getY(); // 0.0m
    config.phaseDelayMs = phaseDelay * 1000.0; // 30ms latency compensation
    config.minScoringDistance = minDistance;
    config.maxScoringDistance = maxDistance;

    shotCalc = new ShotCalculator(config);

    // Load your calibration data into the solver
    shotCalc.loadLUTEntry(1.28, 2000.0, 1.0);
    shotCalc.loadLUTEntry(2.3, 2300.0, 0.96);
    shotCalc.loadLUTEntry(3.0, 2500.0, 0.94);
    shotCalc.loadLUTEntry(3.45, 2400.0, 1.04); // 18.0
    shotCalc.loadLUTEntry(4.0, 2700.0, 1.04);
    shotCalc.loadLUTEntry(5.0, 5500.0, 1.1);
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

  private static final double minDistance = 0.25;
  private static final double maxDistance = 5.7;
  private static final double passingMinDistance = 0.0;
  private static final double passingMaxDistance = 12.0;
  private static final double phaseDelay = 0.03;

  // Launching Maps (Only keeping Hood locally, RPM/TOF are now in the solver)
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  private static final LoggedTunableNumber maxIdleSpeed =
      new LoggedTunableNumber("LaunchCalculator/MaxIdleSpeed", 2000);

  private static final double xPassTarget = Units.inchesToMeters(37);
  private static final double yPassTarget = Units.inchesToMeters(65);

  // Boxes of bad
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));
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
    // Hood Map Init
    hoodAngleMap.put(1.28, Rotation2d.fromDegrees(5.0));
    hoodAngleMap.put(2.3, Rotation2d.fromDegrees(10.0));
    hoodAngleMap.put(3.0, Rotation2d.fromDegrees(15.0));
    hoodAngleMap.put(3.45, Rotation2d.fromDegrees(18.0));
    hoodAngleMap.put(4.0, Rotation2d.fromDegrees(20.0));
    hoodAngleMap.put(4.8, Rotation2d.fromDegrees(28.0));
    hoodAngleMap.put(5.0, Rotation2d.fromDegrees(30.0));

    // Passing Map Init
    passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(45.0));
    passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(45.0));
    passingHoodAngleMap.put(7.80, Rotation2d.fromDegrees(45.0));

    passingFlywheelSpeedMap.put(5.46, 5500.0);
    passingFlywheelSpeedMap.put(6.62, 5500.0);
    passingFlywheelSpeedMap.put(7.80, 5500.0);

    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);
  }

  public LaunchingParameters getParameters() {
    boolean passing =
        AllianceFlipUtil.applyX(Drive.getInstance().getPose().getX())
            > FieldConstants.LinesVertical.hubCenter;

    if (latestParameters != null) {
      return latestParameters;
    }

    Pose2d estimatedPose = Drive.getInstance().getPose();
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);

    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

    Rotation2d driveAngle;
    double hoodAngle;
    double flywheelVelocity;
    double timeOfFlight;
    double lookaheadLauncherToTargetDistance;
    double launcherToTargetDistance;
    boolean isValidShot;

    if (passing) {
      // --- LEGACY NAIVE LOOKAHEAD FOR PASSING ---
      ChassisSpeeds robotRelativeVelocity = Drive.getInstance().getChassisSpeeds();
      Pose2d lookaheadPose =
          estimatedPose.exp(
              new Twist2d(
                  robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                  robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                  robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

      Translation2d target = getPassingTarget();
      Pose2d launcherPosition = lookaheadPose.transformBy(GeomUtil.toTransform2d(robotToLauncher));
      launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

      var robotVelocity = Drive.getInstance().getFieldSetpointVelocity();
      var robotAngle = Drive.getInstance().getRotation();
      ChassisSpeeds launcherVelocity =
          GeomUtil.transformVelocity(
              robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);

      timeOfFlight = passingTimeOfFlightMap.get(launcherToTargetDistance);
      lookaheadLauncherToTargetDistance = launcherToTargetDistance;

      for (int i = 0; i < 20; i++) {
        timeOfFlight = passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance);
        double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
        double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
        lookaheadPose =
            new Pose2d(
                launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                launcherPosition.getRotation());
        lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
      }

      Pose2d lookaheadRobotPose =
          lookaheadPose.transformBy(GeomUtil.toTransform2d(robotToLauncher).inverse());
      driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);
      hoodAngle = passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();
      flywheelVelocity = passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance);
      isValidShot =
          outsideOfBadBoxes
              && lookaheadLauncherToTargetDistance >= passingMinDistance
              && lookaheadLauncherToTargetDistance <= passingMaxDistance;

    } else {
      // --- NEW PHYSICS-BASED SOTM SOLVER FOR HUB ---
      Translation2d target =
          AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

      // Dynamic forward vector to prevent the solver from invalidating shots from "behind" the hub.
      Translation2d hubForward = target.minus(estimatedPose.getTranslation());
      ShotCalculator.ShotInputs inputs =
          new ShotCalculator.ShotInputs(
              estimatedPose,
              Drive.getInstance().getFieldSetpointVelocity(),
              Drive.getInstance().getChassisSpeeds(),
              target,
              hubForward,
              1.0 // Vision confidence default
              );

      ShotCalculator.LaunchParameters result = shotCalc.calculate(inputs);

      // Extract results from solver
      driveAngle = result.driveAngle();
      lookaheadLauncherToTargetDistance = result.solvedDistanceM();
      launcherToTargetDistance = target.getDistance(estimatedPose.getTranslation());
      timeOfFlight = result.timeOfFlightSec();
      flywheelVelocity = result.rpm();

      // Query local map using solver's distance
      hoodAngle = hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();

      // Validate based on solver + bad boxes
      isValidShot = result.isValid() && outsideOfBadBoxes;
    }

    // --- APPLY FILTERS & CONSTRUCT RETURN RECORD ---
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;

    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    latestParameters =
        new LaunchingParameters(
            isValidShot,
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

    // Logging
    Logger.recordOutput(
        "LaunchCalculator/TargetPose",
        new Pose2d(
            passing
                ? getPassingTarget()
                : AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d()),
            Rotation2d.kZero));
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

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

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public Translation2d getPassingTarget() {
    double flippedY = AllianceFlipUtil.apply(Drive.getInstance().getPose()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    Translation2d flippedGoalTranslation =
        AllianceFlipUtil.apply(
            new Translation2d(
                xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));

    return flippedGoalTranslation;
  }

  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation,
        getDriveAngleWithLauncherOffset(GeomUtil.toPose2d(robotTranslation), target));
  }

  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }

  /** Returns the raw, uncompensated time of flight for a static shot at this distance. */
  public double getNaiveTOF(double distance) {
    // Queries the ShotCalculator's base LUT without SOTM compensation
    return shotCalc.getTimeOfFlight(distance);
  }

  public static double getMinTimeOfFlight() {
    return getInstance().shotCalc.getTimeOfFlight(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return getInstance().shotCalc.getTimeOfFlight(maxDistance);
  }
}
