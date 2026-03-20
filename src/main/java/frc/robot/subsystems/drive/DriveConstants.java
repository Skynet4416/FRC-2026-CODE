// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class DriveConstants {
  // MARK: - Darwin Constants

  public static final double darwinTrackWidthXInches = 21.25;
  public static final double darwinTrackWidthYInches = 22.25;
  public static final double darwinFrameWidthXInches = 26.5;
  public static final double darwinFrameWidthYInches = 27.5;
  public static final double darwinFullWidthXInches = 33.0;
  public static final double darwinFullWidthYInches = 34.0;
  public static final double darwinMaxLinearSpeed = 4.2; // Theoretical max is 4.4196
  public static final double darwinMaxTrajectoryLinearSpeed = 4.0;
  public static final double darwinMaxAngularSpeed =
      5.374; // maxLinearSpeed / driveBaseRadius (0.7814886979 meters)
  public static final double darwinWheelRadiusInches = 1.99588001;
  public static final double darwinTrajectoryWheelRadiusInches = 2.0;
  public static final double darwinMaxTrajectoryWheelTorque = 3.0; // N * m
  public static final double darwinMassLbs = 150.0;
  public static final double darwinWheelCOF = 1.5;
  public static final double darwinRotationMOI = 6.0; // kg * m^2

  // SDS MK5i modules, R1 reduction
  public static final double darwinDriveReduction = 7.03125;
  public static final double darwinTurnReductionFL = 26.0;
  public static final double darwinTurnReductionFR = 26.0;
  public static final double darwinTurnReductionBL = 26.0;
  public static final double darwinTurnReductionBR = 26.0;
  // public static final double darwinTurnReductionBR = 26.09090909091; // MK5n

  public static final String darwinCanBus = "*";
  public static final int darwinGyroId = 30;
  public static final int darwinDriveMotorIdFL = 11;
  public static final int darwinDriveMotorIdFR = 13;
  public static final int darwinDriveMotorIdBL = 0;
  public static final int darwinDriveMotorIdBR = 23;

  public static final int darwinTurnMotorIdFL = 10;
  public static final int darwinTurnMotorIdFR = 12;
  public static final int darwinTurnMotorIdBL = 1;
  public static final int darwinTurnMotorIdBR = 22;

  public static final int darwinEncoderIdFL = 40;
  public static final int darwinEncoderIdFR = 41;
  public static final int darwinEncoderIdBL = 42;
  public static final int darwinEncoderIdBR = 43;

  public static final double darwinEncoderOffsetFL = -1.530913;
  public static final double darwinEncoderOffsetFR = 2.078544;
  public static final double darwinEncoderOffsetBL = -2.42369;
  public static final double darwinEncoderOffsetBR = -2.863942;

  // MARK: - Alpha Bot Constants

  public static final double alphabotTrackWidthXInches = 22.75;
  public static final double alphabotTrackWidthYInches = 22.75;
  public static final double alphabotFrameWidthXInches = 28.0;
  public static final double alphabotFrameWidthYInches = 28.0;
  public static final double alphabotFullWidthXInches = 34.5;
  public static final double alphabotFullWidthYInches = 34.5;
  public static final double alphabotMaxLinearSpeed = 4.69;
  public static final double alphabotMaxTrajectoryLinearSpeed = 4.0;
  public static final double alphabotMaxAngularSpeed = 6.46; // maxLinearSpeed / driveBaseRadius
  public static final double alphabotWheelRadiusInches = 1.8796;
  public static final double alphabotTrajectoryWheelRadiusInches = 2.0;
  public static final double alphabotMaxTrajectoryWheelTorque = 3.0; // N * m
  public static final double alphabotMassLbs = 140.0;
  public static final double alphabotWheelCOF = 1.5;
  public static final double alphabotRotationMOI = 6.0; // kg * m^2

  // SDS MK4i modules, L3 reduction
  public static final double alphabotDriveReduction = 6.1224489796;
  public static final double alphabotTurnReductionFL = 21.4285714286;
  public static final double alphabotTurnReductionFR = 21.4285714286;
  public static final double alphabotTurnReductionBL = 21.4285714286;
  public static final double alphabotTurnReductionBR = 21.4285714286;

  public static final String alphabotCanBus = "*";
  public static final int alphabotGyroId = 1;
  public static final int alphabotDriveMotorIdFL = 30;
  public static final int alphabotDriveMotorIdFR = 2;
  public static final int alphabotDriveMotorIdBL = 1;
  public static final int alphabotDriveMotorIdBR = 3;

  public static final int alphabotTurnMotorIdFL = 4;
  public static final int alphabotTurnMotorIdFR = 7;
  public static final int alphabotTurnMotorIdBL = 5;
  public static final int alphabotTurnMotorIdBR = 6;

  public static final int alphabotEncoderIdFL = 0;
  public static final int alphabotEncoderIdFR = 1;
  public static final int alphabotEncoderIdBL = 2;
  public static final int alphabotEncoderIdBR = 3;

  public static final double alphabotEncoderOffsetFL = 0.012886;
  public static final double alphabotEncoderOffsetFR = 0.866168;
  public static final double alphabotEncoderOffsetBL = -1.050078;
  public static final double alphabotEncoderOffsetBR = -2.801255;

  // MARK: - Shared Constants

  public static final double driveKs = 5.0;
  public static final double driveKv = 0.0;
  public static final double driveKp = 35.0;
  public static final double driveKd = 0.0;
  public static final double turnKp = 4000.0;
  public static final double turnKd = 50.0;
  public static final double turnDeadbandDegrees = 0.3;
  public static final double driveCurrentLimitAmps = 80;
  public static final double turnCurrentLimitAmps = 40;

  public static final double trackWidthX =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotTrackWidthXInches
              : darwinTrackWidthXInches);
  public static final double trackWidthY =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotTrackWidthYInches
              : darwinTrackWidthYInches);
  public static final double frameWidthX =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotFrameWidthXInches
              : darwinFrameWidthXInches);
  public static final double frameWidthY =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotFrameWidthYInches
              : darwinFrameWidthYInches);
  public static final double fullWidthX =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotFullWidthXInches
              : darwinFullWidthXInches);
  public static final double fullWidthY =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotFullWidthYInches
              : darwinFullWidthYInches);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed =
      Constants.robot == RobotType.ALPHABOT ? alphabotMaxLinearSpeed : darwinMaxLinearSpeed;
  public static final double maxTrajectoryLinearSpeed =
      Constants.robot == RobotType.ALPHABOT
          ? alphabotMaxTrajectoryLinearSpeed
          : darwinMaxTrajectoryLinearSpeed;
  public static final double maxAngularSpeed =
      Constants.robot == RobotType.ALPHABOT ? alphabotMaxAngularSpeed : darwinMaxAngularSpeed;
  public static final double wheelRadiusInches =
      Constants.robot == RobotType.ALPHABOT ? alphabotWheelRadiusInches : darwinWheelRadiusInches;
  public static final double trajectoryWheelRadiusInches =
      Constants.robot == RobotType.ALPHABOT
          ? alphabotTrajectoryWheelRadiusInches
          : darwinTrajectoryWheelRadiusInches;
  public static final double maxTrajectoryWheelTorque =
      Constants.robot == RobotType.ALPHABOT
          ? alphabotMaxTrajectoryWheelTorque
          : darwinMaxTrajectoryWheelTorque;
  public static final double wheelRadius = Units.inchesToMeters(wheelRadiusInches);
  public static final double trajectoryWheelRadius =
      Units.inchesToMeters(trajectoryWheelRadiusInches);
  public static final double mass =
      Units.lbsToKilograms(Constants.robot == RobotType.ALPHABOT ? alphabotMassLbs : darwinMassLbs);
  public static final double wheelCOF =
      Constants.robot == RobotType.ALPHABOT ? alphabotWheelCOF : darwinWheelCOF;
  public static final double rotationMOI =
      Constants.robot == RobotType.ALPHABOT ? alphabotRotationMOI : darwinRotationMOI;
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double driveReduction =
      Constants.robot == RobotType.ALPHABOT ? alphabotDriveReduction : darwinDriveReduction;
  public static final double turnReductionFL =
      Constants.robot == RobotType.ALPHABOT ? alphabotTurnReductionFL : darwinTurnReductionFL;
  public static final double turnReductionFR =
      Constants.robot == RobotType.ALPHABOT ? alphabotTurnReductionFR : darwinTurnReductionFR;
  public static final double turnReductionBL =
      Constants.robot == RobotType.ALPHABOT ? alphabotTurnReductionBL : darwinTurnReductionBL;
  public static final double turnReductionBR =
      Constants.robot == RobotType.ALPHABOT ? alphabotTurnReductionBR : darwinTurnReductionBR;

  public static final double intakeNearX = fullWidthX / 2.0;
  public static final double intakeFarX = frameWidthX / 2.0 + Units.inchesToMeters(12.0);
  public static final double intakeReferenceX = intakeNearX;
  public static final double intakeWidth = frameWidthY;
}
