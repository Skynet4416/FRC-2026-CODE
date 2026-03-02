// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystemIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystemIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystemIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystemIO;
import frc.robot.subsystems.shooter.hood.HoodSubsystemIOSim;
import frc.robot.subsystems.shooter.hood.HoodSubsystemIOTalonFX;
import frc.robot.subsystems.vision.*;
import frc.robot.util.ContinuousConditionalCommand;
import frc.robot.util.HubShiftUtil;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Vision vision;
  private final Drive drive;
  private final FlywheelSubsystem flywheelSubsystem;
  private final HoodSubsystem hoodSubsystem;
  // Controllers
  private final CommandXboxController driveController = new CommandXboxController(0);
  private SwerveDriveSimulation driveSimulation = null;

  private final CommandXboxController mechanismController = new CommandXboxController(1);
  private final Alert driverControllerDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert mechanismControllerDisconnected =
      new Alert("Mechanism controller disconnected (port 1).", AlertType.kWarning);

  private final Trigger disableFlywheelAutoSpinup = new Trigger(() -> false);
  private final Trigger ignoreHubState = new Trigger(() -> false);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                (robotPose) -> {});
        vision =
            new Vision(
                drive,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        flywheelSubsystem = new FlywheelSubsystem(new FlywheelSubsystemIOTalonFX());
        hoodSubsystem = new HoodSubsystem(new HoodSubsystemIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(), new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        flywheelSubsystem = new FlywheelSubsystem(new FlywheelSubsystemIOSim());
        hoodSubsystem = new HoodSubsystem(new HoodSubsystemIOSim());
        hoodSubsystem.zero();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

        flywheelSubsystem = new FlywheelSubsystem(new FlywheelSubsystemIO() {});
        hoodSubsystem = new HoodSubsystem(new HoodSubsystemIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Hood SysId (Quasistatic Forward)",
        hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Hood SysId (Quasistatic Reverse)",
        hoodSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Hood SysId (Dynamic Forward)",
        hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Hood SysId (Dynamic Reverse)",
        hoodSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        flywheelSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        flywheelSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)",
        flywheelSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)",
        flywheelSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Drive controls
    DoubleSupplier driverX = () -> -driveController.getLeftY();
    DoubleSupplier driverY = () -> -driveController.getLeftX();
    DoubleSupplier driverOmega = () -> -driveController.getRightX();

    // Default command, normal field-relative drive
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    // Lock to 0 when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> Rotation2d.kZero));

    Trigger hubActiveOrPassing =
        new Trigger(
            () ->
                HubShiftUtil.getShiftedShiftInfo().active()
                    || LaunchCalculator.getInstance().getParameters().passing());

    Trigger inLaunchingTolerance =
        new Trigger(
            () ->
                hoodSubsystem.atSetpoint()
                    && flywheelSubsystem.atSetpoint()
                    && DriveCommands.atLaunchGoal());

    // Align and auto-launch
    driveController
        .leftTrigger()
        .whileTrue(DriveCommands.joystickDriveWhileLaunching(drive, driverX, driverY))
        .whileTrue(flywheelSubsystem.runTrackTargetCommand())
        .and(() -> LaunchCalculator.getInstance().getParameters().isValid())
        .and(() -> ignoreHubState.getAsBoolean() || hubActiveOrPassing.getAsBoolean())
        .and(inLaunchingTolerance.debounce(0.25, DebounceType.kFalling))
        .whileTrue(
            Commands.repeatingSequence(
                Commands.waitSeconds(1), Commands.runOnce(this::launchSimulatedProjectile)));

    // Test specific button for simulated launch
    driveController.povUp().onTrue(Commands.runOnce(this::launchSimulatedProjectile));

    // TODO: run indexer when shooting

    // Switch to X pattern when X button is pressed
    driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetOdometry(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    flywheelSubsystem.setDefaultCommand(
        new ContinuousConditionalCommand(
            Commands.runOnce(flywheelSubsystem::stop, flywheelSubsystem),
            flywheelSubsystem.runAtSpeedRADSCommand(
                () -> LaunchCalculator.getInstance().getParameters().flywheelIdleSpeed()),
            disableFlywheelAutoSpinup));

    hoodSubsystem.setDefaultCommand(hoodSubsystem.runTrackTargetCommand());
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Controller disconnected alerts
    driverControllerDisconnected.set(
        !DriverStation.isJoystickConnected(driveController.getHID().getPort()));
    mechanismControllerDisconnected.set(
        !DriverStation.isJoystickConnected(mechanismController.getHID().getPort()));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void launchSimulatedProjectile() {
    if (Constants.currentMode != Constants.Mode.SIM || driveSimulation == null) {
      return;
    }

    var params = LaunchCalculator.getInstance().getParameters();
    if (params == null || !params.isValid()) {
      return;
    }

    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    double rpm = Units.radiansPerSecondToRotationsPerMinute(params.flywheelSpeed());

    // Field relative chassis speeds
    var chassisSpeeds = drive.getChassisSpeeds();
    var fieldRelativeSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
            chassisSpeeds, robotPose.getRotation());

    RebuiltFuelOnFly fuelOnFly =
        new RebuiltFuelOnFly(
            robotPose.getTranslation(),
            new edu.wpi.first.math.geometry.Translation2d(
                frc.robot.subsystems.shooter.LauncherConstants.robotToLauncher.getX(),
                frc.robot.subsystems.shooter.LauncherConstants.robotToLauncher.getY()),
            fieldRelativeSpeeds,
            robotPose.getRotation(), // Assuming launcher faces same direction as robot front
            Meters.of(frc.robot.subsystems.shooter.LauncherConstants.robotToLauncher.getZ()),
            MetersPerSecond.of(rpm / 6000.0 * 37.0),
            Radians.of(params.hoodAngle()));

    // Setup visualizer and callbacks
    Translation2d target2d =
        params.passing()
            ? LaunchCalculator.getInstance().getPassingTarget()
            : frc.robot.util.geometry.AllianceFlipUtil.apply(
                FieldConstants.Hub.topCenterPoint.toTranslation2d());

    fuelOnFly
        .withTargetPosition(
            () ->
                new edu.wpi.first.math.geometry.Translation3d(
                    target2d.getX(),
                    target2d.getY(),
                    params.passing()
                        ? 0.0
                        : FieldConstants.Hub.topCenterPoint.getZ())) // Based on FieldConstants
        .withTargetTolerance(new edu.wpi.first.math.geometry.Translation3d(0.5, 1.2, 0.3))
        .withProjectileTrajectoryDisplayCallBack(
            (pose3ds) ->
                Logger.recordOutput(
                    "Flywheel/FuelProjectileSuccessfulShot",
                    pose3ds.toArray(new edu.wpi.first.math.geometry.Pose3d[0])),
            (pose3ds) ->
                Logger.recordOutput(
                    "Flywheel/FuelProjectileUnsuccessfulShot",
                    pose3ds.toArray(new edu.wpi.first.math.geometry.Pose3d[0])))
        .enableBecomesGamePieceOnFieldAfterTouchGround();

    SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
