// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoFactory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RunBothIndexersCommand;
import frc.robot.commands.TrajCommnd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemIO;
import frc.robot.subsystems.intake.IntakeSubsystemIOSim;
import frc.robot.subsystems.intake.IntakeSubsystemIOTalonFX;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystemIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystemIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystemIOTalonFX;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystemIO;
import frc.robot.subsystems.shooter.hood.HoodSubsystemIOSim;
import frc.robot.subsystems.shooter.hood.HoodSubsystemIOTalonFX;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerIO;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerIOSim;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerIOSparkMax;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystemIO;
import frc.robot.subsystems.spindexer.SpindexerSubsystemIOSim;
import frc.robot.subsystems.spindexer.SpindexerSubsystemIOTalonFX;
import frc.robot.subsystems.vision.*;
import frc.robot.util.ContinuousConditionalCommand;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.SuppliedWaitCommand;
import frc.robot.util.geometry.AllianceFlipUtil;
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
  private final IntakeSubsystem leftIntake;
  private final IntakeSubsystem rightIntake;
  private final Compressor compressor;
  private final AutoFactory autoFactory;

  private static final LoggedTunableNumber intakeSwitchDelay =
      new LoggedTunableNumber("IntakeSwitchDelay", 0.5);
  private static final LoggedTunableNumber confusionZoneMinAngle =
      new LoggedTunableNumber("ConfusionZoneMinAngle", 85.0);
  private static final LoggedTunableNumber confusionZoneMaxAngle =
      new LoggedTunableNumber("ConfusionZoneMaxAngle", 95.0);
  private final FlywheelSubsystem flywheelSubsystem;

  // Value to scale down the max omega during joystick driving, to make it easier for drivers to
  // control the robot's rotation. Should be a value between 0 and 1.
  private static final LoggedTunableNumber maxOmegaScalar =
      new LoggedTunableNumber("Drive/MaxOmegaScalar", 0.8);
  private final HoodSubsystem hoodSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final ShooterIndexerSubsystem shooterIndexerSubsystem;
  // Controllers
  private final CommandPS5Controller driveController = new CommandPS5Controller(0);
  private SwerveDriveSimulation driveSimulation = null;

  private final CommandPS5Controller mechanismController = new CommandPS5Controller(1);
  private final Alert driverControllerDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert mechanismControllerDisconnected =
      new Alert("Mechanism controller disconnected (port 1).", AlertType.kWarning);

  private final Trigger disableFlywheelAutoSpinup;
  private final Trigger ignoreHubState;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Boolean> runWheelsWhenFoldingChooser;
  private final LoggedDashboardChooser<Boolean> disableFlywheelAutoSpinupChooser;
  private final LoggedDashboardChooser<Boolean> ignoreHubStateChooser;

  // How much time in seconds to run the wheels when folding
  private static final LoggedTunableNumber intakeRunWheelsWhileFoldingDelay =
      new LoggedTunableNumber("IntakeRunWheelsWhileFoldingDelay", 1.0);

  // Triggers
  private final Trigger inConfusionZone;
  private final Trigger leftIntakeLowered;
  private final Trigger rightIntakeLowered;

  // Cached state for confusion zone stationary fallback
  private double lastKnownForwardBackwardJoystick = 0.0;

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

        spindexerSubsystem = new SpindexerSubsystem(new SpindexerSubsystemIOTalonFX());
        shooterIndexerSubsystem = new ShooterIndexerSubsystem(new ShooterIndexerIOSparkMax());
        leftIntake =
            new IntakeSubsystem(
                new IntakeSubsystemIOTalonFX(IntakeSubsystem.IntakeSide.LEFT),
                IntakeSubsystem.IntakeSide.LEFT);
        rightIntake =
            new IntakeSubsystem(
                new IntakeSubsystemIOTalonFX(IntakeSubsystem.IntakeSide.RIGHT),
                IntakeSubsystem.IntakeSide.RIGHT);

        compressor = new Compressor(4, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(80, 110);
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

        spindexerSubsystem = new SpindexerSubsystem(new SpindexerSubsystemIOSim());
        shooterIndexerSubsystem = new ShooterIndexerSubsystem(new ShooterIndexerIOSim());
        leftIntake =
            new IntakeSubsystem(
                new IntakeSubsystemIOSim(IntakeSubsystem.IntakeSide.LEFT),
                IntakeSubsystem.IntakeSide.LEFT);
        rightIntake =
            new IntakeSubsystem(
                new IntakeSubsystemIOSim(IntakeSubsystem.IntakeSide.RIGHT),
                IntakeSubsystem.IntakeSide.RIGHT);
        compressor = null;
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

        spindexerSubsystem = new SpindexerSubsystem(new SpindexerSubsystemIO() {});
        shooterIndexerSubsystem = new ShooterIndexerSubsystem(new ShooterIndexerIO() {});

        leftIntake =
            new IntakeSubsystem(new IntakeSubsystemIO() {}, IntakeSubsystem.IntakeSide.LEFT);
        rightIntake =
            new IntakeSubsystem(new IntakeSubsystemIO() {}, IntakeSubsystem.IntakeSide.RIGHT);
        compressor = null;
        break;
    }

    autoFactory =
        new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::resetOdometry, // A function that resets the current robot pose to the
            // provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
            );

    inConfusionZone =
        new Trigger(
            () -> {
              double absAngle =
                  Math.abs(AllianceFlipUtil.apply(drive.getPose().getRotation()).getDegrees());
              return absAngle > confusionZoneMinAngle.get()
                  && absAngle < confusionZoneMaxAngle.get();
            });

    leftIntakeLowered = new Trigger(leftIntake::isLowered);
    rightIntakeLowered = new Trigger(rightIntake::isLowered);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    runWheelsWhenFoldingChooser = new LoggedDashboardChooser<>("Run Wheels When Folding");
    runWheelsWhenFoldingChooser.addDefaultOption("Yes", true);
    runWheelsWhenFoldingChooser.addOption("No", false);

    disableFlywheelAutoSpinupChooser = new LoggedDashboardChooser<>("Disable Flywheel Auto Spinup");
    disableFlywheelAutoSpinupChooser.addDefaultOption("Yes", true);
    disableFlywheelAutoSpinupChooser.addOption("No", false);

    ignoreHubStateChooser = new LoggedDashboardChooser<>("Ignore Hub State");
    ignoreHubStateChooser.addOption("Yes", true);
    ignoreHubStateChooser.addDefaultOption("No", false);

    disableFlywheelAutoSpinup = new Trigger(disableFlywheelAutoSpinupChooser::get);
    ignoreHubState = new Trigger(ignoreHubStateChooser::get);

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
    autoChooser.addOption("Choreo Test", testAuto());
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
    DoubleSupplier driverOmega =
        () -> -driveController.getRightX(); // should be 4 for ds4, 3 for dualsense

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega, maxOmegaScalar::get));

    // Lock to 0 when A button is held
    driveController
        .cross()
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
                    // && flywheelSubsystem.atSetpoint()
                    && DriveCommands.atLaunchGoal());

    Trigger readyToShoot =
        new Trigger(() -> LaunchCalculator.getInstance().getParameters().isValid())
            .and(() -> ignoreHubState.getAsBoolean() || hubActiveOrPassing.getAsBoolean());
    // .and(inLaunchingTolerance.debounce(0.25, DebounceType.kFalling));

    driveController
        .R2()
        .whileTrue(DriveCommands.joystickDriveWhileLaunching(drive, driverX, driverY))
        .whileTrue(flywheelSubsystem.runTrackTargetCommand())
        .whileTrue(hoodSubsystem.runTrackTargetCommand());

    // driveController
    //     .cross()
    //     .whileTrue(new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem));

    driveController
        .L2()
        .and(readyToShoot)
        .whileTrue(
            Commands.parallel(
                new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem)));

    // Test specific button for simulated launch
    driveController.povUp().onTrue(Commands.runOnce(this::launchSimulatedProjectile));

    // Switch to X pattern when X button is pressed
    // driveController.square().onTrue(Commands.runOnce(drive::stopWithX, drive));
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    driveController.L1().onTrue(smartIntakeCommand(IntakeSubsystem.IntakeSide.LEFT));
    driveController.R1().onTrue(smartIntakeCommand(IntakeSubsystem.IntakeSide.RIGHT));

    SmartDashboard.putData("leftIntakeSet", smartIntakeCommand(IntakeSubsystem.IntakeSide.LEFT));
    SmartDashboard.putData("rightIntakeSet", smartIntakeCommand(IntakeSubsystem.IntakeSide.RIGHT));
    SmartDashboard.putData(
        "Run both Indexers",
        new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem));
    SmartDashboard.putData(
        "Stop both Indexers",
        Commands.runOnce(
            () -> {
              spindexerSubsystem.stop();
              shooterIndexerSubsystem.stop();
            },
            spindexerSubsystem,
            shooterIndexerSubsystem));

    // // Reset gyro to 0° when B button is pressed
    // driveController
    //     .circle()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.resetOdometry(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));

    flywheelSubsystem.setDefaultCommand(
        new ContinuousConditionalCommand(
            Commands.runOnce(flywheelSubsystem::stop, flywheelSubsystem),
            flywheelSubsystem.runAtSpeedRPMCommand(
                () -> LaunchCalculator.getInstance().getParameters().flywheelIdleSpeed()),
            disableFlywheelAutoSpinup));

    // flywheelSubsystem.setDefaultCommand(flywheelSubsystem.runFlywheelCommand());

    hoodSubsystem.setDefaultCommand(
        Commands.sequence(hoodSubsystem.zeroCommand(), hoodSubsystem.runTargetAngleCommand()));

    // --- Intake roller logic ---

    // Folded baseline: 0.5 when shooting (trigger held), 0 when idle
    leftIntake.setDefaultCommand(
        Commands.run(
            () -> {
              if (leftIntake.isLowered()) {
                leftIntake.setPercentage(1.0);
              } else {
                leftIntake.setPercentage(driveController.R2().getAsBoolean() ? 0.5 : 0.0);
              }
            },
            leftIntake));
    rightIntake.setDefaultCommand(
        Commands.run(
            () -> {
              if (rightIntake.isLowered()) {
                rightIntake.setPercentage(1.0);
              } else {
                rightIntake.setPercentage(driveController.R2().getAsBoolean() ? 0.2 : 0.0);
              }
            },
            rightIntake));

    // When folding/unfolding
    leftIntakeLowered
        .onTrue(Commands.runOnce(() -> leftIntake.setPercentage(1.0), leftIntake))
        .onFalse(
            Commands.run(() -> leftIntake.setPercentage(1.0), leftIntake)
                .withTimeout(intakeRunWheelsWhileFoldingDelay.get())
                .onlyIf(() -> runWheelsWhenFoldingChooser.get()));
    rightIntakeLowered
        .onTrue(Commands.runOnce(() -> rightIntake.setPercentage(1.0), rightIntake))
        .onFalse(
            Commands.run(() -> rightIntake.setPercentage(1.0), rightIntake)
                .withTimeout(intakeRunWheelsWhileFoldingDelay.get())
                .onlyIf(() -> runWheelsWhenFoldingChooser.get()));
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

    double currentVY = -driveController.getLeftY();
    if (Math.abs(currentVY) >= 0.05) {
      lastKnownForwardBackwardJoystick = currentVY;
    }

    // Log the Intake Confusion Zone Trigger
    Logger.recordOutput("In Intake Direction Confusion Zone", inConfusionZone);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Command smartIntakeCommand(IntakeSubsystem.IntakeSide bumperSide) {
    return Commands.defer(
            () -> {
              IntakeSubsystem.IntakeSide desired = getDesiredIntakeSide(bumperSide);
              IntakeSubsystem target =
                  desired == IntakeSubsystem.IntakeSide.LEFT ? leftIntake : rightIntake;
              IntakeSubsystem other =
                  desired == IntakeSubsystem.IntakeSide.LEFT ? rightIntake : leftIntake;

              if (target.isLowered()) {
                // Target already open → toggle it closed
                return Commands.runOnce(() -> target.setLowered(false));
              } else if (other.isLowered()) {
                // Other is open → close it, wait, then open target
                // We handle rollers directly here because kCancelIncoming blocks the
                // onFalse/onTrue triggers from firing while this command holds the subsystems
                return Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          other.setLowered(false);
                        }),
                    new SuppliedWaitCommand(() -> intakeSwitchDelay.get()),
                    Commands.runOnce(
                        () -> {
                          target.setLowered(true);
                        }));
              } else {
                // Neither open → just open target
                return Commands.runOnce(() -> target.setLowered(true));
              }
            },
            java.util.Set.of(leftIntake, rightIntake))
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  private IntakeSubsystem.IntakeSide getDesiredIntakeSide(IntakeSubsystem.IntakeSide bumperSide) {
    Rotation2d rotation = AllianceFlipUtil.apply(drive.getPose().getRotation());
    if (!inConfusionZone.getAsBoolean()) {
      // OUT OF CONFUSION ZONE -> Use bumper field-relative logic
      boolean facingBackwards = Math.abs(rotation.getDegrees()) > 90.0;
      boolean isLeftBumper = bumperSide == IntakeSubsystem.IntakeSide.LEFT;
      boolean wantsLeft = isLeftBumper ? !facingBackwards : facingBackwards;
      return !wantsLeft ? IntakeSubsystem.IntakeSide.LEFT : IntakeSubsystem.IntakeSide.RIGHT;
    }

    // IN CONFUSION ZONE -> Use velocity vector (bumper choice doesn't matter)
    // Positive is pushing the joystick forward (away from driver)
    double vX = -driveController.getLeftY();

    // Are we facing left (+90 degrees)?
    boolean facingLeft = rotation.getDegrees() > 0;

    // If stationary (no forward/backward input), fallback to the "Last Known Velocity"
    if (Math.abs(vX) < 0.05) {
      vX = lastKnownForwardBackwardJoystick;
    }

    if (vX > 0) {
      // Going Forward (Away from driver)
      // If facing left (+90), right intake is away. If facing right (-90), left intake is
      // away.
      return !facingLeft ? IntakeSubsystem.IntakeSide.LEFT : IntakeSubsystem.IntakeSide.RIGHT;
    } else {
      // Going Backward (Closer to driver)
      // If facing left (+90), left intake is closer. If facing right (-90), right intake is
      // closer.
      return facingLeft ? IntakeSubsystem.IntakeSide.LEFT : IntakeSubsystem.IntakeSide.RIGHT;
    }
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

  public Command testAuto() {
    return Commands.sequence(
        autoFactory.resetOdometry("left_trench"), //
        new TrajCommnd(autoFactory, "left_trench", drive));
  }
}
