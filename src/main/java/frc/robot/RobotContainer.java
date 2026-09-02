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
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RunBothIndexersCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AIControlBridge;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemIO;
import frc.robot.subsystems.intake.IntakeSubsystemIOSim;
import frc.robot.subsystems.intake.IntakeSubsystemIOTalonFX;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.ledSubsystemIOCandle;
import frc.robot.subsystems.shooter.FuelPhysicsSim;
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
import frc.robot.subsystems.vision.GamePieceVisionSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.ContinuousConditionalCommand;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.elasticlib.Elastic;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
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
  private final AIControlBridge aiControlBridge;
  private GamePieceVisionSim gamePieceVisionSim = null;

  // True while the AI bridge owns the drivetrain; driver-assist bindings stand down.
  private Trigger aiControlActive;
  private final Drive drive;
  private final IntakeSubsystem leftIntake;
  private final LedSubsystem ledSubsystem;
  private final Compressor compressor;
  private final AutoFactory autoFactory;

  private final FlywheelSubsystem flywheelSubsystem;

  // Value to scale down the max omega during joystick driving, to make it easier for drivers to
  // control the robot's rotation. Should be a value between 0 and 1.
  private static final LoggedTunableNumber maxOmegaScalar =
      new LoggedTunableNumber("Drive/MaxOmegaScalar", 0.8);

  // Loose heading tolerance (degrees) for passing shots. Wide enough not to fight the driver while
  // being guarded, tight enough to keep passes inside the field boundaries.
  private static final LoggedTunableNumber passingHeadingToleranceDeg =
      new LoggedTunableNumber("LaunchCalculator/PassingHeadingToleranceDeg", 35.0);
  private final HoodSubsystem hoodSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final ShooterIndexerSubsystem shooterIndexerSubsystem;
  private final FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");

  // Where the AI shoots from: on our side of the hub, inside the alliance zone. Past the hub the
  // launch calculator switches to a lob pass, which does not score.
  private static final double shootingDistanceMeters = 2.6;
  private static final double shootingZoneMarginMeters = 0.5;
  private static final double strafeHalfWidth = 1.3;
  private static final double strafeInputScalar = 0.45;

  /** Point the shoot-on-the-move action is translating toward, picked when it starts. */
  private Translation2d shootingStrafeTarget = null;

  // Controllers
  // Lets the driver use either an Xbox or a PS5 controller, selected live from a dashboard chooser.
  private final LoggedDashboardChooser<DriverController.Type> driverControllerTypeChooser =
      new LoggedDashboardChooser<>("Driver Controller Type");
  private final DriverController driveController =
      new DriverController(
          0,
          () -> {
            // Null-safe: chooser can return null before NetworkTables delivers the default.
            DriverController.Type type = driverControllerTypeChooser.get();
            return type == null ? DriverController.Type.XBOX : type;
          });
  private SwerveDriveSimulation driveSimulation = null;
  private IntakeSubsystemIOSim intakeSimIO = null;
  // Fuel loaded at match start in sim. ponytail: guess, set to the real preload count
  private static final int SIM_FUEL_PRELOAD = 8;
  private frc.robot.util.RobotBumpSim robotBumpSim = null;
  private boolean wasOnRamp = false;
  private final edu.wpi.first.wpilibj.Timer teleopElapsedTimer = new edu.wpi.first.wpilibj.Timer();
  //   private final CommandPS5Controller mechanismController = new CommandPS5Controller(1);
  private final Alert driverControllerDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.kWarning);
  private final Alert autoWinnerNotSet = new Alert("!!! AUTO WINNER NOT SET !!!", AlertType.kError);
  //   private final Alert mechanismControllerDisconnected =
  //       new Alert("Mechanism controller disconnected (port 1).", AlertType.kWarning);

  // For elastic
  private final Field2d field = new Field2d();

  private final Trigger disableFlywheelAutoSpinup;
  private final Trigger reverseIndexWhileIntake;
  private final Trigger ignoreHubState;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Boolean> runWheelsWhenFoldingChooser;
  private final LoggedDashboardChooser<Boolean> reverseIndexWhileIntakeChooser;
  private final LoggedDashboardChooser<Boolean> disableFlywheelAutoSpinupChooser;
  private final LoggedDashboardChooser<Boolean> ignoreHubStateChooser;
  // Toggles the loose heading cone gate that keeps passing shots inside the field
  private final LoggedDashboardChooser<Boolean> enablePassingConeChooser;
  private final LoggedDashboardChooser<String> allianceWinOverrideChooser;
  private final LoggedDashboardChooser<DriveCommands.TrenchAlignmentPosition>
      trenchAlignmentPositionChooser;

  // How much time in seconds to run the wheels when folding
  private static final LoggedTunableNumber intakeRunWheelsWhileFoldingDelay =
      new LoggedTunableNumber("IntakeRunWheelsWhileFoldingDelay", 1.0);
  private static final LoggedTunableNumber trenchExtension =
      new LoggedTunableNumber("TrenchExtension", 0.5);

  // Triggers
  private final Trigger leftIntakeLowered;
  private Trigger intakeStruggling;
  private final Trigger autoAlignmentOverride;

  // Shooting triggers, built in configureShootingTriggers()
  private Trigger hubActiveOrPassing;
  private Trigger inPassingTolerance;
  private Trigger inLaunchingTolerance;
  private Trigger readyToShoot;

  private boolean autoAlignmentOverrideState = true;

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
                new VisionIOLimelight(
                    VisionConstants.camera0Name, drive::getRotation, new Transform3d()), // r
                new VisionIOLimelight(
                    VisionConstants.camera1Name,
                    drive::getRotation,
                    new Transform3d(
                        0, 0.0, 0, new Rotation3d(0, 0, Units.degreesToRadians(0))))); // l

        flywheelSubsystem = new FlywheelSubsystem(new FlywheelSubsystemIOTalonFX());
        hoodSubsystem = new HoodSubsystem(new HoodSubsystemIOTalonFX());

        spindexerSubsystem = new SpindexerSubsystem(new SpindexerSubsystemIOTalonFX());
        shooterIndexerSubsystem = new ShooterIndexerSubsystem(new ShooterIndexerIOSparkMax());
        leftIntake = new IntakeSubsystem(new IntakeSubsystemIOTalonFX());

        compressor = new Compressor(4, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(80, 105);
        SmartDashboard.putData("Field", field);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // Disable MapleSim's built-in ramp colliders (bumps as solid walls) so
        // RobotBumpSim can own the bump-crossing physics instead.
        SimulatedArena.overrideInstance(
            new org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt(false));
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.getMapleSimConfig(),
                new Pose2d(
                    AllianceFlipUtil.applyX(3.591),
                    AllianceFlipUtil.applyY(7.430),
                    AllianceFlipUtil.apply(Rotation2d.fromDegrees(-90))));

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        robotBumpSim = new frc.robot.util.RobotBumpSim(Drive.getModuleTranslations());
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        // Simulated PhotonVision cameras. These feed the pose estimator and publish live camera
        // streams, which is what lets an external AI operator see the field through the robot.
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

        // Object detection camera for the fuel, fed from MapleSim's field pieces and the shots
        // in flight in the fuel physics sim.
        gamePieceVisionSim =
            new GamePieceVisionSim(
                driveSimulation::getSimulatedDriveTrainPose, this::simulatedGamePieces);

        flywheelSubsystem = new FlywheelSubsystem(new FlywheelSubsystemIOSim());
        hoodSubsystem = new HoodSubsystem(new HoodSubsystemIOSim());
        hoodSubsystem.zero();

        spindexerSubsystem = new SpindexerSubsystem(new SpindexerSubsystemIOSim());
        shooterIndexerSubsystem = new ShooterIndexerSubsystem(new ShooterIndexerIOSim());
        intakeSimIO = new IntakeSubsystemIOSim();
        leftIntake = new IntakeSubsystem(intakeSimIO);
        compressor = null;

        ballSim.enable();
        ballSim.placeFieldBalls();
        // Intake pickup box from CAD (robot frame, m): LEFT (+Y) side, bumper face to 0.61
        // out, full 0.708m span. Maple-sim style "touch it, get it": active whenever the
        // maple-sim intake is extended (lowered + rollers on) and the hopper has room.
        ballSim.addIntakeZone(
            -0.354,
            0.354,
            0.35,
            0.61,
            () -> intakeSimIO.isIntakeRunning() && intakeSimIO.canHoldMore(),
            intakeSimIO::addGamePieceToIntake);
        intakeSimIO.setHeldCount(SIM_FUEL_PRELOAD);
        SmartDashboard.putData("Field", field);
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

        leftIntake = new IntakeSubsystem(new IntakeSubsystemIO() {});
        compressor = null;
        break;
    }

    ledSubsystem =
        new LedSubsystem(
            new ledSubsystemIOCandle(),
            shooterIndexerSubsystem,
            flywheelSubsystem,
            spindexerSubsystem,
            leftIntake);

    autoFactory =
        new AutoFactory(
                drive::getPose, // A function that returns the current robot pose
                drive::resetOdometry, // A function that resets the current robot pose to the
                // provided Pose2d
                drive::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                drive // The drive subsystem
                )
            .bind("IntakeOpen", Commands.runOnce(() -> leftIntake.setLowered(true), leftIntake));
    // CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());

    leftIntakeLowered = new Trigger(leftIntake::isLowered);
    autoAlignmentOverride = new Trigger(() -> autoAlignmentOverrideState);

    // Driver controller type (Xbox or PS5). Default Xbox; switchable live from the dashboard.
    driverControllerTypeChooser.addDefaultOption("Xbox Controller", DriverController.Type.XBOX);
    driverControllerTypeChooser.addOption("PS5 Controller", DriverController.Type.PS5);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    runWheelsWhenFoldingChooser = new LoggedDashboardChooser<>("Run Wheels When Folding");
    runWheelsWhenFoldingChooser.addDefaultOption("Yes", true);
    runWheelsWhenFoldingChooser.addOption("No", false);

    reverseIndexWhileIntakeChooser = new LoggedDashboardChooser<>("Reverse Index While Intake");
    reverseIndexWhileIntakeChooser.addDefaultOption("Yes", true);
    reverseIndexWhileIntakeChooser.addOption("No", false);

    disableFlywheelAutoSpinupChooser = new LoggedDashboardChooser<>("Disable Flywheel Auto Spinup");
    disableFlywheelAutoSpinupChooser.addOption("Yes", true);
    disableFlywheelAutoSpinupChooser.addDefaultOption("No", false);

    ignoreHubStateChooser = new LoggedDashboardChooser<>("Ignore Hub State");
    ignoreHubStateChooser.addDefaultOption("Yes", true);
    ignoreHubStateChooser.addOption("No", false);

    // Enable the loose heading cone that keeps passing shots inside the field boundaries
    enablePassingConeChooser = new LoggedDashboardChooser<>("Enable Passing Cone");
    enablePassingConeChooser.addDefaultOption("Yes", true);
    enablePassingConeChooser.addOption("No", false);

    allianceWinOverrideChooser = new LoggedDashboardChooser<>("Alliance Win Override");
    allianceWinOverrideChooser.addDefaultOption("None", "None");
    allianceWinOverrideChooser.addOption("Won", "Won");
    allianceWinOverrideChooser.addOption("Lost", "Lost");

    // Null-safe: LoggedDashboardChooser.get() can return null before NetworkTables delivers the
    // default selection (the first loops after enable, when autos run). Unboxing a null Boolean
    // here throws an NPE during scheduler trigger-polling, which kills the running auto command.
    // Map null to each chooser's configured default (Yes=true / No=false).
    disableFlywheelAutoSpinup =
        new Trigger(
            () -> !Boolean.FALSE.equals(disableFlywheelAutoSpinupChooser.get())); // default Yes
    reverseIndexWhileIntake =
        new Trigger(
            () -> !Boolean.FALSE.equals(reverseIndexWhileIntakeChooser.get())); // default Yes
    ignoreHubState =
        new Trigger(() -> Boolean.TRUE.equals(ignoreHubStateChooser.get())); // default No

    HubShiftUtil.setAllianceWinOverride(
        () -> {
          String dashValue = allianceWinOverrideChooser.get();
          if (dashValue != null) {
            if (dashValue.equals("Won")) return Optional.of(true);
            if (dashValue.equals("Lost")) return Optional.of(false);
          }
          return Optional.empty();
        });

    configureShootingTriggers();

    this.intakeStruggling = new Trigger(leftIntake.isStrugglingSupplier());

    trenchAlignmentPositionChooser = new LoggedDashboardChooser<>("Trench Alignment Position");
    trenchAlignmentPositionChooser.addDefaultOption(
        "Middle", DriveCommands.TrenchAlignmentPosition.MIDDLE);
    trenchAlignmentPositionChooser.addOption("Inner", DriveCommands.TrenchAlignmentPosition.INNER);
    trenchAlignmentPositionChooser.addOption("Outer", DriveCommands.TrenchAlignmentPosition.OUTER);

    autoChooser.addDefaultOption(
        "Shoot",
        Commands.sequence(
            Commands.runOnce(() -> hoodSubsystem.zero(), hoodSubsystem), autoShoot(10.0)));
    autoChooser.addOption("Choreo Test", testAuto());
    autoChooser.addOption("Left Trench Double Take", leftTrenchDoubleTake());
    autoChooser.addOption("Left Trench Return Over Bump", leftTrenchIntakeReturnOverBump());
    autoChooser.addOption("Right Trench Return Over Bump", rightTrenchIntakeReturnOverBump());
    autoChooser.addOption("Behind Hub Intake", leftTrenchHubIntakeReturnOverBump());
    autoChooser.addOption("left Trench Single Take", leftTrenchSingleTake());
    autoChooser.addOption("Depot", Deot());

    // Configure the button bindings

    autoChooser.onChange(
        (listener -> {
          // Pose2d[] poses = Choreo.loadTrajectory(listener.getName()).get().getPoses();
          //     double[] arr p= new double[poses.length * 3];
          //     int ndx = 0;
          //     for (PosPe2d pose : poses) {
          //       Translation2d translation = AllianceFlipUtil.apply(pose.getTranslation());
          //       arr[ndx + 0] = translation.getX();
          //       arr[ndx + 1] = translation.getY();
          //       arr[ndx + 2] = AllianceFlipUtil.apply(pose.getRotation()).getDegrees();
          //       ndx += 3;
          //     }
          //     Logger.recordOutput("Choreo/Trajectory", arr);
        }));
    // Natural language / AI control bridge. Publishes the /AIControl/ topics on startup and turns
    // incoming tool calls into PathPlanner pathfinding and mechanism commands.
    aiControlBridge =
        new AIControlBridge(drive::getPose, drive::runVelocity, drive)
            .setShootingZone(this::inShootingZone, this::shootingSpot)
            .registerAction("INTAKE", () -> intakeForSeconds(4.0))
            .registerAction("SHOOT_FUEL", () -> shootAtHub(4.0), true)
            .registerAction("SHOOT_ON_THE_MOVE", () -> shootOnTheMove(5.0), true)
            .registerAction("ALIGN_HUB", () -> aimAtHub(3.0), true);
    registerFieldLandmarks(aiControlBridge);
    aiControlActive = new Trigger(aiControlBridge::isBusy);

    configureButtonBindings();
  }

  /**
   * Publishes the handful of field positions an agent needs to turn "go to the hub" or "run the
   * left trench" into coordinates, so the coordinates live here with the rest of the field geometry
   * rather than being copied into a prompt. Blue-origin, like every other pose in the API.
   */
  private static void registerFieldLandmarks(AIControlBridge bridge) {
    double centreY = FieldConstants.fieldWidth / 2.0;
    double leftLaneY =
        (FieldConstants.LinesHorizontal.leftTrenchOpenStart
                + FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
            / 2.0;
    double rightLaneY =
        (FieldConstants.LinesHorizontal.rightTrenchOpenStart
                + FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
            / 2.0;
    double hubX = FieldConstants.LinesVertical.hubCenter;
    double centreX = FieldConstants.LinesVertical.center;

    bridge
        .setFieldSize(FieldConstants.fieldLength, FieldConstants.fieldWidth)
        .registerLandmark("our_hub", new Pose2d(hubX, centreY, Rotation2d.kZero))
        .registerLandmark(
            "our_shooting_spot",
            new Pose2d(hubX - shootingDistanceMeters, centreY, Rotation2d.kZero))
        .registerLandmark(
            "opponent_hub",
            new Pose2d(FieldConstants.LinesVertical.oppHubCenter, centreY, Rotation2d.k180deg))
        .registerLandmark("field_centre", new Pose2d(centreX, centreY, Rotation2d.kZero))
        .registerLandmark("left_trench_near", new Pose2d(hubX, leftLaneY, Rotation2d.kZero))
        .registerLandmark("left_trench_far", new Pose2d(centreX, leftLaneY, Rotation2d.kZero))
        .registerLandmark("right_trench_near", new Pose2d(hubX, rightLaneY, Rotation2d.kZero))
        .registerLandmark("right_trench_far", new Pose2d(centreX, rightLaneY, Rotation2d.kZero));
  }

  /**
   * Lowers the intake, runs it for a while, then folds it back up. Used by the AI control bridge
   * for its "INTAKE" action.
   */
  private Command intakeForSeconds(double seconds) {
    return Commands.startEnd(() -> leftIntake.setLowered(true), () -> leftIntake.setLowered(false))
        .withTimeout(seconds)
        .withName("AIControl/Intake");
  }

  /**
   * Aims and spins up on the hub without feeding fuel, so the robot is ready when the next shoot
   * command arrives. Used by the AI control bridge for its "ALIGN_HUB" action.
   *
   * <p>Aiming is {@link DriveCommands#joystickDriveWhileLaunching} - the same command the driver's
   * right trigger runs - so the AI gets the real launch solution: heading, lead for robot motion,
   * and the launcher centre-of-rotation offset.
   */
  private Command aimAtHub(double seconds) {
    return Commands.parallel(
            DriveCommands.joystickDriveWhileLaunching(drive, () -> 0.0, () -> 0.0),
            flywheelSubsystem.runTrackTargetCommand(),
            hoodSubsystem.runTrackTargetCommand())
        .withTimeout(seconds)
        .withName("AIControl/AlignHub");
  }

  /**
   * Shoots at the hub from a standstill, on the existing launch drive. Used by the AI control
   * bridge for its "SHOOT_FUEL" action; the bridge drives the robot into the alliance zone first,
   * so this is always a hub shot rather than a pass.
   */
  private Command shootAtHub(double seconds) {
    return shootWhileLaunching(() -> 0.0, () -> 0.0, seconds).withName("AIControl/ShootAtHub");
  }

  /**
   * Shoots while translating across the alliance zone, for the AI control bridge's
   * "SHOOT_ON_THE_MOVE" action.
   *
   * <p>The drivetrain stays on {@link DriveCommands#joystickDriveWhileLaunching}, which leads the
   * target for the robot's own velocity and caps translation at what the shot can tolerate. The
   * only thing added here is where to translate to: the opposite side of the same shooting line, so
   * the robot is always moving while it feeds, and always inside the alliance zone.
   */
  private Command shootOnTheMove(double seconds) {
    return Commands.sequence(
            Commands.runOnce(() -> shootingStrafeTarget = pickStrafeTarget()),
            shootWhileLaunching(() -> strafeInput().getX(), () -> strafeInput().getY(), seconds))
        .withName("AIControl/ShootOnTheMove");
  }

  /**
   * The shooting stack: launch drive, hood and flywheel tracking, indexers whenever the shot is
   * ready, and the simulated projectiles that make the shot visible in AdvantageScope.
   *
   * @param xInput driver-frame translation input, as a joystick axis would give it
   * @param yInput driver-frame translation input, as a joystick axis would give it
   */
  private Command shootWhileLaunching(
      DoubleSupplier xInput, DoubleSupplier yInput, double seconds) {
    return Commands.parallel(
            DriveCommands.joystickDriveWhileLaunching(drive, xInput, yInput),
            flywheelSubsystem.runTrackTargetCommand(),
            hoodSubsystem.runTrackTargetCommand(),
            Commands.repeatingSequence(
                new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem, 1.0)
                    .until(() -> readyToShoot == null || !readyToShoot.getAsBoolean())),
            // Feeds the ball sim so the shots are visible in AdvantageScope (sim only).
            Commands.repeatingSequence(
                Commands.waitSeconds(0.25),
                Commands.runOnce(this::launchSimulatedProjectile)
                    .onlyIf(() -> readyToShoot != null && readyToShoot.getAsBoolean())))
        .withTimeout(seconds);
  }

  /**
   * True when the robot is in our alliance zone, behind the hub, which is where a shot counts as a
   * hub shot: {@link LaunchCalculator} switches to a lob pass the moment the robot is past the hub,
   * so shooting from the neutral zone throws fuel back at our own alliance zone instead of scoring.
   */
  private boolean inShootingZone() {
    return AllianceFlipUtil.applyX(drive.getPose().getX())
        <= FieldConstants.LinesVertical.hubCenter - shootingZoneMarginMeters;
  }

  /** Where to stand to take a hub shot: on the shooting line, level with the hub. */
  private Pose2d shootingSpot() {
    double y = drive.getPose().getY();
    double clamped =
        MathUtil.clamp(
            y, hubShootingLineY() - strafeHalfWidth, hubShootingLineY() + strafeHalfWidth);
    return AllianceFlipUtil.apply(
        new Pose2d(
            FieldConstants.LinesVertical.hubCenter - shootingDistanceMeters,
            AllianceFlipUtil.applyY(clamped),
            Rotation2d.kZero));
  }

  /** Centre of the shooting line, i.e. the hub's Y. */
  private static double hubShootingLineY() {
    return FieldConstants.fieldWidth / 2.0;
  }

  /** The far end of the shooting line from wherever the robot is now. */
  private Translation2d pickStrafeTarget() {
    boolean aboveCentre = AllianceFlipUtil.applyY(drive.getPose().getY()) > hubShootingLineY();
    return AllianceFlipUtil.apply(
        new Translation2d(
            FieldConstants.LinesVertical.hubCenter - shootingDistanceMeters,
            hubShootingLineY() + (aboveCentre ? -strafeHalfWidth : strafeHalfWidth)));
  }

  /**
   * Translation input toward {@link #shootingStrafeTarget}, in the driver frame the launch drive
   * expects (it flips the axes for the red alliance, so pre-flip them here).
   */
  private Translation2d strafeInput() {
    if (shootingStrafeTarget == null) {
      return Translation2d.kZero;
    }
    Translation2d error = shootingStrafeTarget.minus(drive.getPose().getTranslation());
    if (error.getNorm() < 0.25) {
      return Translation2d.kZero;
    }
    Translation2d input = error.div(error.getNorm()).times(strafeInputScalar);
    return AllianceFlipUtil.shouldFlip() ? input.unaryMinus() : input;
  }

  /**
   * Builds the trigger chain that gates shooting. The final gate is {@code readyToShoot}:
   *
   * <pre>
   * readyToShoot = launch parameters valid
   *              AND inLaunchingTolerance (debounced 0.25s on release)
   *              AND (hub active OR passing OR "Ignore Hub State" override in Elastic)
   * </pre>
   */
  private void configureShootingTriggers() {
    // Hub shots are only allowed while our hub is active; passes are exempt from the shift state.
    hubActiveOrPassing =
        new Trigger(
            () ->
                HubShiftUtil.getOfficialShiftInfo().active()
                    || LaunchCalculator.getInstance().getParameters().passing());

    // Heading cone for passing: wide enough not to fight the driver, tight enough to keep passes
    // inside the field. Can be disabled from Elastic ("Enable Passing Cone", null-safe default
    // Yes).
    inPassingTolerance =
        new Trigger(
            () -> {
              Boolean enableCone = enablePassingConeChooser.get();
              boolean coneEnabled = (enableCone == null) || enableCone;
              return !coneEnabled || passingHeadingErrorDeg() <= passingHeadingToleranceDeg.get();
            });

    // Hood + flywheel must always be at setpoint; the heading gate depends on the shot type:
    // hub shots use the drive launch goal, passes use the looser passing cone.
    inLaunchingTolerance =
        new Trigger(
            () ->
                hoodSubsystem.atSetpoint()
                    && flywheelSubsystem.atSetpoint()
                    && (LaunchCalculator.getInstance().getParameters().passing()
                        ? inPassingTolerance.getAsBoolean()
                        : DriveCommands.atLaunchGoal()));

    readyToShoot =
        new Trigger(() -> LaunchCalculator.getInstance().getParameters().isValid())
            .and(inLaunchingTolerance.debounce(0.25, DebounceType.kFalling))
            .and(ignoreHubState.or(hubActiveOrPassing));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Reset hub shift timer when enabling
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));

    // Elastic tab switching
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> Elastic.selectTab("Teleoperated")));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> Elastic.selectTab("Autonomous")));

    // Drive controls
    DoubleSupplier driverX = () -> -driveController.getLeftY();
    DoubleSupplier driverY = () -> -driveController.getLeftX();
    DoubleSupplier driverOmega = () -> -driveController.getRightX();

    double robotHalfWidth = Units.inchesToMeters(17.407);
    Trigger nearTrench =
        new Trigger(
            () -> {
              double x = frc.robot.util.geometry.AllianceFlipUtil.applyX(drive.getPose().getX());
              double y = drive.getPose().getY();

              boolean inTrenchX =
                  x > (FieldConstants.LeftBump.nearLeftCorner.getX() - trenchExtension.get())
                      && x < (FieldConstants.LeftBump.farLeftCorner.getX() + trenchExtension.get());
              boolean inRightTrench =
                  y > robotHalfWidth
                      && y < (FieldConstants.LinesHorizontal.rightTrenchOpenStart - robotHalfWidth);
              boolean inLeftTrench =
                  y > (FieldConstants.LinesHorizontal.leftTrenchOpenEnd + robotHalfWidth)
                      && y < (FieldConstants.fieldWidth - robotHalfWidth);

              return inTrenchX && (inRightTrench || inLeftTrench);
            });

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega, maxOmegaScalar::get));
    // ledSubsystem.setDefaultCommand(Commands.run(ledSubsystem::SetIdle));

    // Lock to 0 when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> Rotation2d.kZero));
    nearTrench
        .and(RobotModeTriggers.teleop())
        .and(driveController.rightTrigger().negate())
        .and(autoAlignmentOverride.negate())
        .and(aiControlActive.negate())
        .whileTrue(
            DriveCommands.autoTrenchAssist(
                    drive,
                    driverX,
                    driverY,
                    driverOmega,
                    maxOmegaScalar::get,
                    leftIntake::isLowered,
                    trenchAlignmentPositionChooser::get)
                .withName("AlignToTrenchCommand"));

    // driveController
    //     .rightStick()
    //     .onTrue(Commands.runOnce(() -> autoAlignmentOverrideState =
    // !autoAlignmentOverrideState));

    driveController
        .rightTrigger()
        .whileTrue(DriveCommands.joystickDriveWhileLaunching(drive, driverX, driverY))
        .whileTrue(flywheelSubsystem.runTrackTargetCommand())
        .whileTrue(hoodSubsystem.runTrackTargetCommand());
    // .onFalse(
    //     Commands.deadline(
    //         Commands.waitSeconds(1.0),
    //         new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem, -0.5)));

    // driveController
    //     .a()
    //     .whileTrue(new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem));

    driveController
        .rightTrigger()
        .and(readyToShoot)
        .whileTrue(
            Commands.parallel(
                new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem, 1.0),
                Commands.repeatingSequence(
                    Commands.waitSeconds(0.4), Commands.runOnce(this::launchSimulatedProjectile))));

    // Lower the intake while the left trigger is held, raise it when released
    driveController
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> leftIntake.setLowered(true)))
        .onFalse(Commands.runOnce(() -> leftIntake.setLowered(false)));

    driveController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> leftIntake.forceReverse(true)))
        .onFalse(Commands.runOnce(() -> leftIntake.forceReverse(false)));

    // While intaking and not yet shooting, run the spindexer in reverse (chooser-gated)
    leftIntakeLowered
        .and(readyToShoot.negate())
        .and(reverseIndexWhileIntake)
        // Teleop only: in auto the running routine owns the spindexer, so letting this trigger grab
        // it would evict and cancel the whole auto (this is what stopped auto ~1s in).
        .and(RobotModeTriggers.teleop())
        .whileTrue(
            Commands.runEnd(
                () -> {
                  spindexerSubsystem.setPercentage(-0.35);
                  spindexerSubsystem.setSub(0.5);
                },
                spindexerSubsystem::stop,
                spindexerSubsystem));

    // Test specific button for simulated launch

    // Switch to X pattern when X button is pressed
    // driveController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    final Runnable resetOdometry =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

    driveController
        .rightBumper()
        .whileTrue(
            Commands.deadline(
                Commands.waitSeconds(1.0),
                new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem, -0.33)));

    SmartDashboard.putData("IntakeOut", Commands.runOnce(() -> leftIntake.setLowered(true)));
    SmartDashboard.putData("IntakeIn", Commands.runOnce(() -> leftIntake.setLowered(false)));
    SmartDashboard.putData(
        "Run both Indexers",
        new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem, 1.0));
    SmartDashboard.putData(
        "Invert both Indexers",
        Commands.deadline(
            Commands.waitSeconds(1.0),
            new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem, -0.5)));
    SmartDashboard.putData("Zero Hood", hoodSubsystem.zeroCommand());

    // Live flywheel RPM calibration: nudge the launch calculation output up/down by 25 RPM.
    SmartDashboard.putData(
        "Flywheel RPM +25",
        Commands.runOnce(() -> LaunchCalculator.getInstance().incrementFlywheelRpmOffset(25.0))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Flywheel RPM -25",
        Commands.runOnce(() -> LaunchCalculator.getInstance().incrementFlywheelRpmOffset(-25.0))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Flywheel RPM Reset",
        Commands.runOnce(() -> LaunchCalculator.getInstance().resetFlywheelRpmOffset())
            .ignoringDisable(true));

    // Live target calibration: nudge the aimed hub target left/right from the driver's point of
    // view (positive Y = driver's left, per the field coordinate convention).
    SmartDashboard.putData(
        "Target Left 5cm",
        Commands.runOnce(() -> LaunchCalculator.getInstance().incrementTargetYOffsetCm(5))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Target Right 5cm",
        Commands.runOnce(() -> LaunchCalculator.getInstance().incrementTargetYOffsetCm(-5))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Target Left 10cm",
        Commands.runOnce(() -> LaunchCalculator.getInstance().incrementTargetYOffsetCm(10))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Target Right 10cm",
        Commands.runOnce(() -> LaunchCalculator.getInstance().incrementTargetYOffsetCm(-10))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Target Reset",
        Commands.runOnce(() -> LaunchCalculator.getInstance().resetTargetYOffset())
            .ignoringDisable(true));

    SmartDashboard.putData(
        "Lower left intake", Commands.runOnce(() -> leftIntake.setLowered(true)));
    SmartDashboard.putData(
        "Rasise left intake", Commands.runOnce(() -> leftIntake.setLowered(false)));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetOdometry(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.kZero))),
                    drive)
                .ignoringDisable(true));

    if (Constants.flywheelTestingMode) {
      // Calibration mode: hold the tunable target RPM so the shooter can be characterized
      flywheelSubsystem.setDefaultCommand(flywheelSubsystem.runFlywheelCommand());
    } else {
      flywheelSubsystem.setDefaultCommand(
          new ContinuousConditionalCommand(
              Commands.runOnce(flywheelSubsystem::stop, flywheelSubsystem),
              flywheelSubsystem.runAtSpeedRPMCommand(
                  () -> LaunchCalculator.getInstance().getParameters().flywheelIdleSpeed()),
              disableFlywheelAutoSpinup));
    }

    hoodSubsystem.setDefaultCommand(
        Commands.sequence(hoodSubsystem.zeroCommand(), hoodSubsystem.runTargetAngleCommand())
            .withName("HoodDefault"));

    // Folded baseline: 0.5 when shooting (trigger held), 0 when idle
    leftIntake.setDefaultCommand(
        Commands.run(
            () -> {
              if (leftIntake.isLowered()) {
                leftIntake.setPercentage(1.00);
              } else {
                leftIntake.setPercentage(spindexerSubsystem.getAppliedVolts() > 0.1 ? 0.35 : 0.0);
              }
            },
            leftIntake));

    // When folding/unfolding
    leftIntakeLowered.onFalse(
        Commands.run(() -> leftIntake.setPercentage(0.4), leftIntake)
            .withTimeout(intakeRunWheelsWhileFoldingDelay.get())
            .onlyIf(() -> !Boolean.FALSE.equals(runWheelsWhenFoldingChooser.get()))); // default Yes

    // ****** RUMBLE ALERTS ******
    // intakeStruggling
    //     .whileTrue(
    //         Commands.runOnce(
    //             () ->
    //                 driveController.setRumble(
    //                     edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0)))
    //     .whileFalse(
    //         Commands.runOnce(
    //             () ->
    //                 driveController.setRumble(
    //                     edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0)));

    // Reset the timer as soon as Teleop starts
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(teleopElapsedTimer::restart));

    spindexerSubsystem.setDefaultCommand(
        Commands.run(() -> spindexerSubsystem.setPercentage(-0.1), spindexerSubsystem));
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    Logger.recordOutput("AutoAlignment/OverrideToggle", autoAlignmentOverrideState);
    // Publish match time
    SmartDashboard.putNumber("Match Time", HubShiftUtil.getMatchTime());
    if (compressor != null) {
      SmartDashboard.putNumber("PSI", compressor.getPressure());
    }

    // Current flywheel RPM calibration offset applied to the launch calculation output
    SmartDashboard.putNumber(
        "Flywheel RPM Offset", LaunchCalculator.getInstance().getFlywheelRpmOffset());
    // Current target Y calibration offset applied to the aimed hub target
    SmartDashboard.putNumber(
        "Target Y Offset", LaunchCalculator.getInstance().getTargetYOffsetMeters());

    // Controller disconnected alerts
    driverControllerDisconnected.set(!driveController.isConnected());
    // mechanismControllerDisconnected.set(
    //     !DriverStation.isJoystickConnected(mechanismController.getHID().getPort()));

    // Update from HubShiftUtil
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getOfficialShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getOfficialShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());

    // For displaying in Elastic
    field.setRobotPose(drive.getPose());

    // --- Leaf conditions that gate shooting (see configureShootingTriggers) ---
    // Shared (both hub shots and passes)
    Logger.recordOutput(
        "LaunchCalculator/Conditions/ParametersValid",
        LaunchCalculator.getInstance().getParameters().isValid());
    Logger.recordOutput("LaunchCalculator/Conditions/HoodAtSetpoint", hoodSubsystem.atSetpoint());
    Logger.recordOutput(
        "LaunchCalculator/Conditions/FlywheelAtSetpoint", flywheelSubsystem.atSetpoint());

    // Hub shots only
    Logger.recordOutput(
        "LaunchCalculator/Conditions/Hub/DriveAtLaunchGoal", DriveCommands.atLaunchGoal());
    Logger.recordOutput(
        "LaunchCalculator/Conditions/Hub/HubActive", HubShiftUtil.getOfficialShiftInfo().active());
    Logger.recordOutput(
        "LaunchCalculator/Conditions/Hub/HubIgnored", ignoreHubState.getAsBoolean());

    // Passes only
    Logger.recordOutput(
        "LaunchCalculator/Conditions/Passing/InPassingTolerance",
        inPassingTolerance != null && inPassingTolerance.getAsBoolean());
    Logger.recordOutput(
        "LaunchCalculator/Conditions/Passing/HeadingErrorDeg", passingHeadingErrorDeg());
    Logger.recordOutput(
        "LaunchCalculator/Conditions/Passing/HeadingToleranceDeg",
        passingHeadingToleranceDeg.get());

    // Final ANDed gate
    Logger.recordOutput(
        "LaunchCalculator/ReadyToShoot", readyToShoot != null && readyToShoot.getAsBoolean());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.resetOdometry(
        new Pose2d(
            AllianceFlipUtil.applyX(3.591),
            AllianceFlipUtil.applyY(7.430),
            AllianceFlipUtil.apply(Rotation2d.fromDegrees(-90))));
    SimulatedArena.getInstance().resetFieldForAuto();
    ballSim.clearBalls();
    ballSim.placeFieldBalls();
    ballSim.resetCounters();
    if (intakeSimIO != null) {
      intakeSimIO.setHeldCount(SIM_FUEL_PRELOAD);
    }
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();

    edu.wpi.first.math.geometry.Pose2d simPose = driveSimulation.getSimulatedDriveTrainPose();
    edu.wpi.first.math.kinematics.ChassisSpeeds fieldSpeeds =
        edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
            driveSimulation.getDriveTrainSimulatedChassisSpeedsRobotRelative(),
            simPose.getRotation());
    edu.wpi.first.math.geometry.Pose3d simPose3d = robotBumpSim.update(simPose, fieldSpeeds, 5);
    boolean currentlyOnRamp = robotBumpSim.isOnRamp();
    if (currentlyOnRamp || wasOnRamp) {
      Pose2d correctedPose = robotBumpSim.getSimWorldPose(simPose);
      driveSimulation.setSimulationWorldPose(correctedPose);
      if (wasOnRamp && !currentlyOnRamp) {
        drive.resetOdometry(correctedPose);
      }
    }
    wasOnRamp = currentlyOnRamp;
    Logger.recordOutput("Drive/Pose3d", simPose3d);

    Logger.recordOutput("FieldSimulation/RobotPosition", simPose);

    ballSim.configureRobot(
        0.7,
        0.7,
        0.15, // width, length, bumperH (approximate values)
        drive::getPose,
        drive::getChassisSpeeds);
    ballSim.tick();

    if (intakeSimIO != null) {
      Logger.recordOutput("Sim/Fuel/HeldBalls", intakeSimIO.getHeldCount());
      Logger.recordOutput("Sim/Fuel/IntakeRunning", intakeSimIO.isIntakeRunning());
    }

    // Object detection camera: re-publish the live fuel positions and process a frame.
    if (gamePieceVisionSim != null) {
      gamePieceVisionSim.update();
    }
  }

  /** Every fuel the cameras could see: MapleSim's field pieces plus the ones in flight. */
  private List<Translation3d> simulatedGamePieces() {
    List<Translation3d> pieces = new ArrayList<>(ballSim.getBallPositions());
    for (Pose3d pose : SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel")) {
      pieces.add(pose.getTranslation());
    }
    return pieces;
  }

  private void launchSimulatedProjectile() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    // Can only shoot fuel the robot actually holds (preload + intaked)
    if (intakeSimIO != null && !intakeSimIO.obtainGamePiece()) return;

    var params = LaunchCalculator.getInstance().getParameters();
    if (params == null || !params.isValid()) return;

    Pose2d robotPose = drive.getPose();
    Rotation2d actualDriveAngle = drive.getRotation();

    // 1. Calculate launcher position in 3D space
    Translation2d launcherPos2d =
        robotPose
            .getTranslation()
            .plus(
                frc.robot.subsystems.shooter.LauncherConstants.robotToLauncher
                    .getTranslation()
                    .toTranslation2d()
                    .rotateBy(actualDriveAngle));

    double launcherZ = frc.robot.subsystems.shooter.LauncherConstants.robotToLauncher.getZ();
    edu.wpi.first.math.geometry.Translation3d launcherPos =
        new edu.wpi.first.math.geometry.Translation3d(
            launcherPos2d.getX(), launcherPos2d.getY(), launcherZ);

    // 2. Real-World Exit Velocity (Forward Kinematics)
    // The ball's speed relative to the robot depends strictly on RPM and slip factor.
    double rpm = params.flywheelSpeed();
    double wheelDiameterMeters = 0.1524; // 6 inches
    double slipFactor = 0.54; // The true slip factor we derived
    double exitSpeed = slipFactor * (rpm * Math.PI * wheelDiameterMeters) / 60.0;

    // 3. Convert scalar speed to a 3D vector relative to the robot
    // Note: Hood angle parameter is assumed to be elevation from horizontal.
    // If it's from vertical, adjust as: (Math.PI / 2.0) - (baseHoodRad + params.hoodAngle())
    double baseHoodRad = edu.wpi.first.math.util.Units.degreesToRadians(12.5);
    double launchRad = (Math.PI / 2.0) - (baseHoodRad + params.hoodAngle());

    double ballRobotRelativeVx = exitSpeed * Math.cos(launchRad);
    double ballRobotRelativeVy = 0.0;
    double ballRobotRelativeVz = exitSpeed * Math.sin(launchRad);

    // 4. Rotate to Field Frame based on the robot's physical heading
    double ballFieldVx =
        ballRobotRelativeVx * actualDriveAngle.getCos()
            - ballRobotRelativeVy * actualDriveAngle.getSin();
    double ballFieldVy =
        ballRobotRelativeVx * actualDriveAngle.getSin()
            + ballRobotRelativeVy * actualDriveAngle.getCos();

    // 5. INHERIT ROBOT MOMENTUM (The part the CD thread missed)
    var robotSpeeds = drive.getChassisSpeeds();
    double launcherOffsetX = frc.robot.subsystems.shooter.LauncherConstants.robotToLauncher.getX();
    double launcherOffsetY = frc.robot.subsystems.shooter.LauncherConstants.robotToLauncher.getY();

    // Tangential velocity (v = w x r) + Translational velocity
    double launcherRobotRelativeVx =
        robotSpeeds.vxMetersPerSecond - (robotSpeeds.omegaRadiansPerSecond * launcherOffsetY);
    double launcherRobotRelativeVy =
        robotSpeeds.vyMetersPerSecond + (robotSpeeds.omegaRadiansPerSecond * launcherOffsetX);

    // Rotate launcher velocity to field frame
    double launcherFieldVx =
        launcherRobotRelativeVx * actualDriveAngle.getCos()
            - launcherRobotRelativeVy * actualDriveAngle.getSin();
    double launcherFieldVy =
        launcherRobotRelativeVx * actualDriveAngle.getSin()
            + launcherRobotRelativeVy * actualDriveAngle.getCos();

    // 6. Final absolute velocity = Ball exit velocity + Launcher velocity
    double finalVx = ballFieldVx + launcherFieldVx;
    double finalVy = ballFieldVy + launcherFieldVy;
    double finalVz = ballRobotRelativeVz;

    edu.wpi.first.math.geometry.Translation3d launchVelocity =
        new edu.wpi.first.math.geometry.Translation3d(finalVx, finalVy, finalVz);

    ballSim.launchBall(launcherPos, launchVelocity, rpm);
  }

  /** Heading error (degrees) between the robot and the passing target. */
  private double passingHeadingErrorDeg() {
    return Math.abs(
        Drive.getInstance()
            .getRotation()
            .minus(LaunchCalculator.getInstance().getParameters().driveAngle())
            .getDegrees());
  }

  public Command testAuto() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    AutoTrajectory trenchShallowIntake = routine.trajectory("left_trench_Shallow_Intake");
    AutoTrajectory trenchDeepIntake = routine.trajectory("left_trench_Deep_Intake");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                trenchShallowIntake.resetOdometry(),
                autoShoot(3.0),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                trenchShallowIntake.cmd().finallyDo(() -> drive.stopWithX()),
                autoShoot(5),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                trenchDeepIntake.cmd().finallyDo(() -> drive.stopWithX()),
                autoShoot(2.5)));

    return routine.cmd();
  }

  /**
   * Returns a command that aims and shoots for a given duration. Runs the flywheel, hood tracking,
   * indexers (when ready), and simulated projectile launches in parallel.
   *
   * @param timeoutSeconds how long to attempt shooting before moving on
   */
  private Command autoShoot(double timeoutSeconds) {
    return Commands.parallel(
            DriveCommands.joystickDriveWhileLaunching(drive, () -> 0.0, () -> 0.0),
            flywheelSubsystem.runTrackTargetCommand(),
            hoodSubsystem.runTrackTargetCommand(),
            Commands.repeatingSequence(
                new RunBothIndexersCommand(spindexerSubsystem, shooterIndexerSubsystem, 1.0)
                    .until(() -> readyToShoot == null || !readyToShoot.getAsBoolean())),
            // Launches simulated projectiles for the ball sim. This only affects simulation
            // (no-op on the real robot) - do not remove it.
            Commands.repeatingSequence(
                Commands.waitSeconds(0.25),
                Commands.runOnce(this::launchSimulatedProjectile)
                    .onlyIf(() -> readyToShoot != null && readyToShoot.getAsBoolean())))
        .withTimeout(timeoutSeconds);
  }

  public Command leftTrenchDoubleTake() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    AutoTrajectory trenchShallowIntake = routine.trajectory("left_trench_Shallow_Intake");
    AutoTrajectory trenchDeepIntake = routine.trajectory("left_trench_Deep_Intake");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> hoodSubsystem.zero()),
                trenchShallowIntake.resetOdometry(),
                Commands.sequence(Commands.runOnce(() -> leftIntake.setLowered(false))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                trenchShallowIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> leftIntake.setLowered(false)))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                trenchDeepIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> leftIntake.setLowered(false))))));

    return routine.cmd();
  }

  public Command Deot() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    AutoTrajectory Depot = routine.trajectory("Depot");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> hoodSubsystem.zero()),
                Depot.resetOdometry(),
                Commands.sequence(Commands.runOnce(() -> leftIntake.setLowered(false))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                Depot.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(10),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> leftIntake.setLowered(false))))));

    return routine.cmd();
  }

  public Command leftTrenchSingleTake() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    AutoTrajectory trenchShallowIntake = routine.trajectory("left_trench_Shallow_Intake");
    AutoTrajectory trenchDeepIntake = routine.trajectory("left_trench_Deep_Intake");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> hoodSubsystem.zero()),
                trenchShallowIntake.resetOdometry(),
                Commands.sequence(Commands.runOnce(() -> leftIntake.setLowered(false))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                trenchShallowIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(15),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> leftIntake.setLowered(false)))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                trenchDeepIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> leftIntake.setLowered(false))))));

    return routine.cmd();
  }

  public Command leftTrenchIntakeReturnOverBump() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    AutoTrajectory firstIntake = routine.trajectory("Left_Trench_Return_Over_Bump");
    AutoTrajectory SecnondIntake = routine.trajectory("Left_Trench_Return_Over_Bump_2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> hoodSubsystem.zero()),
                firstIntake.resetOdometry(),
                Commands.runOnce(
                    () -> {
                      leftIntake.setLowered(false);
                    }),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                firstIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                            () -> {
                              leftIntake.setLowered(false);
                            }))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                SecnondIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                            () -> {
                              leftIntake.setLowered(false);
                            })))));

    return routine.cmd();
  }

  public Command rightTrenchIntakeReturnOverBump() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    AutoTrajectory firstIntake = routine.trajectory("Right_Trench_Return_Over_Bump");
    AutoTrajectory SecnondIntake = routine.trajectory("Right_Trench_Return_Over_Bump_2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> hoodSubsystem.zero()),
                firstIntake.resetOdometry(),
                Commands.runOnce(
                    () -> {
                      leftIntake.setLowered(false);
                    }),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                firstIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                            () -> {
                              leftIntake.setLowered(false);
                            }))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.2),
                SecnondIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(2.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                            () -> {
                              leftIntake.setLowered(false);
                            })))));

    return routine.cmd();
  }

  public Command leftTrenchHubIntakeReturnOverBump() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    AutoTrajectory firstIntake = routine.trajectory("Behind_Hub_Intake_1");
    AutoTrajectory SecnondIntake = routine.trajectory("Behind_Hub_Intake_2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                firstIntake.resetOdometry(),
                Commands.runOnce(() -> hoodSubsystem.zero()),
                Commands.sequence(
                    Commands.parallel(autoShoot(3.5), Commands.waitSeconds(1.0)),
                    Commands.runOnce(
                        () -> {
                          leftIntake.setLowered(false);
                        })),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.5),
                firstIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(3.5),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                            () -> {
                              leftIntake.setLowered(false);
                            }))),
                Commands.runOnce(() -> hoodSubsystem.setTargetAngle(0.0), hoodSubsystem)
                    .withTimeout(0.5),
                SecnondIntake.cmd().finallyDo(() -> drive.stopWithX()),
                Commands.parallel(
                    autoShoot(7),
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(
                            () -> {
                              leftIntake.setLowered(false);
                            })))));

    return routine.cmd();
  }
}
