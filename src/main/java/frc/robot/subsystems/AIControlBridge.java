// Copyright (c) 2026 Skynet 4416
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * NetworkTables bridge that lets an external agent (for example a Gemini Robotics ER 2 tool call
 * turning a natural language prompt into a robot command) drive the robot.
 *
 * <p>The whole API lives under the {@code /AIControl/} table, so it shows up as one folder in
 * AdvantageScope:
 *
 * <table>
 *   <tr><th>Topic</th><th>Type</th><th>Meaning</th></tr>
 *   <tr><td>{@code TargetPose}</td><td>{@code double[3]}</td>
 *       <td>{@code [x meters, y meters, heading degrees]}, blue-origin field coordinates. Writing
 *       it starts a PathPlanner {@code pathfindToPose} to that pose.</td></tr>
 *   <tr><td>{@code MaxSpeed} / {@code MaxAccel}</td><td>{@code double}</td>
 *       <td>Pathfinding limits in m/s and m/s^2, applied to the next path. Crank them up for a
 *       running start at a bump, down to creep into position.</td></tr>
 *   <tr><td>{@code ActionTrigger}</td><td>{@code String}</td>
 *       <td>Name of a registered mechanism action, e.g. {@code "INTAKE"}, {@code "SHOOT_FUEL"},
 *       {@code "ALIGN_HUB"}. {@code "STOP"} cancels everything.</td></tr>
 * </table>
 *
 * <p><b>All robot movement goes through PathPlanner.</b> Navigation is a pathfinding command;
 * aiming is PathPlanner's own rotation controller with its target rotation overridden. Actions run
 * in a second, independent slot, so a shooting action layered on top of an in-progress path gives
 * you shooting on the move for free.
 *
 * <p><b>The agent does not own the heading while a shooting action runs.</b> Those actions are
 * registered as rotation-locking: while one is active the robot's heading is whatever the launch
 * solution needs, and the heading in {@code TargetPose} is ignored (the x/y of a path still gets
 * followed). {@code /AIControl/RotationLocked} says when that is the case, and {@code Notes} spells
 * the rule out for the agent in plain text.
 *
 * <p>Read-only feedback: {@code RobotPose}, {@code Navigating}, {@code ActionRunning}, {@code
 * RotationLocked}, {@code AtTarget}, {@code ActiveTarget}, {@code Status}, {@code LastAction},
 * {@code LastError}, {@code AvailableActions}, {@code HeadingUnits} and {@code Notes}. Every topic
 * is created in the constructor, so they are all there the moment the robot code starts.
 */
public class AIControlBridge extends SubsystemBase {
  /** NetworkTables table that holds the whole API. */
  public static final String TABLE_NAME = "AIControl";

  /** Cancels the running path and action. Always available, never registered by the caller. */
  public static final String STOP_ACTION = "STOP";

  private static final String NOTES =
      "Write /AIControl/TargetPose = [x_m, y_m, heading_deg] to drive somewhere (PathPlanner "
          + "pathfinding). Write /AIControl/ActionTrigger = an entry of AvailableActions to run a "
          + "mechanism. Paths and actions run at the same time, so triggering SHOOT_FUEL while a "
          + "path is running shoots on the move. While RotationLocked is true the shooter owns the "
          + "robot heading and the heading you asked for is ignored until the action finishes. "
          + "Set /AIControl/MaxSpeed (m/s) and /AIControl/MaxAccel (m/s^2) before a TargetPose to "
          + "drive gently or to build up speed. The bumps beside each hub are only crossable with "
          + "a running start - back off a couple of metres and cross at 4+ m/s, or take the flat "
          + "trench lanes along either side wall instead. STOP cancels everything.";

  // Default pathfinding limits. The agent can change them live through MaxSpeed / MaxAccel - worth
  // doing, since crossing a bump needs a running start while lining up on something wants slow.
  private static final double DEFAULT_MAX_SPEED_MPS = 2.5;
  private static final double DEFAULT_MAX_ACCEL_MPS2 = 2.5;
  private static final double MIN_MAX_SPEED_MPS = 0.3;
  private static final double MAX_MAX_SPEED_MPS = 4.5;
  private static final double MIN_MAX_ACCEL_MPS2 = 0.3;
  private static final double MAX_MAX_ACCEL_MPS2 = 8.0;
  private static final double MAX_ANGULAR_SPEED_DEG_PER_SEC = 360.0;
  private static final double MAX_ANGULAR_ACCEL_DEG_PER_SEC2 = 540.0;

  /**
   * A path keeps replanning until the robot is inside these tolerances, or this long has passed.
   */
  private static final double POSE_TOLERANCE_METERS = 0.15;

  private static final double HEADING_TOLERANCE_DEGREES = 5.0;
  private static final double NAVIGATION_TIMEOUT_SECONDS = 20.0;

  /**
   * PathPlanner's pathfinder stops generating paths once the robot is nearly there, which leaves it
   * parked wherever the last trajectory ended. Inside this radius the same holonomic controller
   * PathPlanner follows paths with drives the last few centimetres onto the pose.
   */
  private static final double FINAL_APPROACH_RADIUS_METERS = 0.75;

  // Heading controller used while the shooter owns the rotation. Its output is handed to
  // PathPlanner as rotation feedback, so PathPlanner still drives - it just aims where we say.
  private static final double AIM_KP = 5.0;
  private static final double MAX_AIM_OMEGA_RAD_PER_SEC = Units.degreesToRadians(540.0);

  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<ChassisSpeeds> velocityConsumer;
  private final Subsystem driveSubsystem;
  private final Map<String, RegisteredAction> actions = new LinkedHashMap<>();

  /** Heading the shooter wants; owns the robot's rotation while a rotation-locking action runs. */
  private Supplier<Optional<Rotation2d>> aimHeadingSupplier = Optional::empty;

  // Inputs (written by the AI agent)
  private final DoubleArraySubscriber targetPoseSub;
  private final StringSubscriber actionTriggerSub;

  // Input topics are published once so they exist (with a neutral value) at startup.
  private final DoubleArrayPublisher targetPosePub;
  private final StringPublisher actionTriggerPub;
  private final DoublePublisher maxSpeedPub;
  private final DoublePublisher maxAccelPub;
  private final DoubleSubscriber maxSpeedSub;
  private final DoubleSubscriber maxAccelSub;

  // Outputs (read by the AI agent)
  private final DoubleArrayPublisher robotPosePub;
  private final DoubleArrayPublisher activeTargetPub;
  private final BooleanPublisher navigatingPub;
  private final BooleanPublisher actionRunningPub;
  private final BooleanPublisher rotationLockedPub;
  private final BooleanPublisher atTargetPub;
  private final StringPublisher statusPub;
  private final StringPublisher lastActionPub;
  private final StringPublisher lastErrorPub;
  private final StringArrayPublisher availableActionsPub;
  private final StringPublisher headingUnitsPub;
  private final StringPublisher notesPub;

  private Command navigationCommand = null;
  private Command actionCommand = null;
  private Pose2d activeTarget = null;
  private String lastAction = "";
  private boolean rotationLocked = false;
  private final PIDController aimController = new PIDController(AIM_KP, 0.0, 0.0);

  // PathPlanner's holonomic controller, used for the final approach onto the requested pose.
  private final PPHolonomicDriveController approachController =
      new PPHolonomicDriveController(
          new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(3.0, 0.0, 0.0));
  private boolean wasBusy = false;

  /** An action the agent can trigger by name. */
  private record RegisteredAction(Supplier<Command> commandSupplier, boolean locksRotation) {}

  /**
   * @param poseSupplier the robot's current field pose
   * @param velocityConsumer runs robot-relative chassis speeds, used only to turn in place while
   *     aiming from a standstill - every meter of travel comes from PathPlanner
   * @param driveSubsystem the drivetrain, required by the aiming command so it and a path can never
   *     fight over the drivetrain
   */
  public AIControlBridge(
      Supplier<Pose2d> poseSupplier,
      Consumer<ChassisSpeeds> velocityConsumer,
      Subsystem driveSubsystem) {
    this.poseSupplier = poseSupplier;
    this.velocityConsumer = velocityConsumer;
    this.driveSubsystem = driveSubsystem;

    aimController.enableContinuousInput(-Math.PI, Math.PI);

    NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);

    var targetPoseTopic = table.getDoubleArrayTopic("TargetPose");
    targetPosePub = targetPoseTopic.publish();
    targetPosePub.set(new double[0]);
    targetPoseSub =
        targetPoseTopic.subscribe(
            new double[0], PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

    var actionTriggerTopic = table.getStringTopic("ActionTrigger");
    actionTriggerPub = actionTriggerTopic.publish();
    actionTriggerPub.set("");
    actionTriggerSub =
        actionTriggerTopic.subscribe(
            "", PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

    var maxSpeedTopic = table.getDoubleTopic("MaxSpeed");
    maxSpeedPub = maxSpeedTopic.publish();
    maxSpeedPub.set(DEFAULT_MAX_SPEED_MPS);
    maxSpeedSub = maxSpeedTopic.subscribe(DEFAULT_MAX_SPEED_MPS);

    var maxAccelTopic = table.getDoubleTopic("MaxAccel");
    maxAccelPub = maxAccelTopic.publish();
    maxAccelPub.set(DEFAULT_MAX_ACCEL_MPS2);
    maxAccelSub = maxAccelTopic.subscribe(DEFAULT_MAX_ACCEL_MPS2);

    robotPosePub = table.getDoubleArrayTopic("RobotPose").publish();
    activeTargetPub = table.getDoubleArrayTopic("ActiveTarget").publish();
    navigatingPub = table.getBooleanTopic("Navigating").publish();
    actionRunningPub = table.getBooleanTopic("ActionRunning").publish();
    rotationLockedPub = table.getBooleanTopic("RotationLocked").publish();
    atTargetPub = table.getBooleanTopic("AtTarget").publish();
    statusPub = table.getStringTopic("Status").publish();
    lastActionPub = table.getStringTopic("LastAction").publish();
    lastErrorPub = table.getStringTopic("LastError").publish();
    availableActionsPub = table.getStringArrayTopic("AvailableActions").publish();
    headingUnitsPub = table.getStringTopic("HeadingUnits").publish();
    notesPub = table.getStringTopic("Notes").publish();

    robotPosePub.set(toArray(poseSupplier.get()));
    activeTargetPub.set(new double[0]);
    navigatingPub.set(false);
    actionRunningPub.set(false);
    rotationLockedPub.set(false);
    atTargetPub.set(false);
    statusPub.set("IDLE");
    lastActionPub.set("");
    lastErrorPub.set("");
    headingUnitsPub.set("degrees");
    notesPub.set(NOTES);
    publishAvailableActions();
  }

  /**
   * Sets the heading the shooter needs. While a rotation-locking action runs this is pushed into
   * PathPlanner as the path's rotation target, so aiming and path following are the same motion.
   */
  public AIControlBridge setAimHeadingSupplier(Supplier<Optional<Rotation2d>> supplier) {
    this.aimHeadingSupplier = supplier;
    return this;
  }

  /**
   * Registers a named mechanism action the agent can trigger by writing its name to {@code
   * /AIControl/ActionTrigger}.
   *
   * @param name action name, matched case-insensitively
   * @param commandSupplier builds a fresh command each time the action fires
   * @param locksRotation true if the action aims the robot, taking the heading away from the agent
   */
  public AIControlBridge registerAction(
      String name, Supplier<Command> commandSupplier, boolean locksRotation) {
    actions.put(name.toUpperCase(), new RegisteredAction(commandSupplier, locksRotation));
    publishAvailableActions();
    return this;
  }

  /** Registers an action that leaves the robot's heading under the agent's control. */
  public AIControlBridge registerAction(String name, Supplier<Command> commandSupplier) {
    return registerAction(name, commandSupplier, false);
  }

  /** True while a pathfinding command from this bridge is running. */
  public boolean isNavigating() {
    return navigationCommand != null
        && CommandScheduler.getInstance().isScheduled(navigationCommand);
  }

  /** True while a mechanism action from this bridge is running. */
  public boolean isActionRunning() {
    return actionCommand != null && CommandScheduler.getInstance().isScheduled(actionCommand);
  }

  /** True while the bridge owns the drivetrain, i.e. it is navigating or aiming. */
  public boolean isBusy() {
    return isNavigating() || isActionRunning();
  }

  @Override
  public void periodic() {
    for (double[] value : targetPoseSub.readQueueValues()) {
      handleTargetPose(value);
    }
    for (String value : actionTriggerSub.readQueueValues()) {
      handleAction(value);
    }

    boolean navigating = isNavigating();
    boolean actionRunning = isActionRunning();

    // In simulation nothing else keeps the robot enabled - there is no driver station - so a
    // command from the agent would be scheduled and immediately cancelled. Hold the enable for as
    // long as the bridge is driving. No-op on a real robot.
    if (navigating || actionRunning) {
      enableForCommandInSim();
    }

    if (!navigating && navigationCommand != null) {
      navigationCommand = null;
      if (activeTarget != null && !atPose(poseSupplier.get(), activeTarget)) {
        // The path ran out of time without arriving - say so instead of silently going idle, so
        // the agent can pick a different target rather than waiting forever.
        reportError(
            String.format(
                "could not reach (%.2f, %.2f) - path timed out, or the pose is blocked or inside an"
                    + " obstacle",
                activeTarget.getX(), activeTarget.getY()));
      }
    }
    if (!actionRunning && actionCommand != null) {
      actionCommand = null;
      setRotationLocked(false);
    }
    Pose2d pose = poseSupplier.get();
    boolean atTarget = activeTarget != null && atPose(pose, activeTarget);

    if (wasBusy && !navigating && !actionRunning) {
      statusPub.set(atTarget ? "AT TARGET" : "IDLE");
    }
    wasBusy = navigating || actionRunning;

    robotPosePub.set(toArray(pose));
    navigatingPub.set(navigating);
    actionRunningPub.set(actionRunning);
    atTargetPub.set(atTarget);

    Logger.recordOutput("AIControl/Navigating", navigating);
    Logger.recordOutput("AIControl/ActionRunning", actionRunning);
    Logger.recordOutput("AIControl/RotationLocked", rotationLocked);
    Logger.recordOutput("AIControl/LastAction", lastAction);
    if (activeTarget != null) {
      Logger.recordOutput("AIControl/ActiveTarget", activeTarget);
    }
  }

  /** Cancels the running path and action. */
  public void cancel() {
    cancelNavigation();
    cancelAction();
    statusPub.set("STOPPED");
  }

  private void cancelNavigation() {
    if (navigationCommand != null) {
      CommandScheduler.getInstance().cancel(navigationCommand);
      navigationCommand = null;
    }
    navigatingPub.set(false);
  }

  private void cancelAction() {
    if (actionCommand != null) {
      CommandScheduler.getInstance().cancel(actionCommand);
      actionCommand = null;
    }
    setRotationLocked(false);
    actionRunningPub.set(false);
  }

  private void handleTargetPose(double[] value) {
    if (value.length == 0) {
      // Neutral/startup value, nothing requested.
      return;
    }
    if (value.length != 3) {
      reportError("TargetPose needs [x, y, headingDegrees], got " + value.length + " values");
      return;
    }
    if (!AutoBuilder.isConfigured()) {
      reportError("PathPlanner AutoBuilder is not configured");
      return;
    }

    Pose2d target = new Pose2d(value[0], value[1], Rotation2d.fromDegrees(value[2]));
    startNavigation(target);
    statusPub.set(
        String.format(
            "PATHFINDING to (%.2f, %.2f, %.1f deg)%s",
            target.getX(),
            target.getY(),
            target.getRotation().getDegrees(),
            rotationLocked ? " - heading owned by the shooter" : ""));
  }

  /**
   * Starts (or restarts) the pathfinding command. PathPlanner's own pathfinding command ends when
   * its trajectory timer runs out, whether or not the robot actually arrived, so it is repeated
   * until the pose is reached - each repeat replans from wherever the robot really is.
   */
  private void startNavigation(Pose2d target) {
    cancelNavigation();
    enableForCommandInSim();

    activeTarget = target;
    navigationCommand =
        Commands.sequence(
                // Cross the field with PathPlanner's pathfinder. Its command ends when its
                // trajectory timer runs out rather than when the robot arrives, so repeat it -
                // every repeat replans from wherever the robot really is.
                AutoBuilder.pathfindToPose(target, constraints())
                    .repeatedly()
                    .until(() -> withinFinalApproach(target)),
                finalApproach(target))
            .withTimeout(NAVIGATION_TIMEOUT_SECONDS)
            .withName("AIControl/PathfindToPose");
    CommandScheduler.getInstance().schedule(navigationCommand);

    activeTargetPub.set(toArray(target));
    navigatingPub.set(true);
    Logger.recordOutput("AIControl/ActiveTarget", target);
  }

  /** Turns the robot in place onto the shooter's heading. Occupies the drive slot. */
  private void startAimInPlace() {
    cancelNavigation();
    activeTarget = null;
    activeTargetPub.set(new double[0]);
    enableForCommandInSim();
    navigationCommand =
        Commands.run(
                () -> velocityConsumer.accept(new ChassisSpeeds(0.0, 0.0, aimOmegaRadPerSec())),
                driveSubsystem)
            .until(() -> !rotationLocked)
            .finallyDo(() -> velocityConsumer.accept(new ChassisSpeeds()))
            .withName("AIControl/AimInPlace");
    CommandScheduler.getInstance().schedule(navigationCommand);
    navigatingPub.set(true);
  }

  /** Holds PathPlanner's holonomic controller on the target pose until the robot settles on it. */
  private Command finalApproach(Pose2d target) {
    PathPlannerTrajectoryState goal = new PathPlannerTrajectoryState();
    goal.pose = target;
    goal.heading = target.getRotation();
    goal.fieldSpeeds = new ChassisSpeeds();
    goal.linearVelocity = 0.0;

    return Commands.runOnce(() -> approachController.reset(poseSupplier.get(), new ChassisSpeeds()))
        .andThen(
            Commands.run(
                () ->
                    velocityConsumer.accept(
                        approachController.calculateRobotRelativeSpeeds(poseSupplier.get(), goal)),
                driveSubsystem))
        .until(() -> atPose(poseSupplier.get(), target))
        .finallyDo(() -> velocityConsumer.accept(new ChassisSpeeds()))
        .withName("AIControl/FinalApproach");
  }

  /**
   * Pathfinding limits for the next path, from {@code MaxSpeed} / {@code MaxAccel}. Clamped, so a
   * bad number from the agent slows the robot down rather than launching it across the field.
   */
  private PathConstraints constraints() {
    double maxSpeed =
        MathUtil.clamp(
            maxSpeedSub.get(DEFAULT_MAX_SPEED_MPS), MIN_MAX_SPEED_MPS, MAX_MAX_SPEED_MPS);
    double maxAccel =
        MathUtil.clamp(
            maxAccelSub.get(DEFAULT_MAX_ACCEL_MPS2), MIN_MAX_ACCEL_MPS2, MAX_MAX_ACCEL_MPS2);
    return new PathConstraints(
        maxSpeed,
        maxAccel,
        Units.degreesToRadians(MAX_ANGULAR_SPEED_DEG_PER_SEC),
        Units.degreesToRadians(MAX_ANGULAR_ACCEL_DEG_PER_SEC2));
  }

  private boolean withinFinalApproach(Pose2d target) {
    return poseSupplier.get().getTranslation().getDistance(target.getTranslation())
        <= FINAL_APPROACH_RADIUS_METERS;
  }

  private void handleAction(String rawName) {
    if (rawName == null || rawName.isBlank()) {
      // Neutral/startup value, nothing requested.
      return;
    }

    String name = rawName.trim().toUpperCase();
    lastAction = name;
    lastActionPub.set(name);

    if (name.equals(STOP_ACTION)) {
      cancel();
      return;
    }

    RegisteredAction action = actions.get(name);
    if (action == null) {
      reportError("unknown action '" + rawName + "'");
      return;
    }

    cancelAction();
    enableForCommandInSim();
    actionCommand = action.commandSupplier().get();
    if (actionCommand == null) {
      reportError("action '" + name + "' produced no command");
      return;
    }
    CommandScheduler.getInstance().schedule(actionCommand);
    actionRunningPub.set(true);
    setRotationLocked(action.locksRotation());

    if (action.locksRotation() && !isNavigating()) {
      // Nothing is driving, so there is no path whose rotation we can take over. Turn in place onto
      // the launch heading instead - rotation only, the robot does not travel.
      startAimInPlace();
    }

    statusPub.set(
        "RUNNING " + name + (action.locksRotation() ? " - shooter owns the heading" : ""));
  }

  /**
   * Hands the robot's heading to the shooter, by overriding the rotation target PathPlanner is
   * driving to. Movement still comes entirely from PathPlanner.
   */
  private void setRotationLocked(boolean locked) {
    if (locked == rotationLocked) {
      return;
    }
    rotationLocked = locked;
    rotationLockedPub.set(locked);
    if (locked) {
      aimController.reset();
      PPHolonomicDriveController.overrideRotationFeedback(this::aimOmegaRadPerSec);
    } else {
      PPHolonomicDriveController.clearRotationFeedbackOverride();
    }
  }

  private Optional<Rotation2d> aimHeading() {
    return aimHeadingSupplier.get();
  }

  /** Rotation feedback (rad/s) that turns the robot onto the shooter's heading. */
  private double aimOmegaRadPerSec() {
    Optional<Rotation2d> target = aimHeading();
    if (target.isEmpty()) {
      return 0.0;
    }
    double omega =
        aimController.calculate(
            poseSupplier.get().getRotation().getRadians(), target.get().getRadians());
    return MathUtil.clamp(omega, -MAX_AIM_OMEGA_RAD_PER_SEC, MAX_AIM_OMEGA_RAD_PER_SEC);
  }

  /**
   * Simulation convenience: an AI operator has no driver station, so a command arriving while the
   * robot is disabled would be dropped on the next loop. In simulation we enable teleop for it.
   * No-op on a real robot, where the driver station is the only thing that may enable the robot.
   */
  private void enableForCommandInSim() {
    if (!RobotBase.isSimulation()) {
      return;
    }
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /** Publishes an error to both {@code Status} and the sticky {@code LastError} topic. */
  private void reportError(String message) {
    statusPub.set("ERROR: " + message);
    lastErrorPub.set("ERROR: " + message);
    Logger.recordOutput("AIControl/LastError", message);
  }

  private void publishAvailableActions() {
    String[] names = new String[actions.size() + 1];
    names[0] = STOP_ACTION;
    int i = 1;
    for (String name : actions.keySet()) {
      names[i++] = name;
    }
    availableActionsPub.set(names);
  }

  /**
   * Arrival test. While the shooter owns the heading the requested heading is unreachable by
   * definition, so only the translation counts - otherwise a path would keep replanning forever
   * chasing a heading it is not allowed to have.
   */
  private boolean atPose(Pose2d current, Pose2d target) {
    if (current.getTranslation().getDistance(target.getTranslation()) > POSE_TOLERANCE_METERS) {
      return false;
    }
    return rotationLocked
        || Math.abs(current.getRotation().minus(target.getRotation()).getDegrees())
            <= HEADING_TOLERANCE_DEGREES;
  }

  private static double[] toArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
  }
}
