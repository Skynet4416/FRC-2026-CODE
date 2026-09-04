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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
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
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
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
 *       {@code "SHOOT_ON_THE_MOVE"}, {@code "ALIGN_HUB"}. {@code "STOP"} cancels
 *       everything.</td></tr>
 *   <tr><td>{@code ResetSimulation}</td><td>{@code boolean}</td>
 *       <td>Writing {@code true} cancels everything and puts the simulated match back to its
 *       starting state. It is deliberately not an action, so it never reaches the agent's tool
 *       list: restarting the match is the human's button, not the model's.</td></tr>
 * </table>
 *
 * <p><b>Navigation goes through PathPlanner; shooting goes through the robot's own launch
 * drive.</b> A {@code TargetPose} is a pathfinding command. A shooting action instead runs {@code
 * DriveCommands.joystickDriveWhileLaunching} - the command behind the driver's right trigger -
 * which aims, leads the target for the robot's motion, and limits translation to what the shot can
 * tolerate. That is also what makes shooting on the move real rather than approximate.
 *
 * <p><b>The agent does not own the drivetrain while a shooting action runs</b>, and the heading in
 * {@code TargetPose} no longer applies; {@code /AIControl/RotationLocked} says when that is the
 * case.
 *
 * <p><b>Shots are taken from our alliance zone, behind the hub.</b> Past the hub the launch
 * solution becomes a lob pass back into our own zone instead of a hub shot, so a shooting action
 * triggered from the neutral zone drives into the alliance zone first and shoots when it arrives.
 * {@code /AIControl/InShootingZone} reports where the robot stands.
 *
 * <p>Read-only feedback: {@code RobotPose}, {@code Navigating}, {@code ActionRunning}, {@code
 * RotationLocked}, {@code InShootingZone}, {@code AtTarget}, {@code ActiveTarget}, {@code Status},
 * {@code LastAction}, {@code LastError}, {@code AvailableActions}, {@code HeadingUnits} and {@code
 * Notes}. Every topic is created in the constructor, so they are all there the moment the robot
 * code starts.
 */
public class AIControlBridge extends SubsystemBase {
  /** NetworkTables table that holds the whole API. */
  public static final String TABLE_NAME = "AIControl";

  /** Cancels the running path and action. Always available, never registered by the caller. */
  public static final String STOP_ACTION = "STOP";

  private static final String NOTES =
      "Write /AIControl/TargetPose = [x_m, y_m, heading_deg] to drive somewhere (PathPlanner "
          + "pathfinding). Write /AIControl/ActionTrigger = an entry of AvailableActions to run a "
          + "mechanism. Hub shots only score from our own alliance zone, behind the hub - past the "
          + "hub the launch solution turns into a lob pass back into our zone - so SHOOT_FUEL and "
          + "SHOOT_ON_THE_MOVE drive into that zone first if the robot is not already there "
          + "(InShootingZone says which). While a shooting action runs it owns the drivetrain: it "
          + "aims, leads the target for the robot's own motion, and ignores the heading you asked "
          + "for, which is what RotationLocked reports. Set /AIControl/MaxSpeed (m/s) and "
          + "/AIControl/MaxAccel (m/s^2) before a TargetPose to drive gently or to build up speed. "
          + "The bumps beside each hub are only crossable with a running start - back off a couple "
          + "of metres and cross at 4+ m/s, or take the flat trench lanes along either side wall "
          + "instead. Actions on different mechanisms run at the same time: trigger INTAKE and "
          + "then drive, and the intake stays down for the whole path. Only actions that need the "
          + "same mechanism replace each other, and a shooting action or a new TargetPose takes "
          + "the drivetrain back from whatever had it. RunningActions lists what is running right "
          + "now. STOP cancels everything.";

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

  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<ChassisSpeeds> velocityConsumer;
  private final Subsystem driveSubsystem;
  private final Map<String, RegisteredAction> actions = new LinkedHashMap<>();

  /** Named field positions, published as JSON so an agent can be told where things are. */
  private final Map<String, double[]> landmarks = new LinkedHashMap<>();

  /** True while the robot is standing where a shot counts as a hub shot. */
  private BooleanSupplier inShootingZone = () -> true;

  /** Pose to drive to before shooting, when the robot is not in the shooting zone. */
  private Supplier<Pose2d> shootingSpot = null;

  /** Puts the simulated match back to its starting state. No-op until something registers one. */
  private Runnable simulationReset = null;

  // Inputs (written by the AI agent)
  private final DoubleArraySubscriber targetPoseSub;
  private final StringSubscriber actionTriggerSub;
  private final BooleanSubscriber resetSimulationSub;

  // Input topics are published once so they exist (with a neutral value) at startup.
  private final DoubleArrayPublisher targetPosePub;
  private final StringPublisher actionTriggerPub;
  private final BooleanPublisher resetSimulationPub;
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
  private final BooleanPublisher inShootingZonePub;
  private final BooleanPublisher atTargetPub;
  private final StringPublisher statusPub;
  private final StringPublisher lastActionPub;
  private final StringPublisher lastErrorPub;
  private final StringArrayPublisher availableActionsPub;
  private final StringArrayPublisher runningActionsPub;
  private final StringPublisher headingUnitsPub;
  private final StringPublisher notesPub;
  private final StringPublisher landmarksPub;
  private final DoubleArrayPublisher fieldSizePub;

  private Command navigationCommand = null;

  /**
   * The actions running right now, by name. Mechanisms that do not share a subsystem run at the
   * same time - intaking while a path drives over the fuel is the whole point - so this is a map
   * rather than the one slot it used to be. Ordered, so status text reads in the order the actions
   * were triggered.
   */
  private final Map<String, Command> runningActions = new LinkedHashMap<>();

  private Pose2d activeTarget = null;
  private String lastAction = "";
  private boolean rotationLocked = false;

  /** Name of a shooting action that is still driving into the alliance zone, else null. */
  private String approachingShootAction = null;

  // PathPlanner's holonomic controller, used for the final approach onto the requested pose.
  private final PPHolonomicDriveController approachController =
      new PPHolonomicDriveController(
          new PIDConstants(3.0, 0.0, 0.0), new PIDConstants(3.0, 0.0, 0.0));
  private boolean wasBusy = false;

  /** An action the agent can trigger by name. */
  private record RegisteredAction(Supplier<Command> commandSupplier, boolean ownsDrive) {}

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

    var resetSimulationTopic = table.getBooleanTopic("ResetSimulation");
    resetSimulationPub = resetSimulationTopic.publish();
    resetSimulationPub.set(false);
    resetSimulationSub =
        resetSimulationTopic.subscribe(
            false, PubSubOption.sendAll(true), PubSubOption.keepDuplicates(true));

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
    inShootingZonePub = table.getBooleanTopic("InShootingZone").publish();
    atTargetPub = table.getBooleanTopic("AtTarget").publish();
    statusPub = table.getStringTopic("Status").publish();
    lastActionPub = table.getStringTopic("LastAction").publish();
    lastErrorPub = table.getStringTopic("LastError").publish();
    availableActionsPub = table.getStringArrayTopic("AvailableActions").publish();
    runningActionsPub = table.getStringArrayTopic("RunningActions").publish();
    headingUnitsPub = table.getStringTopic("HeadingUnits").publish();
    notesPub = table.getStringTopic("Notes").publish();
    landmarksPub = table.getStringTopic("Landmarks").publish();
    fieldSizePub = table.getDoubleArrayTopic("FieldSize").publish();

    robotPosePub.set(toArray(poseSupplier.get()));
    activeTargetPub.set(new double[0]);
    navigatingPub.set(false);
    actionRunningPub.set(false);
    rotationLockedPub.set(false);
    inShootingZonePub.set(inShootingZone.getAsBoolean());
    atTargetPub.set(false);
    statusPub.set("IDLE");
    lastActionPub.set("");
    lastErrorPub.set("");
    runningActionsPub.set(new String[0]);
    headingUnitsPub.set("degrees");
    notesPub.set(NOTES);
    fieldSizePub.set(new double[0]);
    publishAvailableActions();
    publishLandmarks();
  }

  /**
   * Teaches the bridge where shots are legal.
   *
   * <p>A shot only counts when it is taken from our alliance zone, behind the hub; from anywhere
   * past the hub the launch solution becomes a lob pass back into our own zone instead of a hub
   * shot. So when a shooting action is triggered from outside that zone, the bridge drives to
   * {@code spot} first and shoots when it gets there.
   */
  public AIControlBridge setShootingZone(BooleanSupplier inZone, Supplier<Pose2d> spot) {
    this.inShootingZone = inZone;
    this.shootingSpot = spot;
    return this;
  }

  /**
   * Registers a named mechanism action the agent can trigger by writing its name to {@code
   * /AIControl/ActionTrigger}.
   *
   * @param name action name, matched case-insensitively
   * @param commandSupplier builds a fresh command each time the action fires
   * @param ownsDrive true for a shooting action: it drives the robot itself, on the launch drive,
   *     so it cancels any path and the agent's heading request no longer applies
   */
  public AIControlBridge registerAction(
      String name, Supplier<Command> commandSupplier, boolean ownsDrive) {
    actions.put(name.toUpperCase(), new RegisteredAction(commandSupplier, ownsDrive));
    publishAvailableActions();
    return this;
  }

  /** Registers an action that leaves the drivetrain under the agent's control. */
  public AIControlBridge registerAction(String name, Supplier<Command> commandSupplier) {
    return registerAction(name, commandSupplier, false);
  }

  /**
   * Teaches the bridge how to put the simulated match back to its starting state, so a dashboard
   * can restart a demo without restarting the robot code.
   *
   * <p>This is not a registered action on purpose: actions are published in {@code
   * AvailableActions} and become the agent's tool list, and an agent that can wipe the field
   * mid-instruction is an agent that can hide its own mistakes. Restarting is the human's button,
   * driven by writing {@code true} to {@code /AIControl/ResetSimulation}.
   *
   * @param reset run on the main robot thread; expected to no-op away from simulation
   */
  public AIControlBridge onSimulationReset(Runnable reset) {
    this.simulationReset = reset;
    return this;
  }

  /** True while a pathfinding command from this bridge is running. */
  public boolean isNavigating() {
    return navigationCommand != null
        && CommandScheduler.getInstance().isScheduled(navigationCommand);
  }

  /** True while at least one mechanism action from this bridge is running. */
  public boolean isActionRunning() {
    return !runningActions.isEmpty();
  }

  /** Names of the actions running right now, in the order they were triggered. */
  public String[] runningActions() {
    return runningActions.keySet().toArray(new String[0]);
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
    for (boolean value : resetSimulationSub.readQueueValues()) {
      if (value) {
        handleResetSimulation();
      }
    }

    pruneFinishedActions();

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
    Pose2d pose = poseSupplier.get();
    boolean atTarget = activeTarget != null && atPose(pose, activeTarget);

    // The approach into the alliance zone is part of the shooting action; say so when it ends and
    // the shot itself starts.
    if (approachingShootAction != null
        && (!runningActions.containsKey(approachingShootAction) || inShootingZone.getAsBoolean())) {
      if (runningActions.containsKey(approachingShootAction)) {
        statusPub.set("RUNNING " + approachingShootAction + " - shooter owns the drivetrain");
      }
      approachingShootAction = null;
    }

    if (wasBusy && !navigating && !actionRunning) {
      statusPub.set(atTarget ? "AT TARGET" : "IDLE");
    }
    wasBusy = navigating || actionRunning;

    robotPosePub.set(toArray(pose));
    navigatingPub.set(navigating);
    actionRunningPub.set(actionRunning);
    atTargetPub.set(atTarget);
    inShootingZonePub.set(inShootingZone.getAsBoolean());
    runningActionsPub.set(runningActions());

    Logger.recordOutput("AIControl/Navigating", navigating);
    Logger.recordOutput("AIControl/ActionRunning", actionRunning);
    Logger.recordOutput("AIControl/RunningActions", runningActions());
    Logger.recordOutput("AIControl/ShooterOwnsDrivetrain", rotationLocked);
    Logger.recordOutput("AIControl/InShootingZone", inShootingZone.getAsBoolean());
    Logger.recordOutput("AIControl/LastAction", lastAction);
    if (activeTarget != null) {
      Logger.recordOutput("AIControl/ActiveTarget", activeTarget);
    }
  }

  /** Cancels the running path and every running action. */
  public void cancel() {
    cancelNavigation();
    cancelActions();
    statusPub.set("STOPPED");
  }

  private void cancelNavigation() {
    if (navigationCommand != null) {
      CommandScheduler.getInstance().cancel(navigationCommand);
      navigationCommand = null;
    }
    navigatingPub.set(false);
  }

  /** Cancels every running action. */
  private void cancelActions() {
    approachingShootAction = null;
    for (Command command : runningActions.values()) {
      CommandScheduler.getInstance().cancel(command);
    }
    runningActions.clear();
    setDriveOwnedByShooter(false);
    actionRunningPub.set(false);
    runningActionsPub.set(new String[0]);
  }

  /**
   * Cancels the running actions that cannot coexist with {@code command}: the ones that need a
   * subsystem it needs. The scheduler enforces this by itself when the new command is scheduled;
   * doing it here keeps {@code runningActions} honest about it in the same loop, and keeps a
   * cancelled action out of the status text.
   *
   * @param except an action name to leave alone, or null
   */
  private void cancelConflictingActions(Command command, String except) {
    var required = command.getRequirements();
    runningActions
        .entrySet()
        .removeIf(
            entry -> {
              if (entry.getKey().equals(except)
                  || Collections.disjoint(entry.getValue().getRequirements(), required)) {
                return false;
              }
              CommandScheduler.getInstance().cancel(entry.getValue());
              return true;
            });
  }

  /**
   * Drops the actions whose commands have ended, so the running set is only what is really live.
   */
  private void pruneFinishedActions() {
    boolean removed =
        runningActions
            .values()
            .removeIf(command -> !CommandScheduler.getInstance().isScheduled(command));
    if (!removed) {
      return;
    }
    if (runningActions.isEmpty()) {
      approachingShootAction = null;
    }
    setDriveOwnedByShooter(anyRunningActionOwnsDrive());
  }

  /** True while one of the running actions is a shooting action, which drives the robot itself. */
  private boolean anyRunningActionOwnsDrive() {
    return runningActions.keySet().stream()
        .map(actions::get)
        .anyMatch(action -> action != null && action.ownsDrive());
  }

  /** Cancels everything the bridge is doing, then puts the simulated match back to the start. */
  private void handleResetSimulation() {
    cancel();
    activeTarget = null;
    activeTargetPub.set(new double[0]);
    atTargetPub.set(false);
    lastAction = "";
    lastActionPub.set("");
    lastErrorPub.set("");
    if (simulationReset == null) {
      reportError("nothing registered to reset the simulation");
      return;
    }
    simulationReset.run();
    statusPub.set("RESET");
    // Nothing is running any more, so the end-of-work branch below must not overwrite this
    // with IDLE on the way out of the same loop.
    wasBusy = false;
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
    navigationCommand = pathfindCommand(target);
    // Taking the drivetrain back ends a shooting action, but not an intake running beside it.
    cancelConflictingActions(navigationCommand, null);
    setDriveOwnedByShooter(anyRunningActionOwnsDrive());
    CommandScheduler.getInstance().schedule(navigationCommand);

    activeTargetPub.set(toArray(target));
    navigatingPub.set(true);
    Logger.recordOutput("AIControl/ActiveTarget", target);
  }

  /**
   * Drives to a pose: PathPlanner's pathfinder for the distance, its controller for the last bit.
   */
  private Command pathfindCommand(Pose2d target) {
    return Commands.sequence(
            // Cross the field with PathPlanner's pathfinder. Its command ends when its trajectory
            // timer runs out rather than when the robot arrives, so repeat it - every repeat
            // replans from wherever the robot really is.
            AutoBuilder.pathfindToPose(target, constraints())
                .repeatedly()
                .until(() -> withinFinalApproach(target)),
            finalApproach(target))
        .withTimeout(NAVIGATION_TIMEOUT_SECONDS)
        .withName("AIControl/PathfindToPose");
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

    enableForCommandInSim();

    Command command = action.commandSupplier().get();
    if (command == null) {
      reportError("action '" + name + "' produced no command");
      return;
    }

    // Retriggering an action restarts it rather than stacking a second copy on the mechanism.
    Command previous = runningActions.remove(name);
    if (previous != null) {
      CommandScheduler.getInstance().cancel(previous);
    }

    boolean approaching = false;
    if (action.ownsDrive()) {
      // A shooting action drives itself, on the launch drive, so no path may be running under it.
      cancelNavigation();
      if (shootingSpot != null && !inShootingZone.getAsBoolean()) {
        // Out of the alliance zone, where a shot is a lob pass rather than a hub shot. Drive in
        // first, then shoot.
        Pose2d spot = shootingSpot.get();
        activeTarget = spot;
        activeTargetPub.set(toArray(spot));
        command = pathfindCommand(spot).andThen(command);
        approaching = true;
      }
    }

    // Only the actions that need a subsystem this one needs have to give way; everything else keeps
    // running alongside it, so the intake can stay down while the robot drives or shoots.
    cancelConflictingActions(command, name);

    runningActions.put(name, command);
    CommandScheduler.getInstance().schedule(command);
    actionRunningPub.set(true);
    runningActionsPub.set(runningActions());
    setDriveOwnedByShooter(anyRunningActionOwnsDrive());

    if (approaching) {
      Pose2d spot = activeTarget;
      approachingShootAction = name;
      statusPub.set(
          String.format(
              "MOVING INTO THE ALLIANCE ZONE (%.2f, %.2f), then %s",
              spot.getX(), spot.getY(), name));
    } else {
      statusPub.set(
          "RUNNING "
              + String.join(" + ", runningActions())
              + (anyRunningActionOwnsDrive() ? " - shooter owns the drivetrain" : ""));
    }
  }

  /** Records that a shooting action has taken the drivetrain (aim, lead and all) from the agent. */
  private void setDriveOwnedByShooter(boolean owned) {
    if (owned == rotationLocked) {
      return;
    }
    rotationLocked = owned;
    rotationLockedPub.set(owned);
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

  /**
   * Names a spot on the field, so an agent can be told where things are instead of guessing
   * coordinates. Landmarks are published to {@code /AIControl/Landmarks} as a JSON object of {@code
   * name: [x_m, y_m, heading_deg]}, ready to drop into a prompt.
   */
  public AIControlBridge registerLandmark(String name, Pose2d pose) {
    landmarks.put(name, toArray(pose));
    publishLandmarks();
    return this;
  }

  /** Publishes the field size to {@code /AIControl/FieldSize} as {@code [length_m, width_m]}. */
  public AIControlBridge setFieldSize(double lengthMeters, double widthMeters) {
    fieldSizePub.set(new double[] {lengthMeters, widthMeters});
    return this;
  }

  private void publishLandmarks() {
    StringBuilder json = new StringBuilder("{");
    boolean first = true;
    for (Map.Entry<String, double[]> entry : landmarks.entrySet()) {
      double[] pose = entry.getValue();
      json.append(first ? "" : ",")
          .append(
              String.format(
                  java.util.Locale.US,
                  "\"%s\":[%.3f,%.3f,%.1f]",
                  entry.getKey(),
                  pose[0],
                  pose[1],
                  pose[2]));
      first = false;
    }
    landmarksPub.set(json.append("}").toString());
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
