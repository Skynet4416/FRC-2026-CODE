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
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AIGameBrief;
import frc.robot.commands.CollectFuelCommand;
import frc.robot.util.HubShiftUtil;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
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
 *       running start at a bump, down to creep into position. Clamped down (never up) to {@code
 *       COLLECT_SPEED_MPS} while the intake is down.</td></tr>
 *   <tr><td>{@code ActionTrigger}</td><td>{@code String}</td>
 *       <td>Name of a registered mechanism action, e.g. {@code "INTAKE"}, {@code "STOW_INTAKE"},
 *       {@code "COLLECT_FUEL"}, {@code "GRAB_FUEL"}, {@code "SHOOT_FUEL"}, {@code
 *       "SHOOT_ON_THE_MOVE"}, {@code "ALIGN_HUB"}, {@code "AUTO_FOLD_ON"}, {@code
 *       "AUTO_FOLD_OFF"}. {@code "STOP"} cancels everything.</td></tr>
 *   <tr><td>{@code CollectTarget}</td><td>{@code double}</td>
 *       <td>How many fuel {@code COLLECT_FUEL}/{@code GRAB_FUEL} should gather before stopping on
 *       their own. Defaults to 10.</td></tr>
 *   <tr><td>{@code AutoFoldIntake}</td><td>{@code boolean}</td>
 *       <td>Defaults to {@code true}. While true the bridge folds the intake by itself for
 *       clearance crossing a trench or a bump, and lowers it again once clear, without losing
 *       track of what the agent actually asked for. While false the intake only ever moves when
 *       an action tells it to.</td></tr>
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
 * <p><b>The agent does not own the drivetrain while a shooting action or {@code COLLECT_FUEL}/
 * {@code GRAB_FUEL} runs</b>, and the heading in {@code TargetPose} no longer applies; {@code
 * /AIControl/RotationLocked} says when that is the case.
 *
 * <p><b>Shots are taken from our alliance zone, behind the hub.</b> Past the hub the launch
 * solution becomes a lob pass back into our own zone instead of a hub shot, so a shooting action
 * triggered from the neutral zone drives into the alliance zone first and shoots when it arrives.
 * {@code /AIControl/InShootingZone} reports where the robot stands. {@code COLLECT_FUEL} and
 * {@code GRAB_FUEL} are exempt from this pre-approach - they drive wherever the fuel actually is.
 *
 * <p><b>The intake latches.</b> {@code INTAKE} lowers it and leaves it there, rollers running,
 * until {@code STOW_INTAKE}, {@code STOP}, or the hopper filling up ends it - it never times out.
 * {@code COLLECT_FUEL}/{@code GRAB_FUEL} drive a slow sweep with the intake down the whole time.
 * See {@code Fuel}, {@code FuelPositions}, {@code FuelOnBoard}, {@code Zones}, {@code
 * IntakePolicy}, {@code HubState} and {@code GameBrief} below for what the agent gets to reason
 * about all of this with.
 *
 * <p>Read-only feedback: {@code RobotPose}, {@code Navigating}, {@code ActionRunning}, {@code
 * RotationLocked}, {@code InShootingZone}, {@code AtTarget}, {@code ActiveTarget}, {@code Status},
 * {@code LastAction}, {@code LastError}, {@code AvailableActions}, {@code HeadingUnits}, {@code
 * Notes}, {@code Landmarks}, {@code Fuel} (String JSON; ground-truth fuel awareness, {@code
 * available:false} off simulation), {@code FuelPositions} (flat {@code [x,y,x,y,...]} of every
 * fuel on the ground), {@code FuelOnBoard}, {@code Zones} (String JSON of named field
 * rectangles), {@code IntakePolicy} (String JSON; what the auto-fold is doing and why), {@code
 * HubState} (String JSON; is our hub active right now) and {@code GameBrief} (String; the REBUILT
 * rules). Every topic is created in the constructor, so they are all there the moment the robot
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
          + "mechanism. "
          // --- Intake: this is the fix for "the operator keeps folding it and doesn't know it
          // can drive while intaking" - say it plainly and repeat the load-bearing facts.
          + "INTAKE LATCHES: it lowers the intake and leaves it down, rollers running, until "
          + "STOW_INTAKE or STOP - it does not time out and you do not need to re-trigger it. You "
          + "collect fuel by DRIVING with the intake down, not by standing still: trigger INTAKE "
          + "once, then drive over the fuel, and it keeps collecting the whole way. Keep the "
          + "intake down as your normal state - you lose nothing by having it out in the open "
          + "field, you collect anything you happen to drive over, and the robot automatically "
          + "folds it only for the trench and the bumps and lowers it again once clear "
          + "(AutoFoldIntake, on by default; AUTO_FOLD_ON/AUTO_FOLD_OFF and IntakePolicy control "
          + "and report it). Do not fold it between pieces - STOW_INTAKE is for when you want it "
          + "up for your own reasons, not routine housekeeping. The intake is on your LEFT flank, "
          + "about half a metre out (see Fuel.pickup), so line fuel up to pass down your left "
          + "side, and the robot clamps you to COLLECT_SPEED_MPS while the intake is down so you "
          + "do not blow past it - Status says when that clamp is active. COLLECT_FUEL is the "
          + "preferred way to gather fuel: trigger it and it plans and drives the whole sweep "
          + "itself, intake down, until CollectTarget pieces are aboard or nothing is left in "
          + "reach; GRAB_FUEL is the same sweep aimed at only the fuel right around you, for when "
          + "you just want what's nearby with no planning. Read Fuel (JSON: on_board, capacity, "
          + "zones, nearest, clusters, pickup) and FuelPositions/FuelOnBoard for where fuel "
          + "actually is - GameBrief and Zones explain the field itself. "
          // --- Hub shift: shots earn nothing while your hub is inactive.
          + "Your hub is not always active - HubState (JSON: active, shift, shift_remaining_s) "
          + "reports the current shift, and while active is false a shot scores nothing, so that "
          + "is the time to collect rather than shoot; HubState.hub_state_ignored says whether "
          + "this robot is currently enforcing that at all. "
          + "Hub shots only score from our own alliance zone, behind the hub - past the "
          + "hub the launch solution turns into a lob pass back into our zone - so SHOOT_FUEL and "
          + "SHOOT_ON_THE_MOVE drive into that zone first if the robot is not already there "
          + "(InShootingZone says which). While a shooting action, COLLECT_FUEL or GRAB_FUEL runs "
          + "it owns the drivetrain: a shooting action aims and leads the target for the robot's "
          + "own motion, COLLECT_FUEL/GRAB_FUEL drive the sweep, and either way the heading you "
          + "asked for is ignored, which is what RotationLocked reports. Set /AIControl/MaxSpeed "
          + "(m/s) and /AIControl/MaxAccel (m/s^2) before a TargetPose to drive gently or to "
          + "build up speed. "
          + "The bumps beside each hub are only crossable with a running start - back off a couple "
          + "of metres and cross at 4+ m/s, or take the flat trench lanes along either side wall "
          + "instead. Actions on different mechanisms run at the same time: trigger INTAKE and "
          + "then drive, and the intake stays down for the whole path. Only actions that need the "
          + "same mechanism replace each other, and a shooting action, COLLECT_FUEL/GRAB_FUEL or a "
          + "new TargetPose takes the drivetrain back from whatever had it. RunningActions lists "
          + "what is running right now. STOP cancels everything, including latched INTAKE (which "
          + "folds it back up) - STOW_INTAKE does the same for INTAKE/COLLECT_FUEL/GRAB_FUEL "
          + "without cancelling anything else.";

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

  /**
   * Pathfinding max speed while the intake is down - crossing the field slowly is what makes
   * "drive with the intake down" actually sweep fuel instead of blowing past it. Only ever lowers
   * the agent's own request, never raises it: an agent that deliberately asked to creep slower
   * keeps creeping slower.
   */
  private static final double COLLECT_SPEED_MPS = 1.5;

  /** Default for {@code /AIControl/CollectTarget}: how much fuel a collect sweep gathers. */
  private static final double DEFAULT_COLLECT_TARGET = 10.0;

  /**
   * {@code Fuel}/{@code FuelPositions} rebuild a string for a few hundred balls every call; at the
   * full 50 Hz robot loop rate that is wasted work for something a language model reads a few
   * times a second at most. Throttled to about 5 Hz instead.
   */
  private static final double FUEL_PUBLISH_PERIOD_SECONDS = 0.2;

  private static final int MAX_FUEL_JSON_NEAREST = 12;
  private static final int MAX_FUEL_JSON_CLUSTERS = 8;
  private static final double FUEL_CLUSTER_RADIUS_M = 1.5;

  // Auto-fold intake (ADDENDUM v2.1): fold only for the trench/bump hazards, and only that far
  // early/late - see updateIntakeAutoFold().
  private static final double FOLD_MARGIN_M = 0.35;
  private static final double FOLD_EXIT_HYSTERESIS_M = 0.15;
  private static final String[] HAZARD_ZONE_NAMES = {
    "left_trench", "right_trench", "left_bump", "right_bump"
  };

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

  // Fuel awareness (setFuelAwareness()) - null-safe defaults so a real robot, or a bridge under
  // construction before RobotContainer finishes wiring it, never throws.
  private Supplier<List<Translation2d>> fieldFuelSupplier = List::of;
  private IntSupplier fuelOnBoardSupplier = () -> 0;
  private IntSupplier fuelCapacitySupplier = () -> 0;
  private BooleanSupplier intakeDownSupplier = () -> false;
  private BooleanSupplier intakeCollectingSupplier = () -> false;
  private boolean fuelAwarenessConfigured = false;

  /** Whether shots are allowed regardless of hub shift - the "Ignore Hub State" chooser. */
  private BooleanSupplier hubStateIgnoredSupplier = () -> true;

  /** Named field rectangles, e.g. {@code left_trench -> [xMin, yMin, xMax, yMax]}. */
  private final Map<String, double[]> zones = new LinkedHashMap<>();

  private final Map<String, String> zoneDescriptions = new LinkedHashMap<>();

  // Intake auto-fold (ADDENDUM v2.1). intakeWantsDown is the agent's actual intent, persisted
  // across an auto-fold so crossing a bump can never permanently lose the intake; the physical
  // mechanism position is decided once, every loop, in updateIntakeAutoFold() - nothing else may
  // call intakeActuator directly, or the two would fight over who owns the solenoid.
  //
  // intakeUnderAiControl only flips true the first time an AI action actually touches the
  // intake (see setIntakeWantsDown). Until then updateIntakeAutoFold() does not actuate anything,
  // so a driver running teleop, an autonomous routine, or the dashboard test buttons are never
  // silently fighting a background loop they have no reason to expect exists. Once the agent
  // does ask for the intake, the bridge owns it for the rest of the match - matching how a
  // shooting action already takes the drivetrain from the driver.
  private boolean intakeWantsDown = false;
  private boolean intakeUnderAiControl = false;
  private Consumer<Boolean> intakeActuator = null;
  private String foldedForHazard = null;

  private double lastFuelPublishTime = -FUEL_PUBLISH_PERIOD_SECONDS;

  // Inputs (written by the AI agent)
  private final DoubleArraySubscriber targetPoseSub;
  private final StringSubscriber actionTriggerSub;
  private final BooleanSubscriber resetSimulationSub;
  private final DoubleSubscriber collectTargetSub;
  private final BooleanSubscriber autoFoldIntakeSub;

  // Input topics are published once so they exist (with a neutral value) at startup.
  private final DoubleArrayPublisher targetPosePub;
  private final StringPublisher actionTriggerPub;
  private final BooleanPublisher resetSimulationPub;
  private final DoublePublisher maxSpeedPub;
  private final DoublePublisher maxAccelPub;
  private final DoubleSubscriber maxSpeedSub;
  private final DoubleSubscriber maxAccelSub;
  private final DoublePublisher collectTargetPub;
  private final BooleanPublisher autoFoldIntakePub;

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
  private final StringPublisher fuelPub;
  private final DoubleArrayPublisher fuelPositionsPub;
  private final DoublePublisher fuelOnBoardPub;
  private final StringPublisher zonesPub;
  private final StringPublisher intakePolicyPub;
  private final StringPublisher hubStatePub;
  private final StringPublisher gameBriefPub;

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

  /**
   * An action the agent can trigger by name.
   *
   * @param approachShootingZoneFirst true for a shooting action: an {@code ownsDrive} action that
   *     only scores from the alliance zone, so the bridge drives it there first. {@code
   *     COLLECT_FUEL}/{@code GRAB_FUEL} also own the drivetrain but drive wherever the fuel is, so
   *     they set this false.
   */
  private record RegisteredAction(
      Supplier<Command> commandSupplier, boolean ownsDrive, boolean approachShootingZoneFirst) {}

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

    var collectTargetTopic = table.getDoubleTopic("CollectTarget");
    collectTargetPub = collectTargetTopic.publish();
    collectTargetPub.set(DEFAULT_COLLECT_TARGET);
    collectTargetSub = collectTargetTopic.subscribe(DEFAULT_COLLECT_TARGET);

    var autoFoldIntakeTopic = table.getBooleanTopic("AutoFoldIntake");
    autoFoldIntakePub = autoFoldIntakeTopic.publish();
    autoFoldIntakePub.set(true);
    autoFoldIntakeSub = autoFoldIntakeTopic.subscribe(true);

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
    fuelPub = table.getStringTopic("Fuel").publish();
    fuelPositionsPub = table.getDoubleArrayTopic("FuelPositions").publish();
    fuelOnBoardPub = table.getDoubleTopic("FuelOnBoard").publish();
    zonesPub = table.getStringTopic("Zones").publish();
    intakePolicyPub = table.getStringTopic("IntakePolicy").publish();
    hubStatePub = table.getStringTopic("HubState").publish();
    gameBriefPub = table.getStringTopic("GameBrief").publish();

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
    fuelPub.set(unavailableFuelJson());
    fuelPositionsPub.set(new double[0]);
    fuelOnBoardPub.set(0);
    zonesPub.set("{}");
    intakePolicyPub.set("{}");
    hubStatePub.set("{}");
    gameBriefPub.set(AIGameBrief.TEXT);
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
   * Wires the bridge to publish {@code HubShiftUtil}'s shift schedule as {@code HubState}, the
   * single biggest gap in an agent's model of the game: without it, a shot that scores nothing
   * because the hub happens to be inactive looks indistinguishable from one that just missed.
   *
   * @param hubStateIgnored whether shots currently count regardless of shift (the "Ignore Hub
   *     State" dashboard chooser) - reported as-is, never sugar-coated, since telling the agent
   *     its shots are blocked when they are not would be worse than saying nothing
   */
  public AIControlBridge setHubStateSource(BooleanSupplier hubStateIgnored) {
    this.hubStateIgnoredSupplier = hubStateIgnored;
    return this;
  }

  /**
   * Wires the bridge to real fuel awareness: where the fuel is, how much the robot is holding, and
   * whether the intake is down and actually collecting. Feeds the {@code Fuel}, {@code
   * FuelPositions} and {@code FuelOnBoard} topics, the intake-down speed clamp, and (together with
   * {@link #setIntakeActuator}) the automatic trench/bump fold.
   *
   * @param fieldFuel every fuel resting on the ground, blue-origin field XY; sim ground truth,
   *     empty on a real robot with nothing to report
   * @param onBoard fuel currently held
   * @param capacity how much fuel the hopper can hold
   * @param intakeDown whether the intake mechanism is physically down right now
   * @param intakeCollecting whether it is down AND actually spinning rollers into fuel
   */
  public AIControlBridge setFuelAwareness(
      Supplier<List<Translation2d>> fieldFuel,
      IntSupplier onBoard,
      IntSupplier capacity,
      BooleanSupplier intakeDown,
      BooleanSupplier intakeCollecting) {
    this.fieldFuelSupplier = fieldFuel;
    this.fuelOnBoardSupplier = onBoard;
    this.fuelCapacitySupplier = capacity;
    this.intakeDownSupplier = intakeDown;
    this.intakeCollectingSupplier = intakeCollecting;
    this.fuelAwarenessConfigured = true;
    return this;
  }

  /**
   * Gives the bridge a way to actually move the intake mechanism, so it can run the automatic
   * trench/bump fold (ADDENDUM v2.1) on its own: raise the intake for clearance the moment the
   * robot enters a hazard rectangle, and put it back exactly where the agent last asked for once
   * clear. This is the ONLY thing that may command the mechanism's position - every action that
   * wants the intake down or up instead calls {@link #setIntakeWantsDown}, so the two can never
   * fight over the solenoid.
   *
   * @param applyLowered physically lowers (true) or raises (false) the intake
   */
  public AIControlBridge setIntakeActuator(Consumer<Boolean> applyLowered) {
    this.intakeActuator = applyLowered;
    return this;
  }

  /**
   * Records what the agent actually wants the intake to do, independent of what the auto-fold is
   * doing to it right now. {@code INTAKE}, {@code COLLECT_FUEL} and {@code GRAB_FUEL} call this
   * with {@code true} (repeatedly, from {@code execute()}, which is cheap and safe); {@code
   * STOW_INTAKE} and {@link #cancel()} (i.e. {@code STOP}) call it with {@code false}. {@link
   * #updateIntakeAutoFold()} is the only thing that reads it.
   *
   * <p>The first call is also what hands the intake mechanism to the bridge - see {@code
   * intakeUnderAiControl}'s javadoc.
   */
  public void setIntakeWantsDown(boolean down) {
    this.intakeWantsDown = down;
    this.intakeUnderAiControl = true;
  }

  /**
   * Turns the automatic trench/bump fold on or off, mirroring {@code /AIControl/AutoFoldIntake} so
   * {@code AUTO_FOLD_ON}/{@code AUTO_FOLD_OFF} can be plain actions instead of asking the agent to
   * poke a raw NetworkTables value.
   */
  public void setAutoFoldIntake(boolean enabled) {
    autoFoldIntakePub.set(enabled);
  }

  /**
   * Registers a named field rectangle for the {@code Zones} topic, so the agent can reason about
   * "am I in the neutral zone" without hard-coded coordinates of its own. Also feeds the automatic
   * trench/bump fold for the four hazard names in {@link #HAZARD_ZONE_NAMES}.
   *
   * @param name zone name, e.g. {@code "left_trench"} - blue-relative and fixed, matching {@link
   *     #registerLandmark}
   * @param what one line of human-readable context for the JSON
   */
  public AIControlBridge registerZone(
      String name, double xMin, double yMin, double xMax, double yMax, String what) {
    zones.put(name, new double[] {xMin, yMin, xMax, yMax});
    zoneDescriptions.put(name, what);
    publishZones();
    return this;
  }

  /**
   * Ends specific running actions immediately, as if each had been cancelled on its own.
   *
   * <p>The scheduler's usual trick for "a new command needing the same subsystem bumps the old
   * one" only works when the old command actually requires that subsystem, and {@code INTAKE} /
   * {@code COLLECT_FUEL} / {@code GRAB_FUEL} deliberately do not require the intake (so its
   * default command keeps spinning the rollers underneath them - see {@code RobotContainer}). An
   * action that means to end them, {@code STOW_INTAKE}, has to say so by name instead.
   *
   * @param names action names to end, matched case-insensitively; a name that is not running is
   *     ignored
   */
  public void endActions(String... names) {
    boolean changed = false;
    for (String name : names) {
      Command running = runningActions.remove(name.toUpperCase());
      if (running != null) {
        CommandScheduler.getInstance().cancel(running);
        changed = true;
      }
    }
    if (!changed) {
      return;
    }
    if (runningActions.isEmpty()) {
      approachingShootAction = null;
    }
    setDriveOwnedByShooter(anyRunningActionOwnsDrive());
    actionRunningPub.set(isActionRunning());
    runningActionsPub.set(runningActions());
  }

  /**
   * Current value of {@code /AIControl/CollectTarget}: how many pieces a collect action should
   * gather before stopping on its own.
   */
  public double getCollectTarget() {
    return collectTargetSub.get(DEFAULT_COLLECT_TARGET);
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
    return registerAction(name, commandSupplier, ownsDrive, ownsDrive);
  }

  /** Registers an action that leaves the drivetrain under the agent's control. */
  public AIControlBridge registerAction(String name, Supplier<Command> commandSupplier) {
    return registerAction(name, commandSupplier, false, false);
  }

  /**
   * Registers a drive-owning action that should NOT get the automatic pre-approach into the
   * shooting spot - e.g. {@code COLLECT_FUEL}/{@code GRAB_FUEL}, which drive wherever the fuel
   * actually is rather than a fixed pre-shot position. Every other {@code ownsDrive} action is
   * assumed to be a shooting action and gets that approach (see {@link #registerAction(String,
   * Supplier, boolean)}).
   */
  public AIControlBridge registerDriveAction(String name, Supplier<Command> commandSupplier) {
    return registerAction(name, commandSupplier, true, false);
  }

  private AIControlBridge registerAction(
      String name,
      Supplier<Command> commandSupplier,
      boolean ownsDrive,
      boolean approachShootingZoneFirst) {
    actions.put(
        name.toUpperCase(),
        new RegisteredAction(commandSupplier, ownsDrive, approachShootingZoneFirst));
    publishAvailableActions();
    return this;
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

    // The intake auto-fold is a control loop, not telemetry, so it runs every loop regardless of
    // the throttle below - a 5 Hz reaction time to "about to hit a bump" is not good enough.
    updateIntakeAutoFold();

    double now = Timer.getFPGATimestamp();
    if (now - lastFuelPublishTime >= FUEL_PUBLISH_PERIOD_SECONDS) {
      lastFuelPublishTime = now;
      publishFuel(pose);
      publishHubState();
    }
  }

  /** Cancels the running path and every running action. */
  public void cancel() {
    cancelNavigation();
    cancelActions();
    // STOP is the one place a superseded INTAKE/COLLECT_FUEL/GRAB_FUEL should fold the intake
    // back up rather than leaving it as the auto-fold found it - everything just stopped, so
    // there is no sweep left to protect by keeping it down. Guarded on intakeUnderAiControl -
    // written directly rather than through setIntakeWantsDown() - so a STOP that has never
    // touched the intake cannot be the thing that grabs it away from the driver.
    if (intakeUnderAiControl) {
      intakeWantsDown = false;
    }
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
    // A fresh match starts with the intake back under manual/autonomous control, exactly like a
    // robot that has just been turned on - the agent re-takes it the moment it asks for INTAKE,
    // COLLECT_FUEL, GRAB_FUEL or STOW_INTAKE again.
    intakeUnderAiControl = false;
    intakeWantsDown = false;
    foldedForHazard = null;
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
            "PATHFINDING to (%.2f, %.2f, %.1f deg)%s%s",
            target.getX(),
            target.getY(),
            target.getRotation().getDegrees(),
            rotationLocked ? " - heading owned by the shooter" : "",
            speedClampNote()));
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
   *
   * <p>While the intake is down this also clamps the speed to {@code COLLECT_SPEED_MPS} - never
   * up, so an agent that deliberately asked for something slower keeps it - because a fast path
   * blows straight through a line of fuel instead of sweeping it. {@link #speedClampNote()} says
   * so in {@code Status} whenever it actually bites.
   */
  private PathConstraints constraints() {
    double maxSpeed =
        MathUtil.clamp(
            maxSpeedSub.get(DEFAULT_MAX_SPEED_MPS), MIN_MAX_SPEED_MPS, MAX_MAX_SPEED_MPS);
    if (intakeDownSupplier.getAsBoolean()) {
      maxSpeed = Math.min(maxSpeed, COLLECT_SPEED_MPS);
    }
    double maxAccel =
        MathUtil.clamp(
            maxAccelSub.get(DEFAULT_MAX_ACCEL_MPS2), MIN_MAX_ACCEL_MPS2, MAX_MAX_ACCEL_MPS2);
    return new PathConstraints(
        maxSpeed,
        maxAccel,
        Units.degreesToRadians(MAX_ANGULAR_SPEED_DEG_PER_SEC),
        Units.degreesToRadians(MAX_ANGULAR_ACCEL_DEG_PER_SEC2));
  }

  /** Suffix for {@code Status} explaining the intake-down speed clamp, or "" when it isn't active. */
  private String speedClampNote() {
    return intakeDownSupplier.getAsBoolean()
        ? String.format(" (clamped to %.1f m/s - intake down)", COLLECT_SPEED_MPS)
        : "";
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
      // A shooting action - or COLLECT_FUEL/GRAB_FUEL - drives itself, so no path may be running
      // under it.
      cancelNavigation();
      if (action.approachShootingZoneFirst()
          && shootingSpot != null
          && !inShootingZone.getAsBoolean()) {
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
   * Rebuilds and publishes {@code Zones} from every rectangle registered so far. Zones are static
   * for the life of the match, so - unlike {@code Fuel} - this runs once per {@link
   * #registerZone} call rather than every loop.
   */
  private void publishZones() {
    StringBuilder json = new StringBuilder("{");
    boolean first = true;
    for (Map.Entry<String, double[]> entry : zones.entrySet()) {
      double[] r = entry.getValue();
      json.append(first ? "" : ",")
          .append('"')
          .append(entry.getKey())
          .append("\":{\"x_min\":")
          .append(num(r[0]))
          .append(",\"y_min\":")
          .append(num(r[1]))
          .append(",\"x_max\":")
          .append(num(r[2]))
          .append(",\"y_max\":")
          .append(num(r[3]))
          .append(",\"what\":\"")
          .append(jsonEscape(zoneDescriptions.getOrDefault(entry.getKey(), "")))
          .append("\"}");
      first = false;
    }
    zonesPub.set(json.append("}").toString());
  }

  /**
   * The {@code Fuel} JSON when there is nothing to report - off simulation, or before {@link
   * #setFuelAwareness} has been wired up. Carries every key the "available: true" shape has, just
   * empty/zeroed, so a consumer never has to special-case a missing field.
   */
  private static String unavailableFuelJson() {
    return "{\"available\":false,\"source\":\"none\",\"vision_source\":\"none\",\"on_board\":0,"
        + "\"capacity\":0,\"intake_down\":false,\"intake_collecting\":false,"
        + "\"collect_target\":"
        + num(DEFAULT_COLLECT_TARGET)
        + ",\"total_on_field\":0,\"zones\":{},\"nearest\":[],\"clusters\":[],"
        + "\"seen_by_camera\":[],\"pickup\":null}";
  }

  /**
   * Rebuilds and publishes {@code Fuel} and {@code FuelPositions}, throttled from {@link
   * #periodic()} to about 5 Hz - a few hundred balls is not free to re-stringify every 20 ms, and
   * nothing reading this is polling faster than that anyway.
   */
  private void publishFuel(Pose2d pose) {
    if (!fuelAwarenessConfigured || !RobotBase.isSimulation()) {
      fuelPub.set(unavailableFuelJson());
      fuelPositionsPub.set(new double[0]);
      fuelOnBoardPub.set(0);
      return;
    }

    List<Translation2d> fuel = fieldFuelSupplier.get();
    if (fuel == null) {
      fuel = List.of();
    }
    Translation2d robot = pose.getTranslation();

    double[] flat = new double[fuel.size() * 2];
    for (int i = 0; i < fuel.size(); i++) {
      flat[i * 2] = fuel.get(i).getX();
      flat[i * 2 + 1] = fuel.get(i).getY();
    }
    fuelPositionsPub.set(flat);

    int onBoard = fuelOnBoardSupplier.getAsInt();
    fuelOnBoardPub.set(onBoard);

    String json =
        "{\"available\":true,\"source\":\"simulation ground truth\","
            + "\"vision_source\":\"sim-ground-truth\",\"on_board\":"
            + onBoard
            + ",\"capacity\":"
            + fuelCapacitySupplier.getAsInt()
            + ",\"intake_down\":"
            + intakeDownSupplier.getAsBoolean()
            + ",\"intake_collecting\":"
            + intakeCollectingSupplier.getAsBoolean()
            + ",\"collect_target\":"
            + num(getCollectTarget())
            + ",\"total_on_field\":"
            + fuel.size()
            + ",\"zones\":"
            + buildZoneFuelJson(fuel, robot)
            + ",\"nearest\":"
            + buildNearestFuelJson(fuel, robot)
            + ",\"clusters\":"
            + buildFuelClustersJson(fuel)
            + ",\"seen_by_camera\":[]"
            + ",\"pickup\":"
            + buildPickupJson(robot, fuel)
            + "}";
    fuelPub.set(json);
  }

  /** Per-zone fuel counts, centroid and nearest piece, for the {@code Fuel.zones} sub-object. */
  private String buildZoneFuelJson(List<Translation2d> fuel, Translation2d robot) {
    Map<String, Integer> counts = new LinkedHashMap<>();
    Map<String, double[]> sums = new LinkedHashMap<>();
    Map<String, Translation2d> nearest = new LinkedHashMap<>();
    Map<String, Double> nearestDistance = new LinkedHashMap<>();
    for (String name : zones.keySet()) {
      counts.put(name, 0);
      sums.put(name, new double[] {0.0, 0.0});
      nearestDistance.put(name, Double.POSITIVE_INFINITY);
    }

    for (Translation2d f : fuel) {
      for (Map.Entry<String, double[]> entry : zones.entrySet()) {
        double[] r = entry.getValue();
        if (f.getX() < r[0] || f.getX() > r[2] || f.getY() < r[1] || f.getY() > r[3]) {
          continue;
        }
        String name = entry.getKey();
        counts.merge(name, 1, Integer::sum);
        double[] sum = sums.get(name);
        sum[0] += f.getX();
        sum[1] += f.getY();
        double distance = f.getDistance(robot);
        if (distance < nearestDistance.get(name)) {
          nearestDistance.put(name, distance);
          nearest.put(name, f);
        }
      }
    }

    StringBuilder json = new StringBuilder("{");
    boolean first = true;
    for (String name : zones.keySet()) {
      int count = counts.get(name);
      double[] sum = sums.get(name);
      double cx = count > 0 ? sum[0] / count : 0.0;
      double cy = count > 0 ? sum[1] / count : 0.0;
      Translation2d nf = nearest.get(name);
      json.append(first ? "" : ",")
          .append('"')
          .append(name)
          .append("\":{\"count\":")
          .append(count)
          .append(",\"centre\":[")
          .append(num(cx))
          .append(",")
          .append(num(cy))
          .append("],\"nearest\":")
          .append(nf == null ? "null" : "[" + num(nf.getX()) + "," + num(nf.getY()) + "]")
          .append(",\"nearest_distance_m\":")
          .append(nf == null ? "null" : num(nearestDistance.get(name)))
          .append("}");
      first = false;
    }
    return json.append("}").toString();
  }

  /** The {@code MAX_FUEL_JSON_NEAREST} closest fuel to the robot, nearest first. */
  private String buildNearestFuelJson(List<Translation2d> fuel, Translation2d robot) {
    List<Translation2d> sorted =
        fuel.stream()
            .sorted(Comparator.comparingDouble(f -> f.getDistance(robot)))
            .limit(MAX_FUEL_JSON_NEAREST)
            .toList();
    StringBuilder json = new StringBuilder("[");
    for (int i = 0; i < sorted.size(); i++) {
      Translation2d f = sorted.get(i);
      json.append(i == 0 ? "" : ",")
          .append("{\"x\":")
          .append(num(f.getX()))
          .append(",\"y\":")
          .append(num(f.getY()))
          .append(",\"distance_m\":")
          .append(num(f.getDistance(robot)))
          .append("}");
    }
    return json.append("]").toString();
  }

  /** The {@code MAX_FUEL_JSON_CLUSTERS} biggest fuel blobs on the field, biggest first. */
  private String buildFuelClustersJson(List<Translation2d> fuel) {
    List<double[]> clusters = greedyClusterFuel(fuel);
    clusters.sort((a, b) -> Double.compare(b[2], a[2]));
    int limit = Math.min(clusters.size(), MAX_FUEL_JSON_CLUSTERS);
    StringBuilder json = new StringBuilder("[");
    for (int i = 0; i < limit; i++) {
      double[] c = clusters.get(i);
      json.append(i == 0 ? "" : ",")
          .append("{\"x\":")
          .append(num(c[0]))
          .append(",\"y\":")
          .append(num(c[1]))
          .append(",\"count\":")
          .append((int) c[2])
          .append(",\"radius_m\":")
          .append(num(c[3]))
          .append("}");
    }
    return json.append("]").toString();
  }

  /**
   * Greedy blob clustering: pick the first unclustered piece as a seed, sweep up every unclustered
   * piece within {@code FUEL_CLUSTER_RADIUS_M} of it, call that one cluster, repeat. Not a real
   * clustering algorithm (it is seed-order dependent and does not chain outward), but a field of a
   * few hundred fuel does not need one - this is O(n^2) and cheap at the 5 Hz this runs.
   *
   * @return each cluster as {@code {x, y, count, radius_m}}
   */
  private static List<double[]> greedyClusterFuel(List<Translation2d> fuel) {
    int n = fuel.size();
    boolean[] used = new boolean[n];
    List<double[]> clusters = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      if (used[i]) {
        continue;
      }
      Translation2d seed = fuel.get(i);
      List<Translation2d> members = new ArrayList<>();
      for (int j = i; j < n; j++) {
        if (!used[j] && fuel.get(j).getDistance(seed) <= FUEL_CLUSTER_RADIUS_M) {
          members.add(fuel.get(j));
          used[j] = true;
        }
      }
      double cx = members.stream().mapToDouble(Translation2d::getX).average().orElse(seed.getX());
      double cy = members.stream().mapToDouble(Translation2d::getY).average().orElse(seed.getY());
      Translation2d centroid = new Translation2d(cx, cy);
      double radius = members.stream().mapToDouble(m -> m.getDistance(centroid)).max().orElse(0.0);
      clusters.add(new double[] {cx, cy, members.size(), radius});
    }
    return clusters;
  }

  /**
   * The {@code Fuel.pickup} advice: where to stand, which way to face, and where the sweep ends,
   * for the nearest fuel in range. Built from {@link CollectFuelCommand#planPickup} - the exact
   * same static method {@code COLLECT_FUEL}/{@code GRAB_FUEL} use to actually drive - so the
   * advice the model reads and the sweep the robot runs can never disagree.
   */
  private String buildPickupJson(Translation2d robot, List<Translation2d> fuel) {
    List<Translation2d> inRange =
        fuel.stream().filter(f -> f.getDistance(robot) <= CollectFuelCommand.SEARCH_RADIUS_M).toList();
    Optional<CollectFuelCommand.PickupPlan> planned = CollectFuelCommand.planPickup(robot, inRange);
    if (planned.isEmpty()) {
      return "null";
    }
    CollectFuelCommand.PickupPlan plan = planned.get();
    double maxSpeed = Math.min(maxSpeedSub.get(DEFAULT_MAX_SPEED_MPS), COLLECT_SPEED_MPS);
    return "{\"x\":"
        + num(plan.entryPoint().getX())
        + ",\"y\":"
        + num(plan.entryPoint().getY())
        + ",\"heading_deg\":"
        + num(plan.heading().getDegrees())
        + ",\"max_speed\":"
        + num(maxSpeed)
        + ",\"sweep_to\":["
        + num(plan.sweepEnd().getX())
        + ","
        + num(plan.sweepEnd().getY())
        + "],\"fuel\":["
        + num(plan.fuel().getX())
        + ","
        + num(plan.fuel().getY())
        + "],\"note\":\"drive here with INTAKE down, fuel passes on your LEFT\"}";
  }

  /**
   * Decides the intake's actual physical position, every loop: down when the agent wants it down,
   * UNLESS the robot is presently inside a trench/bump hazard rectangle and auto-fold is on, in
   * which case it folds for clearance and remembers which hazard so it knows when it is safe to
   * lower again. This is the ONLY place {@link #intakeActuator} is called - see its javadoc for
   * why that has to be true - and it is also where {@code IntakePolicy} is published.
   *
   * <p>Hysteresis: the fold trigger uses the hazard rectangle inflated by {@code FOLD_MARGIN_M}
   * (fold a bit early), but once folded the exit test requires clearing an even larger rectangle
   * ({@code FOLD_MARGIN_M + FOLD_EXIT_HYSTERESIS_M}) before lowering again, so sitting right on
   * the fold boundary cannot make the intake flap.
   */
  private void updateIntakeAutoFold() {
    boolean autoFold = autoFoldIntakeSub.get(true);

    if (!intakeUnderAiControl) {
      // No AI action has ever touched the intake, so it stays fully under driver/autonomous
      // control and this loop must not actuate anything - just report reality.
      intakePolicyPub.set(
          "{\"intake_down\":"
              + intakeDownSupplier.getAsBoolean()
              + ",\"auto_fold\":"
              + autoFold
              + ",\"folded_for\":null,\"in_hazard\":false,\"hazard\":null,"
              + "\"wants_down\":false,\"reason\":\"intake under manual/autonomous control\"}");
      return;
    }

    Translation2d point = poseSupplier.get().getTranslation();

    if (foldedForHazard != null
        && !(autoFold
            && inflatedZoneContains(
                foldedForHazard, FOLD_MARGIN_M + FOLD_EXIT_HYSTERESIS_M, point))) {
      foldedForHazard = null;
    }
    if (foldedForHazard == null && autoFold) {
      for (String hazard : HAZARD_ZONE_NAMES) {
        if (inflatedZoneContains(hazard, FOLD_MARGIN_M, point)) {
          foldedForHazard = hazard;
          break;
        }
      }
    }

    boolean lowered = foldedForHazard == null && intakeWantsDown;
    if (intakeActuator != null) {
      intakeActuator.accept(lowered);
    }

    String reason;
    if (foldedForHazard != null) {
      reason = "folded to clear " + foldedForHazard + " - lowering again once past it";
    } else if (intakeWantsDown) {
      reason = "intake down";
    } else {
      reason = "intake stowed";
    }

    String hazardJson = foldedForHazard == null ? "null" : '"' + foldedForHazard + '"';
    intakePolicyPub.set(
        "{\"intake_down\":"
            + lowered
            + ",\"auto_fold\":"
            + autoFold
            + ",\"folded_for\":"
            + hazardJson
            + ",\"in_hazard\":"
            + (foldedForHazard != null)
            + ",\"hazard\":"
            + hazardJson
            + ",\"wants_down\":"
            + intakeWantsDown
            + ",\"reason\":\""
            + jsonEscape(reason)
            + "\"}");

    Logger.recordOutput("AIControl/Intake/WantsDown", intakeWantsDown);
    Logger.recordOutput("AIControl/Intake/Lowered", lowered);
    Logger.recordOutput("AIControl/Intake/FoldedFor", foldedForHazard == null ? "" : foldedForHazard);
  }

  /** True if {@code point} is inside the named zone, inflated by {@code margin} on every side. */
  private boolean inflatedZoneContains(String zoneName, double margin, Translation2d point) {
    double[] r = zones.get(zoneName);
    if (r == null) {
      return false;
    }
    return point.getX() >= r[0] - margin
        && point.getX() <= r[2] + margin
        && point.getY() >= r[1] - margin
        && point.getY() <= r[3] + margin;
  }

  /**
   * Publishes {@code HubState} from {@link HubShiftUtil}: whether OUR hub is active right now,
   * which shift we are in, and how long it has left. This is the single biggest gap in an agent's
   * model of the game without it - a shot that scores nothing because the hub is inactive looks
   * indistinguishable from one that just missed.
   */
  private void publishHubState() {
    HubShiftUtil.ShiftInfo info = HubShiftUtil.getOfficialShiftInfo();
    hubStatePub.set(
        "{\"active\":"
            + info.active()
            + ",\"shift\":\""
            + info.currentShift()
            + "\",\"shift_remaining_s\":"
            + num(Math.max(0.0, info.remainingTime()))
            + ",\"match_time_s\":"
            + num(Math.max(0.0, HubShiftUtil.getMatchTime()))
            + ",\"hub_state_ignored\":"
            + hubStateIgnoredSupplier.getAsBoolean()
            + "}");
    Logger.recordOutput("AIControl/HubState/Active", info.active());
    Logger.recordOutput("AIControl/HubState/Shift", info.currentShift().toString());
  }

  /** Rounds to 2 decimals for a readable JSON number - the model does not need float noise. */
  private static String num(double v) {
    return String.format(java.util.Locale.US, "%.2f", v);
  }

  /** Minimal escaping for the handful of human-written strings ({@code what}, {@code reason}). */
  private static String jsonEscape(String s) {
    return s.replace("\\", "\\\\").replace("\"", "\\\"");
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
