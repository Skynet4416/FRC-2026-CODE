// Copyright (c) 2026 Skynet 4416
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import java.util.ArrayDeque;
import java.util.Comparator;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the robot along the nearest line of fuel with the intake down, so an AI operator can ask
 * for fuel instead of having to work out a pose, a heading and a speed by hand.
 *
 * <p>This is the "AI understands it can collect while moving" fix: {@code AIControlBridge}'s
 * {@code INTAKE} action only latches the mechanism down, it never drives anywhere. This command is
 * the thing that actually drives - it owns the drivetrain the way a shooting action does, but
 * instead of a fixed shooting spot it re-plans a short sweep every time the fuel it was chasing
 * runs out, until it has collected enough or there is nothing left in reach.
 *
 * <p>It is also the command behind {@code GRAB_FUEL}: that action is this same sweep fed from a
 * tighter, nearby-only fuel list instead of the whole field, so "grab what's in front of me" and
 * "sweep that line of fuel" are one implementation with two different {@code fuelSupplier}s -
 * never two separate driving loops.
 */
public class CollectFuelCommand extends Command {
  /**
   * The intake pickup box (see {@code RobotContainer#addIntakeZone}) is a strip down the robot's
   * LEFT (+Y) flank, robot-relative x in [-0.354, 0.354], y in [0.35, 0.61]. This is that band's
   * centre, sideways from the robot: (0.35 + 0.61) / 2.
   */
  public static final double INTAKE_OFFSET_M = 0.48;

  /** Only plan around fuel within this far of the robot - there is no point sweeping the field. */
  public static final double SEARCH_RADIUS_M = 8.0;

  /** Fuel this close to the nearest piece counts as part of the same line to sweep. */
  private static final double CLUSTER_RADIUS_M = 1.5;

  /**
   * Below this eigenvalue ratio the local fuel reads as a line, not a round blob; above it there
   * is no dominant direction to sweep along, so aim at the cluster's centre instead.
   */
  private static final double BLOB_ROUNDNESS_THRESHOLD = 0.5;

  private static final double ENTRY_BACKOFF_M = 0.8;
  private static final double SWEEP_OVERRUN_M = 0.5;
  private static final double MAX_SWEEP_LENGTH_M = 4.0;
  private static final double APPROACH_SPEED_MPS = 2.0;
  private static final double SWEEP_SPEED_MPS = 1.0;
  private static final double POSITION_KP = 2.5;
  private static final double HEADING_KP = 4.0;
  private static final double MAX_OMEGA_RAD_PER_SEC = Math.PI;
  private static final double POSITION_TOLERANCE_M = 0.12;
  private static final double OVERALL_TIMEOUT_SECONDS = 30.0;

  // Hub avoidance: route around either hub, inflated a bit, via whichever trench lane is closer.
  private static final double HUB_AVOID_MARGIN_M = 0.6;
  private static final double LEFT_LANE_Y =
      (FieldConstants.LinesHorizontal.leftTrenchOpenStart
              + FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
          / 2.0;
  private static final double RIGHT_LANE_Y =
      (FieldConstants.LinesHorizontal.rightTrenchOpenStart
              + FieldConstants.LinesHorizontal.rightTrenchOpenEnd)
          / 2.0;
  private static final Rect[] HUB_RECTS = {
    inflatedHubRect(
        FieldConstants.Hub.nearLeftCorner,
        FieldConstants.Hub.nearRightCorner,
        FieldConstants.Hub.farLeftCorner,
        FieldConstants.Hub.farRightCorner),
    inflatedHubRect(
        FieldConstants.Hub.oppNearLeftCorner,
        FieldConstants.Hub.oppNearRightCorner,
        FieldConstants.Hub.oppFarLeftCorner,
        FieldConstants.Hub.oppFarRightCorner),
  };

  private enum Phase {
    DRIVING_TO_ENTRY,
    SWEEPING,
    DONE
  }

  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<ChassisSpeeds> velocityConsumer;
  private final Supplier<List<Translation2d>> fuelSupplier;
  private final Runnable lowerIntake;
  private final IntSupplier onBoard;
  private final DoubleSupplier collectTarget;

  private final Timer timer = new Timer();
  private final Deque<Translation2d> route = new ArrayDeque<>();

  private Phase phase = Phase.DONE;
  private int startingOnBoard = 0;
  private Rotation2d sweepHeading = Rotation2d.kZero;
  private Translation2d sweepTarget = Translation2d.kZero;
  private String idleReason = "";

  /**
   * @param poseSupplier the robot's current field pose
   * @param velocityConsumer runs robot-relative chassis speeds, same one the AI bridge drives with
   * @param driveSubsystem the drivetrain - required, so this and a path or a shot can never fight
   *     over it
   * @param fuelSupplier ground-truth fuel to plan around; {@code COLLECT_FUEL} passes the whole
   *     field, {@code GRAB_FUEL} passes a tighter nearby-only list - that is the entire difference
   *     between the two actions
   * @param lowerIntake declares intent to keep the intake down; called every loop, cheap on
   *     purpose, because the actual mechanism position is arbitrated elsewhere (the automatic
   *     trench/bump fold can legitimately raise it out from under this command mid-sweep)
   * @param onBoard fuel currently held, so progress can be measured against {@code collectTarget}
   * @param collectTarget how many additional pieces to gather before stopping
   */
  public CollectFuelCommand(
      Supplier<Pose2d> poseSupplier,
      Consumer<ChassisSpeeds> velocityConsumer,
      Subsystem driveSubsystem,
      Supplier<List<Translation2d>> fuelSupplier,
      Runnable lowerIntake,
      IntSupplier onBoard,
      DoubleSupplier collectTarget) {
    this.poseSupplier = poseSupplier;
    this.velocityConsumer = velocityConsumer;
    this.fuelSupplier = fuelSupplier;
    this.lowerIntake = lowerIntake;
    this.onBoard = onBoard;
    this.collectTarget = collectTarget;
    addRequirements(driveSubsystem);
    setName("AIControl/CollectFuel");
  }

  @Override
  public void initialize() {
    timer.restart();
    startingOnBoard = onBoard.getAsInt();
    lowerIntake.run();
    plan();
  }

  @Override
  public void execute() {
    lowerIntake.run();
    Pose2d pose = poseSupplier.get();

    switch (phase) {
      case DRIVING_TO_ENTRY -> {
        Translation2d target = route.peekFirst();
        if (target == null) {
          phase = Phase.SWEEPING;
        } else {
          driveToward(pose, target, APPROACH_SPEED_MPS);
          if (target.getDistance(pose.getTranslation()) <= POSITION_TOLERANCE_M) {
            route.pollFirst();
            if (route.isEmpty()) {
              phase = Phase.SWEEPING;
            }
          }
        }
      }
      case SWEEPING -> {
        driveToward(pose, sweepTarget, SWEEP_SPEED_MPS);
        if (sweepTarget.getDistance(pose.getTranslation()) <= POSITION_TOLERANCE_M) {
          // The line we were sweeping is done - see if there is another one to chase.
          plan();
        }
      }
      case DONE -> velocityConsumer.accept(new ChassisSpeeds());
    }

    Logger.recordOutput("AIControl/Collect/Phase", phase.toString());
    Logger.recordOutput("AIControl/Collect/OnBoard", onBoard.getAsInt());
  }

  @Override
  public void end(boolean interrupted) {
    velocityConsumer.accept(new ChassisSpeeds());
    // Deliberately does not touch the intake. COLLECT_FUEL/GRAB_FUEL always leave it down when
    // they end, whether that is finishing on their own (target met, hopper full, nothing left in
    // reach) or losing the drivetrain to something else that took over, e.g. a shooting action.
    Logger.recordOutput(
        "AIControl/Collect/Phase", "IDLE" + (idleReason.isEmpty() ? "" : " - " + idleReason));
  }

  @Override
  public boolean isFinished() {
    if (phase == Phase.DONE) {
      return true;
    }
    if (timer.hasElapsed(OVERALL_TIMEOUT_SECONDS)) {
      idleReason = "timed out after " + OVERALL_TIMEOUT_SECONDS + " s";
      return true;
    }
    int target = startingOnBoard + (int) Math.round(collectTarget.getAsDouble());
    return onBoard.getAsInt() >= target;
  }

  /**
   * Picks the next fuel line to sweep and lays out a route to it, from wherever the robot is right
   * now. Called once at the start and again every time a sweep runs its course, which is how the
   * command keeps going until the target count is met or the field runs dry.
   */
  private void plan() {
    Pose2d pose = poseSupplier.get();
    Translation2d robot = pose.getTranslation();
    List<Translation2d> fuel = fuelSupplier.get();
    List<Translation2d> inRange =
        (fuel == null ? List.<Translation2d>of() : fuel).stream()
            .filter(f -> f.getDistance(robot) <= SEARCH_RADIUS_M)
            .toList();

    Optional<PickupPlan> planned = planPickup(robot, inRange);
    if (planned.isEmpty()) {
      phase = Phase.DONE;
      idleReason = "no fuel within " + SEARCH_RADIUS_M + " m";
      Logger.recordOutput("AIControl/Collect/PlanStatus", idleReason);
      return;
    }

    PickupPlan p = planned.get();
    route.clear();
    route.addAll(routeAvoidingHubs(robot, p.entryPoint()));
    route.addLast(p.entryPoint());
    sweepHeading = p.heading();
    sweepTarget = p.sweepEnd();
    phase = Phase.DRIVING_TO_ENTRY;

    Logger.recordOutput("AIControl/Collect/PlanStatus", "sweeping toward " + p.fuel());
    Logger.recordOutput("AIControl/Collect/TargetFuel", p.fuel());
    Logger.recordOutput("AIControl/Collect/EntryPoint", p.entryPoint());
    Logger.recordOutput("AIControl/Collect/SweepEnd", p.sweepEnd());
    Logger.recordOutput("AIControl/Collect/HeadingDeg", p.heading().getDegrees());
  }

  /** Simple holonomic P controller: drive at {@code target}, hold {@link #sweepHeading}. */
  private void driveToward(Pose2d pose, Translation2d target, double maxSpeedMps) {
    Translation2d error = target.minus(pose.getTranslation());
    double distance = error.getNorm();
    double speed = Math.min(maxSpeedMps, POSITION_KP * distance);
    Translation2d fieldVelocity =
        distance > 1e-6 ? error.times(speed / distance) : Translation2d.kZero;

    double headingErrorRad =
        MathUtil.angleModulus(sweepHeading.getRadians() - pose.getRotation().getRadians());
    double omega =
        MathUtil.clamp(
            HEADING_KP * headingErrorRad, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);

    ChassisSpeeds fieldSpeeds =
        new ChassisSpeeds(fieldVelocity.getX(), fieldVelocity.getY(), omega);
    velocityConsumer.accept(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, pose.getRotation()));
  }

  /** One planned pickup: which fuel, what heading to sweep it on, and where to stand. */
  public record PickupPlan(
      Translation2d fuel, Rotation2d heading, Translation2d entryPoint, Translation2d sweepEnd) {}

  /**
   * Plans a pickup for the nearest fuel to {@code robot} out of {@code fuelInRange}, or empty if
   * the list is empty. This is the single source of truth for "where do I stand to collect that
   * ball" - both the sweep above and {@code AIControlBridge}'s {@code pickup} advice in the {@code
   * Fuel} JSON call this, so the two can never disagree about what "go collect it" means.
   */
  public static Optional<PickupPlan> planPickup(Translation2d robot, List<Translation2d> fuelInRange) {
    if (fuelInRange.isEmpty()) {
      return Optional.empty();
    }
    Translation2d p0 =
        fuelInRange.stream().min(Comparator.comparingDouble(f -> f.getDistance(robot))).get();

    List<Translation2d> local =
        fuelInRange.stream().filter(f -> f.getDistance(p0) <= CLUSTER_RADIUS_M).toList();

    Rotation2d theta = sweepHeadingFor(robot, p0, local);
    Translation2d dir = new Translation2d(theta.getCos(), theta.getSin());

    double maxProjection = 0.0;
    for (Translation2d f : local) {
      Translation2d rel = f.minus(p0);
      double projection = rel.getX() * dir.getX() + rel.getY() * dir.getY();
      maxProjection = Math.max(maxProjection, projection);
    }
    double sweepLength =
        MathUtil.clamp(maxProjection + SWEEP_OVERRUN_M, SWEEP_OVERRUN_M, MAX_SWEEP_LENGTH_M);

    Translation2d entryPoint = pickupPoint(p0, theta).minus(dir.times(ENTRY_BACKOFF_M));
    Translation2d sweepEndFuel = p0.plus(dir.times(sweepLength));
    Translation2d sweepEnd = pickupPoint(sweepEndFuel, theta);

    return Optional.of(new PickupPlan(p0, theta, entryPoint, sweepEnd));
  }

  /**
   * The point the robot's centre must occupy for {@code fuel} to sit inside the intake's pickup
   * box while the robot holds {@code heading}.
   *
   * <p>Derivation: the pickup box is centred, robot-relative, at {@code (0, INTAKE_OFFSET_M)} -
   * straight out the robot's LEFT (+Y) side. Rotating that robot-relative offset into the field
   * frame by {@code heading} gives the fuel's position relative to the robot centre:
   *
   * <pre>
   *   fuel - robotCentre = R(heading) * (0, INTAKE_OFFSET_M)
   *                      = (-INTAKE_OFFSET_M * sin(heading), INTAKE_OFFSET_M * cos(heading))
   *   robotCentre = fuel - (-INTAKE_OFFSET_M * sin(heading), INTAKE_OFFSET_M * cos(heading))
   *               = fuel + INTAKE_OFFSET_M * (sin(heading), -cos(heading))
   * </pre>
   *
   * <p>Sanity check with {@code heading = 0} (facing +X, so the robot's left is +Y): a piece at
   * (5.0, 4.0) wants the robot centred at (5.0, 4.0 - 0.48) = (5.0, 3.52) - offset to the robot's
   * RIGHT of the piece, so the piece ends up on the robot's LEFT as it drives forward through it.
   */
  public static Translation2d pickupPoint(Translation2d fuel, Rotation2d heading) {
    return fuel.plus(
        new Translation2d(
            INTAKE_OFFSET_M * heading.getSin(), -INTAKE_OFFSET_M * heading.getCos()));
  }

  /**
   * Sweep heading for a local group of fuel: the direction of its principal spread (a line of
   * fuel gets swept lengthwise), or, if the group reads as a round blob rather than a line, the
   * direction from the robot to its centroid.
   */
  private static Rotation2d sweepHeadingFor(
      Translation2d robot, Translation2d p0, List<Translation2d> local) {
    Translation2d towardCluster;
    if (local.size() < 2) {
      towardCluster = p0.minus(robot);
      return towardCluster.getNorm() > 1e-6
          ? new Rotation2d(towardCluster.getX(), towardCluster.getY())
          : Rotation2d.kZero;
    }

    double meanX = local.stream().mapToDouble(Translation2d::getX).average().orElse(p0.getX());
    double meanY = local.stream().mapToDouble(Translation2d::getY).average().orElse(p0.getY());
    double sxx = 0, syy = 0, sxy = 0;
    for (Translation2d f : local) {
      double dx = f.getX() - meanX;
      double dy = f.getY() - meanY;
      sxx += dx * dx;
      syy += dy * dy;
      sxy += dx * dy;
    }
    int n = local.size();
    sxx /= n;
    syy /= n;
    sxy /= n;

    // Eigen-decomposition of the 2x2 covariance [[sxx,sxy],[sxy,syy]]: the principal axis angle,
    // and how much bigger the dominant eigenvalue is than the other (a true line has one
    // eigenvalue near zero; a round blob has both about equal).
    double principalAngle = 0.5 * Math.atan2(2 * sxy, sxx - syy);
    double meanTerm = (sxx + syy) / 2.0;
    double spread = Math.hypot((sxx - syy) / 2.0, sxy);
    double lambda1 = meanTerm + spread;
    double lambda2 = meanTerm - spread;

    towardCluster = new Translation2d(meanX - robot.getX(), meanY - robot.getY());
    Translation2d direction;
    boolean isLineLike = lambda1 > 1e-9 && (lambda2 / lambda1) < BLOB_ROUNDNESS_THRESHOLD;
    if (isLineLike) {
      direction = new Translation2d(Math.cos(principalAngle), Math.sin(principalAngle));
      // The principal axis is a line, not a ray - pick the half that actually points away from
      // the robot so the sweep drives forward through the fuel rather than back through itself.
      if (direction.getX() * towardCluster.getX() + direction.getY() * towardCluster.getY() < 0) {
        direction = direction.times(-1);
      }
    } else {
      direction = towardCluster;
    }

    if (direction.getNorm() < 1e-6) {
      direction = towardCluster.getNorm() > 1e-6 ? towardCluster : new Translation2d(1, 0);
    }
    return new Rotation2d(direction.getX(), direction.getY());
  }

  /**
   * If the straight line from {@code from} to {@code to} crosses either hub (inflated for
   * clearance), inserts one waypoint that routes around it via whichever trench lane's Y is
   * closer - simple point-to-point-to-point, not a real path planner, which is all a sweep this
   * short needs.
   */
  private static List<Translation2d> routeAvoidingHubs(Translation2d from, Translation2d to) {
    for (Rect hub : HUB_RECTS) {
      if (hub.intersects(from, to)) {
        double midY = (from.getY() + to.getY()) / 2.0;
        double laneY =
            Math.abs(midY - LEFT_LANE_Y) <= Math.abs(midY - RIGHT_LANE_Y)
                ? LEFT_LANE_Y
                : RIGHT_LANE_Y;
        double waypointX = (from.getX() + to.getX()) / 2.0;
        return List.of(new Translation2d(waypointX, laneY));
      }
    }
    return List.of();
  }

  /** Axis-aligned rectangle with a segment-intersection test, used only for hub avoidance. */
  private record Rect(double xMin, double yMin, double xMax, double yMax) {
    /** Liang-Barsky line-clip: true if the segment {@code a -> b} passes through this rect. */
    boolean intersects(Translation2d a, Translation2d b) {
      double dx = b.getX() - a.getX();
      double dy = b.getY() - a.getY();
      double[] p = {-dx, dx, -dy, dy};
      double[] q = {a.getX() - xMin, xMax - a.getX(), a.getY() - yMin, yMax - a.getY()};
      double t0 = 0.0;
      double t1 = 1.0;
      for (int i = 0; i < 4; i++) {
        if (Math.abs(p[i]) < 1e-9) {
          if (q[i] < 0) {
            return false;
          }
        } else {
          double r = q[i] / p[i];
          if (p[i] < 0) {
            if (r > t1) {
              return false;
            }
            if (r > t0) {
              t0 = r;
            }
          } else {
            if (r < t0) {
              return false;
            }
            if (r < t1) {
              t1 = r;
            }
          }
        }
      }
      return true;
    }
  }

  private static Rect inflatedHubRect(Translation2d... corners) {
    double xMin = Double.POSITIVE_INFINITY;
    double yMin = Double.POSITIVE_INFINITY;
    double xMax = Double.NEGATIVE_INFINITY;
    double yMax = Double.NEGATIVE_INFINITY;
    for (Translation2d c : corners) {
      xMin = Math.min(xMin, c.getX());
      yMin = Math.min(yMin, c.getY());
      xMax = Math.max(xMax, c.getX());
      yMax = Math.max(yMax, c.getY());
    }
    return new Rect(
        xMin - HUB_AVOID_MARGIN_M,
        yMin - HUB_AVOID_MARGIN_M,
        xMax + HUB_AVOID_MARGIN_M,
        yMax + HUB_AVOID_MARGIN_M);
  }
}
