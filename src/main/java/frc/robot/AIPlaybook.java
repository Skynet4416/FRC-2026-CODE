// Copyright (c) 2026 Skynet 4416
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import java.util.List;

/**
 * The team's real autonomous knowledge, ported for the AI control bridge's {@code Tactics} and
 * {@code Playbook} topics: the driving/mechanism tactics 4416 actually bakes into its seven Choreo
 * autos, and six of those autos themselves as literal, replayable plays.
 *
 * <p>Sourced from the seven auto-building methods in {@code RobotContainer} and the live {@code
 * .traj} files they drive (waypoints, speed constraints, and the {@code IntakeOpen} event timing),
 * not from {@code ChoreoVars}/{@code ChoreoTraj} - those two files are dead code referenced by
 * nothing in this repo, and none of their coordinates were used here.
 *
 * <p>Only six plays are published. {@code leftTrenchHubIntakeReturnOverBump()} ("Behind Hub
 * Intake") calls {@code routine.trajectory("Behind_Hub_Intake_1")} and {@code "_2"}, and neither
 * {@code .traj} file exists anywhere in the deploy directory - it is a live bug in the chooser, not
 * a tactic, so there are no coordinates to port and it is left out entirely rather than guessed at.
 * See {@code RobotContainer}'s own auto-building methods for that routine; nothing here should be
 * taken as a stand-in for it.
 *
 * <p>Blue-origin coordinates and WPILib headings throughout (0 degrees = robot front toward +X,
 * positive = counter-clockwise), matching every other pose in the AI control bridge's API - {@code
 * AutoFactory} mirrors these for red automatically, same as it does for the real autos.
 */
public final class AIPlaybook {
  private AIPlaybook() {}

  /**
   * Ten operating rules distilled from every real Choreo auto this team has run, addressed directly
   * to whatever is driving - human or agent. Trimmed to the rule and the number that matters, not
   * the waypoint-by-waypoint derivation; see the {@code Playbook} topic for the actual routes these
   * rules came from.
   */
  public static final String TACTICS =
      """
      Ten operating rules distilled from every real Choreo auto this team has run. Follow them \
      the same way whether a human is driving or you are.

      1. A hub shot only counts from our own alliance zone, behind the hub (roughly blue-X <= \
      hub-X minus 0.5 m - InShootingZone says so directly). Past that line a shot becomes a \
      non-scoring lob pass back into your own zone. Always be back inside the zone before you \
      shoot.
      2. You do not have to be pre-aimed to shoot. A shooting action rotates the chassis onto \
      the live launch solution and leads for your own motion while it fires - arrive roughly in \
      the zone, roughly on the hub's heading, and trigger the shot; it finishes the aim for you.
      3. The trench is the highway, the bump is a shortcut. Use a trench lane (it runs the full \
      field length, along either side wall) for the long transit out to the neutral-zone fuel \
      line and back. Cross a bump only on the return leg, at a brisk but controlled speed (about \
      2 m/s) - never outbound, and never as a flat-out sprint.
      4. Deploy the intake once you are actually over fuel, not at the start of a drive. Keep it \
      stowed while you cover empty ground, and drop it right as you reach the fuel line so it is \
      down for the collection, not the whole trip.
      5. Two collection speeds are used on purpose. Creep at about 0.35 m/s when precision \
      matters most: a wall-hugging stack, or the first pass through a fixed dip. Use a cautious \
      1.0-1.125 m/s for a first pass through an open fuel line. Do not slow down at all for a \
      second pass through ground you already swept once - trust the intake to grab it in stride.
      6. Size your shoot window to how much fuel you expect to be holding, not a fixed constant. \
      A normal collection leg earns a short 2.5-5 s shot window; a leg expected to fill the \
      hopper (a full depot sweep, or a second sweep stacked on the first) earns 7-15 s to \
      actually empty it.
      7. Fold the intake about one second into every shot, never at the instant the shot starts. \
      Let it finish grabbing whatever the drive was still gathering, then stow it clear of the \
      indexers and hood for the rest of the shot.
      8. Ignore the hub's active/inactive shift state while running a scripted play or an \
      autonomous routine - fire on your own timer instead of waiting for the hub to go active, \
      the way these routines are designed to run.
      9. Known fuel sources, roughly in priority order: the preloaded hopper (shoot it cold \
      before you move at all), the neutral-zone fuel line (reachable from either trench lane), \
      and the depot's wall-hugging stack. Never cross into the opponent's half of the field for \
      fuel.
      10. If you need one constant collection speed instead of tuning per fuel line, 1.0 m/s is \
      the right default - it sits in the middle of the 0.35-1.125 m/s range these tactics \
      actually use.
      """;

  /** One leg of a play, or one mechanism action - see {@link DriveStep} and {@link ActionStep}. */
  public sealed interface Step permits DriveStep, ActionStep {}

  /**
   * Pathfind to a pose at a specific speed cap - the same {@code TargetPose} pathfinding the bridge
   * already does for the agent, just with the speed the real auto used for that leg (its {@code
   * MaxVelocity} constraint where the source {@code .traj} file set one, otherwise the trajectory's
   * own observed cruise speed for that segment).
   *
   * @param x blue-origin field X, meters
   * @param y blue-origin field Y, meters
   * @param headingDeg field heading, degrees, WPILib convention (0 = +X, counter-clockwise
   *     positive)
   * @param speedMps pathfinding max speed for this leg, m/s
   * @param note human-readable context for why this leg looks the way it does; empty when the
   *     source auto did not call for one
   */
  public record DriveStep(double x, double y, double headingDeg, double speedMps, String note)
      implements Step {}

  /**
   * Trigger a registered mechanism action and wait for it if it is the kind of action that owns the
   * drivetrain for a bounded time (a shot, a collect sweep); fire it and move straight on to the
   * next step if it is a background toggle instead (INTAKE, STOW_INTAKE and the auto-fold switches
   * never block a play, exactly like the real auto's own instantaneous {@code setLowered()} calls
   * never block a trajectory).
   *
   * @param name a name from {@code /AIControl/AvailableActions}, e.g. {@code "SHOOT_FUEL"}
   * @param secondsOverride for a {@code SHOOT_FUEL} step, the shot window the real auto actually
   *     used for this moment (2.5-15 s depending on how much fuel the leg before it was expected to
   *     collect - see {@code ShootSeconds} on the bridge); 0 or less means "leave {@code
   *     ShootSeconds} exactly as it already is."
   * @param note human-readable context; empty when the source auto did not call for one
   */
  public record ActionStep(String name, double secondsOverride, String note) implements Step {}

  /**
   * One replayable auto, in the same shape {@code RUN_PLAY} walks: a name to select it by, a
   * one-line summary of the tactic, and the ordered steps that carry it out. The first step is
   * always a {@link DriveStep} to the play's authored start pose - the real auto is physically
   * placed there and just resets odometry, but {@code RUN_PLAY} can be triggered from anywhere on
   * the field, so it has to actually drive there first.
   */
  public record Play(String name, String purpose, List<Step> steps) {}

  // Approach speed for the synthetic "drive to the authored start pose" step every play opens
  // with - not from a real auto (which starts parked there), just a sane transit speed for
  // getting there from wherever RUN_PLAY was actually triggered.
  private static final double START_APPROACH_SPEED_MPS = 2.0;

  private static DriveStep start(double x, double y, double headingDeg) {
    return new DriveStep(
        x,
        y,
        headingDeg,
        START_APPROACH_SPEED_MPS,
        "play start pose - the real auto starts parked here via resetOdometry(); drive here"
            + " first since RUN_PLAY can be triggered from anywhere on the field");
  }

  private static DriveStep d(double x, double y, double headingDeg, double speedMps) {
    return new DriveStep(x, y, headingDeg, speedMps, "");
  }

  private static DriveStep d(double x, double y, double headingDeg, double speedMps, String note) {
    return new DriveStep(x, y, headingDeg, speedMps, note);
  }

  private static ActionStep a(String name) {
    return new ActionStep(name, 0.0, "");
  }

  private static ActionStep a(String name, String note) {
    return new ActionStep(name, 0.0, note);
  }

  private static ActionStep shoot(double seconds, String note) {
    return new ActionStep("SHOOT_FUEL", seconds, note);
  }

  /**
   * Every play this team actually runs, in chooser order, minus the one broken entry (see the class
   * javadoc). Each is a faithful port of its {@code RobotContainer} auto method and the {@code
   * .traj} files it drives - names match the real methods, snake_cased.
   */
  public static final List<Play> PLAYS =
      List.of(
          new Play(
              "choreo_test",
              "Shoot the preload cold, then run two trench-and-dip sweeps of different reach,"
                  + " shooting after each.",
              List.of(
                  start(3.635, 7.440, -90.7),
                  shoot(3.0, "shoot the preload from the start pose before moving at all"),
                  a("INTAKE", "lower the intake before departing"),
                  d(5.421, 7.440, -90.7, 2.0),
                  d(
                      7.683,
                      7.160,
                      -143.3,
                      2.8,
                      "sprint out along the trench toward the neutral" + " zone"),
                  d(
                      7.789,
                      6.605,
                      -178.0,
                      0.9,
                      "full stop point in the real auto before the slow" + " dip"),
                  d(
                      7.810,
                      5.245,
                      -177.2,
                      0.35,
                      "deliberate creep through the fuel-line dip - a"
                          + " collection pass, not transit"),
                  d(7.741, 7.109, -146.0, 1.6),
                  d(4.646, 7.440, -90.7, 4.2, "sprint home along the trench"),
                  d(2.741, 7.118, -70.6, 1.1, "end pose aimed back toward the hub"),
                  shoot(5.0, "empties whatever the shallow sweep collected"),
                  a("STOW_INTAKE", "real auto stows about 1 s into the shot, not at shot start"),
                  a("INTAKE"),
                  d(3.242, 7.432, -90.7, 1.7),
                  d(
                      5.429,
                      7.440,
                      -91.9,
                      3.3,
                      "this leg's reach (~x=6.2) is shorter than the" + " first sweep's (~x=7.8)"),
                  d(6.357, 6.468, 180.0, 1.2),
                  d(6.242, 4.624, -178.8, 0.35, "deeper dip than lap 1, same 0.35 m/s creep cap"),
                  d(6.345, 6.331, -155.4, 2.0),
                  d(4.684, 7.440, -90.7, 3.2),
                  d(2.988, 7.209, -90.5, 1.0),
                  shoot(2.5, "final shot"),
                  a("STOW_INTAKE"))),
          new Play(
              "left_trench_double_take",
              "Same double trench-and-dip sweep as choreo_test, but no up-front preload shot -"
                  + " go straight into the first collect lap, and shorter shot windows.",
              List.of(
                  start(3.635, 7.440, -90.7),
                  a(
                      "STOW_INTAKE",
                      "explicit stow before departing, even though nothing was down" + " yet"),
                  a("INTAKE"),
                  d(5.421, 7.440, -90.7, 2.0),
                  d(7.683, 7.160, -143.3, 2.8),
                  d(7.789, 6.605, -178.0, 0.9),
                  d(7.810, 5.245, -177.2, 0.35, "collection creep"),
                  d(4.646, 7.440, -90.7, 4.2),
                  d(2.741, 7.118, -70.6, 1.1),
                  shoot(2.5, ""),
                  a("STOW_INTAKE", "fires about 1 s after the shot starts, not at shot start"),
                  a("INTAKE"),
                  d(5.429, 7.440, -91.9, 3.3),
                  d(6.357, 6.468, 180.0, 1.2),
                  d(6.242, 4.624, -178.8, 0.35, "deeper collection creep"),
                  d(4.684, 7.440, -90.7, 3.2),
                  d(2.988, 7.209, -90.5, 1.0),
                  shoot(2.5, ""),
                  a("STOW_INTAKE"))),
          new Play(
              "left_trench_single_take",
              "Identical drive plan to left_trench_double_take; the only real difference is a"
                  + " much longer first shot window. Despite the name, it still runs both"
                  + " sweeps.",
              List.of(
                  start(3.635, 7.440, -90.7),
                  a("STOW_INTAKE"),
                  a("INTAKE"),
                  d(5.421, 7.440, -90.7, 2.0),
                  d(7.683, 7.160, -143.3, 2.8),
                  d(7.789, 6.605, -178.0, 0.9),
                  d(7.810, 5.245, -177.2, 0.35, "collection creep"),
                  d(4.646, 7.440, -90.7, 4.2),
                  d(2.741, 7.118, -70.6, 1.1),
                  // The real auto gives this shot 15s to empty the hopper before still going on
                  // to run the deep sweep - the whole reason for the (misleading) method name.
                  // ShootSeconds makes this a single accurate step instead of the old workaround
                  // of repeating SHOOT_FUEL four times.
                  shoot(15.0, "long dwell to empty the hopper before continuing to lap 2"),
                  a("STOW_INTAKE"),
                  a("INTAKE"),
                  d(5.429, 7.440, -91.9, 3.3),
                  d(6.357, 6.468, 180.0, 1.2),
                  d(6.242, 4.624, -178.8, 0.35, "deeper collection creep"),
                  d(4.684, 7.440, -90.7, 3.2),
                  d(2.988, 7.209, -90.5, 1.0),
                  shoot(2.5, ""),
                  a("STOW_INTAKE"))),
          new Play(
              "depot",
              "Hug the alliance wall at a deliberate crawl to vacuum the depot's stacked fuel,"
                  + " then take one long shot from wherever the sweep ends - no need to return to"
                  + " a standoff spot first.",
              List.of(
                  start(3.445, 4.074, -1.8),
                  a("STOW_INTAKE"),
                  a(
                      "INTAKE",
                      "down from the very first instant here - the fuel is right at the"
                          + " start, unlike the trench routines"),
                  d(2.240, 4.136, -1.5, 1.3),
                  d(
                      1.462,
                      4.764,
                      56.0,
                      1.2,
                      "full stop and turn to face the wall line here in" + " the real auto"),
                  d(
                      0.881,
                      5.518,
                      56.0,
                      0.35,
                      "wall-hugging creep begins - x stays ~0.85-0.89,"
                          + " right against the depot's depth"),
                  d(0.891, 5.873, 56.0, 0.35),
                  d(0.852, 6.238, 56.0, 0.35),
                  d(0.852, 6.544, 56.0, 0.35),
                  d(
                      0.832,
                      7.245,
                      -35.3,
                      0.35,
                      "end pose is at the trench mouth, well outside the" + " usual standoff spot"),
                  shoot(
                      10.0,
                      "largest window of any play - the whole depot stack is expected"
                          + " on board"),
                  a("STOW_INTAKE", "fires about 1 s after the shot starts"))),
          new Play(
              "left_trench_return_over_bump",
              "Sweep out the left trench into the neutral-zone fuel line at a cautious pace,"
                  + " then cut back across the raised bump (not the trench) at a brisk controlled"
                  + " speed to a shooting spot behind the hub; repeat with a shallower second"
                  + " lap.",
              List.of(
                  start(4.400, 7.440, 180.0),
                  a("STOW_INTAKE"),
                  a(
                      "INTAKE",
                      "fires almost immediately - down for essentially the whole first" + " lap"),
                  d(5.500, 7.440, 180.0, 2.6),
                  d(7.683, 7.160, 180.0, 2.3, "full stop point follows this in the real auto"),
                  d(
                      7.789,
                      6.000,
                      -178.0,
                      1.125,
                      "cautious collection dip - faster than the 0.35"
                          + " creep used elsewhere but still deliberately slowed"),
                  d(7.826, 4.492, 180.0, 1.0),
                  d(
                      6.067,
                      5.350,
                      -135.2,
                      2.3,
                      "turning back toward the hub, now crossing into" + " the bump's footprint"),
                  d(
                      5.681,
                      5.600,
                      -135.2,
                      2.0,
                      "bump crossing - brisk, explicitly capped at 2.0"
                          + " m/s, not a flat-out sprint"),
                  d(4.363, 5.600, -135.2, 2.0),
                  d(
                      3.153,
                      5.600,
                      -46.2,
                      1.6,
                      "end pose aimed at the hub, inside the alliance" + " zone"),
                  shoot(5.0, ""),
                  a("STOW_INTAKE"),
                  d(
                      2.697,
                      6.383,
                      180.0,
                      1.8,
                      "second lap starts by cutting toward the trench"
                          + " mouth; the intake stays UP through this leg"),
                  d(2.929, 7.450, 180.0, 1.8),
                  d(5.421, 7.500, 180.0, 4.0),
                  a("INTAKE", "fires late this lap, after the initial reposition is done"),
                  d(6.273, 7.206, 180.0, 2.1, "shallower reach than lap 1 (~x=6.3 vs ~x=7.8)"),
                  d(
                      6.479,
                      6.000,
                      -178.0,
                      3.2,
                      "dip is essentially uncapped this lap - no"
                          + " deliberate slowdown, unlike lap 1"),
                  d(6.595, 4.483, -177.2, 1.6),
                  d(5.681, 5.700, -135.2, 2.0, "bump crossing, same 2.0 m/s cap as lap 1"),
                  d(4.363, 5.700, -135.2, 2.0),
                  d(3.153, 5.700, -45.6, 1.6),
                  shoot(2.5, ""),
                  a("STOW_INTAKE"))),
          new Play(
              "right_trench_return_over_bump",
              "Mirror image of left_trench_return_over_bump on the right trench/bump.",
              List.of(
                  start(4.400, 0.629, 0.0),
                  a("STOW_INTAKE"),
                  a("INTAKE"),
                  d(5.500, 0.629, 0.0, 2.6),
                  d(7.683, 0.909, 0.0, 1.5),
                  d(
                      7.789,
                      2.069,
                      0.0,
                      1.125,
                      "cautious collection dip (mirrored: sweeps UP in Y" + " instead of down)"),
                  d(7.826, 3.577, 0.0, 1.0),
                  d(6.067, 2.719, 135.2, 2.3),
                  d(5.681, 2.469, 135.2, 2.0, "bump crossing, capped at 2.0 m/s"),
                  d(4.363, 2.469, 135.2, 2.0),
                  d(3.153, 2.469, 46.2, 1.6, "end pose aimed at the hub"),
                  shoot(5.0, ""),
                  a("STOW_INTAKE"),
                  d(2.697, 0.884, 90.0, 2.5),
                  d(3.879, 0.560, 90.0, 3.1),
                  d(5.421, 0.569, 90.0, 3.7),
                  a("INTAKE", "fires late this lap"),
                  d(6.273, 0.863, 71.0, 1.1, "shallower reach than lap 1"),
                  d(6.479, 2.069, 0.0, 3.1, "dip essentially uncapped this lap"),
                  d(6.595, 3.586, 0.0, 1.6),
                  d(5.681, 2.500, 135.2, 2.0, "bump crossing, same 2.0 m/s cap"),
                  d(4.363, 2.500, 135.2, 2.0),
                  d(3.153, 2.500, 45.6, 1.6),
                  shoot(2.5, ""),
                  a("STOW_INTAKE"))));
}
