// Copyright (c) 2026 Skynet 4416
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

/**
 * The REBUILT (2026) game, in plain language, for the AI control bridge's {@code GameBrief} topic.
 *
 * <p>Sourced from the official 2026 game manual (Team Update 22) and cross-checked against this
 * repo's own field geometry; the full sourcing notes live alongside the task that produced this
 * text. This robot has no climber, so the End Game climbing line is context for reading the field,
 * not a plan - no climb action is registered anywhere in this codebase.
 */
public final class AIGameBrief {
  private AIGameBrief() {}

  public static final String TEXT =
      """
      A REBUILT match lasts 2 minutes 40 seconds: a 20-second autonomous period where robots \
      score without driver input, then 2 minutes 20 seconds of driver-controlled teleop. Teleop \
      breaks into a 10-second Transition Shift, four 25-second Alliance Shifts, and a final \
      30-second End Game. Both alliances' hubs are active during auto, Transition, and End Game. \
      During the four Alliance Shifts only one alliance's hub is active at a time: whichever \
      alliance scores more fuel in auto has its own hub go inactive first, the other alliance is \
      active instead, and they swap every shift after that. Check whether your hub is active \
      before committing to a shot - fuel scored in an inactive hub earns nothing.

      The field is a carpeted rectangle a little over 16 meters long and about 8 meters wide, \
      mirrored end to end. Each alliance's zone sits against its own wall, holding a climbable \
      tower with three rungs and a depot (a small penned area preloaded with fuel). Your hub sits \
      at the far edge of your alliance zone, flanked by two raised ramp-like bumps, facing a \
      neutral zone in the middle of the field where loose fuel is piled at the start of the match. \
      A low trench passage under a structure on each side lets fuel move between the alliance \
      zone and neutral zone. An outpost at each end of the field is where your human player feeds \
      fuel onto the field, or receives fuel you push back to them.

      Fuel is the only game piece: a foam ball about six inches across. You score by launching it \
      up into your hub's opening near the top of the structure (roughly six feet up); it must \
      fall down through that opening to count, and each fuel scored in an active hub is worth one \
      point. You may hold and carry any number of fuel at once. You can only legally shoot into \
      your hub while your robot is at least partly inside your own alliance zone - not from the \
      neutral zone or the opponent's side. Fuel comes from a preload, your depot, the neutral \
      zone pile, or your outpost.

      A good cycle: while your hub is inactive, collect fuel from the neutral zone, your depot, \
      or your outpost; get back into your alliance zone; then fire into the hub the moment it \
      goes active, repeating as fast as you can reload. Once End Game starts, both hubs go active \
      again - climbing the tower then earns a bonus worth far more than a single shot, a strong \
      closing play if you have the mechanism for it.
      """;
}
