// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final RobotType robot = RobotType.ALPHABOT;
  public static final boolean tuningMode = true;

  public final class Subsystems {

    public final class Shooter {

      public static final double GEAR_RATIO = 2.0;
      public static final boolean MOTOR_COAST = true;
      public static final boolean SHOOTER_INVERTED = false;

      public static final double DEFAULT_TARGET_RPM = 4500.0;
      public static final double RPM_TOLERANCE = 100.0;

      public static class Id {
        public static final int LEADER_ID = 0;
        public static final int FOLLOWER_ID = 0;
      }

      public static final class ClosedLoop {

        public static final double KP = 1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double KS = 0.0;
        public static final double KV = 0.0;
        public static final double KA = 0.0;
      }

      public static final class CurrentLimits {

        // Supply Limit: Protects the Battery (Prevents Brownouts)
        // Highly recommended for Shooters.
        public static final boolean SUPPLY_ENABLED = false;

        // TODO tune this value
        public static final double SUPPLY_LIMIT_AMPS = 0.0; // Holding limit

        // Stator Limit: Protects the Motor (Prevents Burnout)
        // Keep this HIGH for Shooters to allow fast spin-up.
        public static final boolean STATOR_ENABLED = false;
        public static final double STATOR_LIMIT_AMPS = 0.0;
      }
    }
  }

  // Disables hardware stuff
  public static boolean disableHAL = false;

  public static Mode getMode() {
    return switch (robot) {
      case COMPBOT, ALPHABOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    COMPBOT,
    ALPHABOT,
    SIMBOT
  }
}
