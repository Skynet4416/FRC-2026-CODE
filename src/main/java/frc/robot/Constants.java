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

  public static class Subsystem {
    public static class Intake {
      public static final boolean ROLLER_BREAK = true;

      public static class Id {
        public static class Motor {
          public static final int LEFT_ROLLER = 0;
          public static final int RIGHT_ROLLER = 1;
        }

        public static class Pneumatics {
          public static final int LEFT_REVERSE = 1;
          public static final int LEFT_FORWARDS = 0;
          public static final int RIGHT_REVERSE = 3;
          public static final int RIGHT_FORWARDS = 2;
        }
      }

      public static class ClosedLoop {
        public static final double KP = 0.003;
        public static final double KI = 0.0001;
        public static final double KD = 0.00001;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
      }

      public static final class CurrentLimits {

        // Supply Limit: Protects the Battery (Prevents Brownouts)
        // Highly recommended for Shooters.
        public static final boolean SUPPLY_ENABLED = true;

        // TODO tune this value
        public static final double SUPPLY_LIMIT_AMPS = 40; // Holding limit

        // Stator Limit: Protects the Motor (Prevents Burnout)
        // Keep this HIGH for Shooters to allow fast spin-up.
        public static final boolean STATOR_ENABLED = false;
        public static final double STATOR_LIMIT_AMPS = 0.0;
      }
    }
  }
}
