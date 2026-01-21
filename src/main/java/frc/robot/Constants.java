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
  public static final boolean tuningMode = false;

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
          public static final int ROLLER = 0;
        }

        public static class Pneumatics {
          public static final int REVERSE = 0;
          public static final int FORWARDS = 0;
        }
      }

      public static class ClosedLoop {
        public static final int KP = 0;
        public static final int KI = 0;
        public static final int KD = 0;
        public static final int KS = 0;
        public static final int KG = 0;
      }
    }
  }
}
