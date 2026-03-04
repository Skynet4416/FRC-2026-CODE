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
  public static final double loopPeriodSecs = 0.02;

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

    public static final class Shooter {
      public static final class Flywheel {
        public static final double GEAR_RATIO = 72 / 56;
        public static final boolean MOTOR_COAST = true;
        public static final boolean SHOOTER_INVERTED = false;

        public static final double DEFAULT_TARGET_RPM = 1000.0;
        public static final double RPM_TOLERANCE = 20.0;

        public static class Id {
          public static final int LEADER_ID = 12;
          public static final int FOLLOWER_ID = 17;
        }

        public static final class ClosedLoop {

          public static final double KP = 0.1;
          public static final double KI = 0.0;
          public static final double KD = 0.0;

          public static final double KS = 0.0;
          public static final double KV = 0.12;
          public static final double KA = 0.0;
        }

        public static final class CurrentLimits {

          // Supply Limit: Protects the Battery (Prevents Brownouts)
          // Highly recommended for Shooters.
          public static final boolean SUPPLY_ENABLED = true;

          // TODO tune this value
          public static final double SUPPLY_LIMIT_AMPS = 40.0; // Holding limit

          // Stator Limit: Protects the Motor (Prevents Burnout)
          // Keep this HIGH for Shooters to allow fast spin-up.
          public static final boolean STATOR_ENABLED = false;
          public static final double STATOR_LIMIT_AMPS = 0.0;
        }
      }

      public static final class Hood {
        public static final double GEAR_RATIO = 875.0 / 38.0;
        public static final boolean INVERTED = false;
        public static final double ZERO_SPEED = -0.1; // Duty Cycle
        public static final double STALL_CURRENT_LIMIT = 20.0; // Amps
        public static final double MAX_ANGLE_DEG = 45.0;
        public static final double MIN_ANGLE_DEG = 0.0;
        public static final double HOMING_VOLTS = -2.0;
        public static final double HOMING_VELOCITY_THRESHOLD_RPM = 5.0;

        public static class Id {
          public static final int MOTOR_ID = 20; // TODO: Set ID
        }

        public static final class ClosedLoop {
          public static final double KP = 2;
          public static final double KI = 0.0;
          public static final double KD = 0.0;
          public static final double KS = 0.0;
          public static final double KV = 0.0;
          public static final double KA = 0.0;
        }

        public static final class CurrentLimits {
          public static final boolean SUPPLY_ENABLED = true;
          public static final double SUPPLY_LIMIT_AMPS = 40.0;
          public static final boolean STATOR_ENABLED = false;
          public static final double STATOR_LIMIT_AMPS = 0.0;
        }
      }
    }

    public static class Spindexer {
      public static final boolean ROLLER_BREAK = true;

      public static class Id {
        public static class Motor {
          public static final int INDEXER = 0;
        }
      }

      public static class ClosedLoop {
        public static final double KP = 0.003;
        public static final double KI = 0.0001;
        public static final double KD = 0.00001;
        public static final double KS = 0.0;
        public static final double KV = 0.0;
      }

      public static class TalonFXClosedLoop {
        public static final double KP = 8.0;
        public static final double KI = 0.0;
        public static final double KD = 0.0;
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
