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
  public static final RobotType robot = RobotType.COMPBOT;
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

  public static class Subsystems {

    public static final class Drive {
      public static final int PIGEON_ID = 10;

      public static final class FrontLeft {
        public static final int DRIVE_ID = 11;
        public static final int STEER_ID = 12;
        public static final int ENCODER_ID = 13;
      }

      public static final class FrontRight {
        public static final int DRIVE_ID = 14;
        public static final int STEER_ID = 15;
        public static final int ENCODER_ID = 16;
      }

      public static final class BackLeft {
        public static final int DRIVE_ID = 17;
        public static final int STEER_ID = 18;
        public static final int ENCODER_ID = 19;
      }

      public static final class BackRight {
        public static final int DRIVE_ID = 20;
        public static final int STEER_ID = 21;
        public static final int ENCODER_ID = 22;
      }
    }

    public static final class Shooter {
      public static final class Flywheel {
        public static final double GEAR_RATIO = 72 / 56;
        public static final boolean MOTOR_COAST = true;
        public static final boolean SHOOTER_INVERTED = true;

        public static final double DEFAULT_TARGET_RPM = 1000.0;
        public static final double RPM_TOLERANCE = 100.0;

        public static class Id {
          public static final int LEADER_ID = 40;
          public static final int FOLLOWER_ID = 41;
        }

        public static final class ClosedLoop {

          public static final double KP = 10;
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
        public static final double STALL_CURRENT_LIMIT = 20.0; // Amps
        public static final double MAX_ANGLE_DEG = 68.0;
        public static final double MIN_ANGLE_DEG = 0.0;
        public static final double HOMING_VOLTS = -2.0;

        public static class Id {
          public static final int MOTOR_ID = 42;
        }

        public static final class ClosedLoop {
          public static final double KP = 10;
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
      public static final double RPM_TOLERANCE = 20.0;

      public static class Id {
        public static class Motor {
          public static final int INDEXER = 43;
          public static final int SHOOTER_INDEXER = 44;
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
        public static final double SUPPLY_LIMIT_AMPS = 30; // Holding limit

        // Stator Limit: Protects the Motor (Prevents Burnout)
        // Keep this HIGH for Shooters to allow fast spin-up.
        public static final boolean STATOR_ENABLED = false;
        public static final double STATOR_LIMIT_AMPS = 0.0;
      }
    }

    public static class Intake {
      public static final boolean ROLLER_BREAK = true;
      public static final double RPM_TOLERANCE = 20.0;

      public static class Id {
        public static class Motor {
          public static final int LEFT_ROLLER = 30;
          public static final int RIGHT_ROLLER = 31;
        }

        public static class Pneumatics {
          public static final int LEFT_REVERSE = 1; //
          public static final int LEFT_FORWARDS = 15;
          public static final int RIGHT_REVERSE = 2; //
          public static final int RIGHT_FORWARDS = 14;
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

    public static class Climber {
      public static class Id {
        public static final int LEFT_MOTOR = 50;
        public static final int RIGHT_MOTOR = 51;
        public static final int THIRD_MOTOR = 52;
      }
    }
  }
}
