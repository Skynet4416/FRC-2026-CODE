// spotless:off
package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo.
 * DO NOT MODIFY THIS FILE YOURSELF; instead, change these values
 * in the Choreo GUI.
 */
public final class ChoreoVars {
    public static final LinearVelocity BallIntakeSpeed = Units.MetersPerSecond.of(1.5);
    public static final LinearVelocity maxVelocity = Units.MetersPerSecond.of(3);

    public static final class Poses {
        public static final Pose2d hubCenterStart = new Pose2d(3.58, 4.035, Rotation2d.fromRadians(0));
        public static final Pose2d launchLeftTower = new Pose2d(2.605, 5.117, Rotation2d.fromRadians(0));
        public static final Pose2d launchRightTower = new Pose2d(2.605, 2.376, Rotation2d.fromRadians(0));
        public static final Pose2d leftClimbEdge = new Pose2d(1.06, 4.617, Rotation2d.fromRadians(0));
        public static final Pose2d leftThroughDepot = new Pose2d(0.5, 7.039, Rotation2d.fromRadians(0));
        public static final Pose2d leftTrench = new Pose2d(3.58, 7.232, Rotation2d.fromRadians(0));
        public static final Pose2d outpost = new Pose2d(0.7, 0.65, Rotation2d.fromRadians(3.1415927));
        public static final Pose2d rightClimbEdge = new Pose2d(1.06, 2.876, Rotation2d.fromRadians(1.5707963));
        public static final Pose2d rightThroughDepot = new Pose2d(0.5, 4.888, Rotation2d.fromRadians(0));
        public static final Pose2d rightTrench = new Pose2d(3.58, 0.837, Rotation2d.fromRadians(0));
    }
}
// spotless:on
