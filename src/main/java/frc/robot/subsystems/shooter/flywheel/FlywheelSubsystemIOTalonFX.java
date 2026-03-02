package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

public class FlywheelSubsystemIOTalonFX implements FlywheelSubsystemIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Alert leaderDisconnected;
  private final Alert followerDisconnected;

  private double targetRPM = 0.0;

  public FlywheelSubsystemIOTalonFX() {
    leaderMotor = new TalonFX(Constants.Subsystems.Shooter.Flywheel.Id.LEADER_ID);
    followerMotor = new TalonFX(Constants.Subsystems.Shooter.Flywheel.Id.FOLLOWER_ID);

    leaderDisconnected = new Alert("Flywheel leader motor disconnected!", AlertType.kWarning);
    followerDisconnected = new Alert("Flywheel follower motor disconnected!", AlertType.kWarning);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KV;
    config.Slot0.kA = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KA;

    // 1. Supply Limit (Battery Safety)
    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.SUPPLY_LIMIT_AMPS;

    // 2. Stator Limit (Motor Safety)
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted =
        Constants.Subsystems.Shooter.Flywheel.SHOOTER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    leaderMotor.getConfigurator().apply(config);

    leaderMotor.setNeutralMode(
        Constants.Subsystems.Shooter.Flywheel.MOTOR_COAST
            ? NeutralModeValue.Coast
            : NeutralModeValue.Brake);
    followerMotor.setControl(
        new Follower(
            Constants.Subsystems.Shooter.Flywheel.Id.LEADER_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM =
        leaderMotor.getVelocity().getValueAsDouble()
            * 60.0
            / Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO;

    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();

    inputs.supplyCurrentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();

    inputs.tempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.connected = leaderMotor.isConnected();

    inputs.setpointRPM = targetRPM;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - targetRPM)
            <= Constants.Subsystems.Shooter.Flywheel.RPM_TOLERANCE;

    leaderDisconnected.set(!inputs.connected);
    followerDisconnected.set(!followerMotor.isConnected());
  }

  @Override
  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
    double targetMotorRPS = (rpm / 60.0) * Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO;
    leaderMotor.setControl(velocityRequest.withVelocity(targetMotorRPS));
  }

  @Override
  public void setTargetRADS(double radiansPerSecond) {
    setTargetRPM(Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond));
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(voltageRequest.withOutput(volts));
  }
}
