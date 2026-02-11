package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    leaderMotor = new TalonFX(Constants.Subsystems.Shooter.Id.LEADER_ID);
    followerMotor = new TalonFX(Constants.Subsystems.Shooter.Id.FOLLOWER_ID);

    leaderDisconnected = new Alert("Flywheel leader motor disconnected!", AlertType.kWarning);
    followerDisconnected = new Alert("Flywheel follower motor disconnected!", AlertType.kWarning);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Shooter.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Shooter.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Shooter.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Shooter.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Shooter.ClosedLoop.KV;
    config.Slot0.kA = Constants.Subsystems.Shooter.ClosedLoop.KA;

    // 1. Supply Limit (Battery Safety)
    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Shooter.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Shooter.CurrentLimits.SUPPLY_LIMIT_AMPS;

    // 2. Stator Limit (Motor Safety)
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Shooter.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Shooter.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted =
        Constants.Subsystems.Shooter.SHOOTER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    leaderMotor.getConfigurator().apply(config);

    leaderMotor.setNeutralMode(
        Constants.Subsystems.Shooter.MOTOR_COAST ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    followerMotor.setControl(
        new Follower(Constants.Subsystems.Shooter.Id.LEADER_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM =
        leaderMotor.getVelocity().getValueAsDouble()
            * 60.0
            / Constants.Subsystems.Shooter.GEAR_RATIO;

    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();

    inputs.supplyCurrentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();

    inputs.tempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.connected = leaderMotor.isConnected();

    inputs.setpointRPM = targetRPM;

    leaderDisconnected.set(!inputs.connected);
    followerDisconnected.set(!followerMotor.isConnected());
  }

  @Override
  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
    double motorRPS = (rpm / 60.0) * Constants.Subsystems.Shooter.GEAR_RATIO;
    if (rpm < targetRPM) {
      leaderMotor.set(1);
    } else {
      leaderMotor.setControl(velocityRequest.withVelocity(motorRPS));
    }
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(voltageRequest.withOutput(volts));
  }
}
