package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

public class FlywheelSubsystemIOTalonFX implements FlywheelSubsystemIO {

  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final CANdle candle = new CANdle(6);

  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
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
    inputs.velocityRPM = leaderMotor.getVelocity().getValueAsDouble() * 60.0;

    inputs.appliedVolts = leaderMotor.getMotorVoltage().getValueAsDouble();

    inputs.supplyCurrentAmps = leaderMotor.getStatorCurrent().getValueAsDouble();

    inputs.tempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
    inputs.connected = leaderMotor.isConnected();

    inputs.setpointRPM = targetRPM;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - targetRPM)
                <= Constants.Subsystems.Shooter.Flywheel.RPM_TOLERANCE
            && inputs.velocityRPM > 200;

    inputs.rotations = leaderMotor.getRotorPosition().getValueAsDouble();
    leaderDisconnected.set(!inputs.connected);
    followerDisconnected.set(!followerMotor.isConnected());

    double charge = (leaderMotor.getSupplyVoltage().getValueAsDouble() - 8) / 4.0;
    candle.setControl(
        new SolidColor(0, 64)
            .withColor(new RGBWColor((int) ((1 - charge) * 255), (int) (charge * 255), 0)));
  }

  @Override
  public void setTargetRPM(double rpm) {
    targetRPM = rpm * Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO;
    double targetMotorRPS = (targetRPM / 60.0);
    leaderMotor.setControl(velocityRequest.withVelocity(targetMotorRPS));
  }

  @Override
  public void setTargetRADS(double radiansPerSecond) {
    setTargetRPM(Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond));
  }

  @Override
  public void configPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    var slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    leaderMotor.getConfigurator().apply(slot0);
  }

  @Override
  public void setVoltage(double volts) {
    leaderMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    setVoltage(0);
    this.targetRPM = 0.0;
  }
}
