package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

public class HoodSubsystemIOTalonFX implements HoodSubsystemIO {

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final Alert motorDisconnected;
  private double targetAngle = 0.0;

  public HoodSubsystemIOTalonFX() {
    motor = new TalonFX(Constants.Subsystems.Shooter.Hood.Id.MOTOR_ID);
    motorDisconnected = new Alert("Hood motor disconnected!", AlertType.kWarning);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Shooter.Hood.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Shooter.Hood.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Shooter.Hood.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Shooter.Hood.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Shooter.Hood.ClosedLoop.KV;
    config.Slot0.kA = Constants.Subsystems.Shooter.Hood.ClosedLoop.KA;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.SUPPLY_LIMIT_AMPS;

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted =
        Constants.Subsystems.Shooter.Hood.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.angle =
        (motor.getPosition().getValueAsDouble() / Constants.Subsystems.Shooter.Hood.GEAR_RATIO)
            * 360.0;
    inputs.velocityRPM =
        motor.getVelocity().getValueAsDouble()
            * 60.0
            / Constants.Subsystems.Shooter.Hood.GEAR_RATIO;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.connected = motor.isConnected();
    inputs.targetAngle = targetAngle;
    inputs.atSetpoint = Math.abs(targetAngle - inputs.angle) <= HoodSubsystem.toleranceDeg.get();

    motorDisconnected.set(!inputs.connected);
  }

  @Override
  public void setTargetAngle(double degrees) {
    targetAngle = degrees;
    motor.setControl(
        positionRequest.withPosition(
            (degrees / 360.0) * Constants.Subsystems.Shooter.Hood.GEAR_RATIO));
  }

  @Override
  public void setTargetAngleWithVelocity(double degrees, double velocityRPM) {
    targetAngle = degrees;
    motor.setControl(
        positionRequest
            .withPosition((degrees / 360.0) * Constants.Subsystems.Shooter.Hood.GEAR_RATIO)
            .withVelocity(velocityRPM));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void set(double percentOutput) {
    motor.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setAngle(double degrees) {
    motor.setPosition((degrees / 360.0) * Constants.Subsystems.Shooter.Hood.GEAR_RATIO);
  }
}
