package frc.robot.subsystems.drive.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class FlywheelSubsystemIOTalonFX implements FlywheelSubsystemIO {

  private final TalonFX motor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public FlywheelSubsystemIOTalonFX() {
    motor = new TalonFX(Constants.ShooterConstants.MOTOR_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.ShooterConstants.KP;
    config.Slot0.kI = Constants.ShooterConstants.KI;
    config.Slot0.kD = Constants.ShooterConstants.KD;

    config.Slot0.kS = Constants.ShooterConstants.KS;
    config.Slot0.kV = Constants.ShooterConstants.KV;
    config.Slot0.kA = Constants.ShooterConstants.KA;

    motor.getConfigurator().apply(config);

    motor.setNeutralMode(
        Constants.ShooterConstants.MOTOR_COAST ? NeutralModeValue.Coast : NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM =
        motor.getVelocity().getValueAsDouble() * 60.0 / Constants.ShooterConstants.GEAR_RATIO;

    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();

    inputs.supplyCurrentAmps = motor.getStatorCurrent().getValueAsDouble();

    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setTargetRPM(double rpm) {
    double motorRPS = (rpm / 60.0) * Constants.ShooterConstants.GEAR_RATIO;
    motor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
