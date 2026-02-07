package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class IntakeSubsystemIOTalonFX implements IntakeSubsystemIO {

  private final TalonFX motor;
  private final DoubleSolenoid solenoid;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Intake motor disconnected!", AlertType.kWarning);
  private double currentSetpoint = 0.0;

  public IntakeSubsystemIOTalonFX() {
    motor = new TalonFX(Constants.Subsystem.Intake.Id.Motor.ROLLER);
    solenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.Subsystem.Intake.Id.Pneumatics.FORWARDS,
            Constants.Subsystem.Intake.Id.Pneumatics.REVERSE);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystem.Intake.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystem.Intake.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystem.Intake.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystem.Intake.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystem.Intake.ClosedLoop.KV;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystem.Intake.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystem.Intake.CurrentLimits.SUPPLY_LIMIT_AMPS;

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystem.Intake.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystem.Intake.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Default
    config.MotorOutput.NeutralMode =
        Constants.Subsystem.Intake.ROLLER_BREAK ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    motor.getConfigurator().apply(config);

    setLowered(false);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRPM = motor.getVelocity().getValueAsDouble() * 60.0;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.lowered = (solenoid.get() == DoubleSolenoid.Value.kForward);

    inputs.connected = motorConnectedDebouncer.calculate(motor.isConnected());
    motorDisconnectedAlert.set(!inputs.connected);
    inputs.setpointRPM = this.currentSetpoint;
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.currentSetpoint = rpm;
    motor.setControl(velocityRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setLowered(boolean lowered) {
    solenoid.set(lowered ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
}
