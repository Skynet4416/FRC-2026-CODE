package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

public class SpindexerSubsystemIOTalonFX implements SpindexerSubsystemIO {

  private final TalonFX motorMain;
  private final TalonFX motorSub;
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Spindexer motor disconnected!", AlertType.kWarning);
  private double currentSetpoint = 0.0;
  private double requestedPercentage = 0.0;

  public SpindexerSubsystemIOTalonFX() {
    motorMain = new TalonFX(Constants.Subsystems.Spindexer.Id.Motor.INDEXER);
    motorSub = new TalonFX(Constants.Subsystems.Spindexer.Id.Motor.INDEXER_SUB);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Spindexer.TalonFXClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Spindexer.TalonFXClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Spindexer.TalonFXClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Spindexer.TalonFXClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Spindexer.TalonFXClosedLoop.KV;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Spindexer.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Spindexer.CurrentLimits.SUPPLY_LIMIT_AMPS;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 15.0;

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Spindexer.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Spindexer.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Default
    config.MotorOutput.NeutralMode =
        Constants.Subsystems.Spindexer.ROLLER_BREAK
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    motorMain.getConfigurator().apply(config);
    motorSub.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.velocityRPM = motorMain.getVelocity().getValueAsDouble() * 60.0;
    inputs.appliedVolts = motorMain.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motorMain.getStatorCurrent().getValueAsDouble();

    inputs.connected = motorConnectedDebouncer.calculate(motorMain.isConnected());
    motorDisconnectedAlert.set(!inputs.connected);
    inputs.setpointRPM = this.currentSetpoint;
    inputs.requestedPercentage = this.requestedPercentage;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - inputs.setpointRPM)
            <= Constants.Subsystems.Spindexer.RPM_TOLERANCE;
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.currentSetpoint = rpm;
    this.requestedPercentage = 0.0;
    motorMain.setControl(velocityVoltageRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setVoltage(double volts) {
    this.requestedPercentage = 0.0;
    motorMain.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void set(double percentage) {
    this.requestedPercentage = percentage;
    motorMain.set(percentage);
  }

  @Override
  public void setSub(double percentage) {
    motorSub.set(Math.abs(percentage) / 2.f);
  }

  @Override
  public void stop() {
    setVoltage(0);
    setSub(0);
    this.currentSetpoint = 0.0;
    this.requestedPercentage = 0.0;
  }
}
