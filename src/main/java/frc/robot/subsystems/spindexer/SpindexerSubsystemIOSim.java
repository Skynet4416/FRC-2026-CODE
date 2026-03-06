package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SpindexerSubsystemIOSim implements SpindexerSubsystemIO {

  private final TalonFX motor;
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Spindexer motor disconnected!", AlertType.kWarning);
  private double currentSetpoint = 0.0;

  private final DCMotor maxGearbox = DCMotor.getNEO(1);
  private final DCMotorSim dcMotorSim;
  private final TalonFXSimState motorSim;

  public SpindexerSubsystemIOSim() {
    motor = new TalonFX(Constants.Subsystems.Spindexer.Id.Motor.INDEXER);

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

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Spindexer.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Spindexer.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Default
    config.MotorOutput.NeutralMode =
        Constants.Subsystems.Spindexer.ROLLER_BREAK
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;

    motor.getConfigurator().apply(config);

    // Sim related
    this.motorSim = motor.getSimState();
    this.dcMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(maxGearbox, 0.005, 1.0), maxGearbox);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    // 0.02 is loop time
    this.dcMotorSim.setInputVoltage(this.motorSim.getMotorVoltage());
    this.dcMotorSim.update(0.02);

    this.motorSim.setRawRotorPosition(
        Units.radiansToRotations(this.dcMotorSim.getAngularPositionRad()));
    this.motorSim.setRotorVelocity(
        Units.radiansToRotations(this.dcMotorSim.getAngularVelocityRadPerSec()));
    this.motorSim.setSupplyVoltage(12.0);

    inputs.velocityRPM = motor.getVelocity().getValueAsDouble() * 60.0;
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = motor.getStatorCurrent().getValueAsDouble();

    inputs.connected = motorConnectedDebouncer.calculate(motor.isConnected());
    motorDisconnectedAlert.set(!inputs.connected);
    inputs.setpointRPM = this.currentSetpoint;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - inputs.setpointRPM)
            <= Constants.Subsystems.Spindexer.RPM_TOLERANCE;
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.currentSetpoint = rpm;
    motor.setControl(velocityVoltageRequest.withVelocity(rpm / 60.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    setVoltage(0);
    this.currentSetpoint = 0.0;
  }

  @Override
  public void set(double percentage) {
    motor.set(percentage);
  }
}
