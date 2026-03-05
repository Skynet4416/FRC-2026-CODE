package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

public class SpindexerSubsystemIOSparkMax implements SpindexerSubsystemIO {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final ClosedLoopConfig closedLoopConfig;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Spindexer motor disconnected!", AlertType.kWarning);
  private double currentSetpoint = 0.0;

  public SpindexerSubsystemIOSparkMax() {
    this.motor =
        new SparkMax(Constants.Subsystems.Spindexer.Id.Motor.INDEXER, MotorType.kBrushless);
    this.closedLoopConfig =
        new ClosedLoopConfig()
            .p(Constants.Subsystems.Spindexer.ClosedLoop.KP)
            .i(Constants.Subsystems.Spindexer.ClosedLoop.KI)
            .d(Constants.Subsystems.Spindexer.ClosedLoop.KD)
            .apply(
                new FeedForwardConfig()
                    .kV(Constants.Subsystems.Spindexer.ClosedLoop.KV)
                    .kS(Constants.Subsystems.Spindexer.ClosedLoop.KS));
    this.motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(
            Constants.Subsystems.Spindexer.ROLLER_BREAK
                ? SparkBaseConfig.IdleMode.kBrake
                : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .apply(closedLoopConfig);

    this.motor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.velocityRPM = this.motor.getEncoder().getVelocity();
    inputs.appliedVolts = this.motor.getAppliedOutput() * this.motor.getBusVoltage();
    inputs.supplyCurrentAmps = this.motor.getOutputCurrent();

    inputs.connected = motorConnectedDebouncer.calculate(this.motor.getFirmwareVersion() != 0);
    motorDisconnectedAlert.set(!inputs.connected);
    inputs.setpointRPM = this.currentSetpoint;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - inputs.setpointRPM)
            <= Constants.Subsystems.Spindexer.RPM_TOLERANCE;
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.currentSetpoint = rpm;
    this.motor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void setVoltage(double volts) {
    this.motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0);
    this.currentSetpoint = 0.0;
  }
}
