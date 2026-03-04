package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SpindexerSubsystemIOSparkMaxSim implements SpindexerSubsystemIO {
  private final SparkMax motor;
  DCMotor maxGearbox = DCMotor.getKrakenX44Foc(1);

  private final SparkMaxConfig motorConfig;
  private final ClosedLoopConfig closedLoopConfig;
  private final SparkMaxSim motorSim;
  private final DCMotorSim dcMotorSim;
  private double currentSetpoint = 0.0;

  public SpindexerSubsystemIOSparkMaxSim() {
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

    // Sim related
    this.motorSim = new SparkMaxSim(motor, maxGearbox);
    this.dcMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(maxGearbox, 0.005, 1.0), maxGearbox);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    this.motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(this.dcMotorSim.getAngularVelocityRadPerSec()),
        this.motor.getBusVoltage(),
        0.02);
    this.dcMotorSim.setInputVoltage(this.motorSim.getAppliedOutput() * this.motor.getBusVoltage());
    this.dcMotorSim.update(0.02);

    inputs.velocityRPM = this.motorSim.getVelocity();
    inputs.appliedVolts = this.motorSim.getAppliedOutput() * this.motor.getBusVoltage();
    inputs.supplyCurrentAmps = this.motorSim.getMotorCurrent();
    inputs.connected = true;
    inputs.setpointRPM = this.currentSetpoint;
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
}
