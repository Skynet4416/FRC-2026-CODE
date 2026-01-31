package frc.robot.subsystems.intake;

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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.Constants;

public class IntakeSubsystemIOSim implements IntakeSubsystemIO {
  private final SparkMax motor;
  DCMotor maxGearbox = DCMotor.getNEO(1);

  private final SparkMaxConfig motorConfig;
  private final DoubleSolenoidSim solenoidSim;
  private final ClosedLoopConfig closedLoopConfig;
  private final SparkMaxSim motorSim;
  private final DCMotorSim dcMotorSim;

  public IntakeSubsystemIOSim() {
    this.motor = new SparkMax(Constants.Subsystem.Intake.Id.Motor.ROLLER, MotorType.kBrushless);
    this.solenoidSim =
        new DoubleSolenoidSim(
            PneumaticsModuleType.REVPH,
            Constants.Subsystem.Intake.Id.Pneumatics.FORWARDS,
            Constants.Subsystem.Intake.Id.Pneumatics.REVERSE);
    this.closedLoopConfig =
        new ClosedLoopConfig()
            .p(Constants.Subsystem.Intake.ClosedLoop.KP)
            .i(Constants.Subsystem.Intake.ClosedLoop.KI)
            .d(Constants.Subsystem.Intake.ClosedLoop.KD)
            .apply(
                new FeedForwardConfig()
                    .kV(Constants.Subsystem.Intake.ClosedLoop.KV)
                    .kS(Constants.Subsystem.Intake.ClosedLoop.KS));
    this.motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(
            Constants.Subsystem.Intake.ROLLER_BREAK
                ? SparkBaseConfig.IdleMode.kBrake
                : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .apply(closedLoopConfig);

    this.motor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.setLowered(false);

    // Sim related
    this.motorSim = new SparkMaxSim(motor, maxGearbox);
    this.dcMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(maxGearbox, 0.005, 1.0), maxGearbox);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    this.motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(this.dcMotorSim.getAngularVelocityRadPerSec()),
        this.motor.getBusVoltage(),
        0.02);
    this.dcMotorSim.setInputVoltage(this.motorSim.getAppliedOutput() * this.motor.getBusVoltage());
    this.dcMotorSim.update(0.02);

    inputs.velocityRPM = this.motorSim.getVelocity();
    inputs.appliedVolts = this.motorSim.getAppliedOutput() * this.motor.getBusVoltage();
    inputs.supplyCurrentAmps = this.motorSim.getMotorCurrent();
    inputs.lowered = (this.solenoidSim.get() == DoubleSolenoid.Value.kForward);
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.motor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
  }

  @Override
  public void setVoltage(double volts) {
    this.motor.setVoltage(volts);
  }

  @Override
  public void setLowered(boolean lowered) {
    this.solenoidSim.set(lowered ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
}
