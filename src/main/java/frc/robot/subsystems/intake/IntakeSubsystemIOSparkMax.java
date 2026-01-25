package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class IntakeSubsystemIOSparkMax implements IntakeSubsystemIO {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final DoubleSolenoid solenoid;
  private final ClosedLoopConfig closedLoopConfig;

  public IntakeSubsystemIOSparkMax() {
    this.motor = new SparkMax(Constants.Subsystem.Intake.Id.Motor.ROLLER, MotorType.kBrushless);
    this.solenoid =
        new DoubleSolenoid(
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
                    .kG(Constants.Subsystem.Intake.ClosedLoop.KG)
                    .kS(Constants.Subsystem.Intake.ClosedLoop.KS));
    this.motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(
            Constants.Subsystem.Intake.ROLLER_BREAK
                ? SparkBaseConfig.IdleMode.kBrake
                : SparkBaseConfig.IdleMode.kCoast)
        .apply(closedLoopConfig);

    this.motor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRPM = this.motor.getEncoder().getVelocity();
    //TODO: add the updates for the other inputs
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
    this.solenoid.set(lowered ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }
}
