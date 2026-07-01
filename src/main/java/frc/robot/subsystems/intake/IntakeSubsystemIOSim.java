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
  // ponytail: hopper capacity is a guess, tune to the real spindexer
  public static final int FUEL_CAPACITY = 30;

  // Wheel RPM above which the rollers count as actually intaking
  private static final double INTAKING_MIN_RPM = 50.0;

  private final SparkMax motor;
  DCMotor maxGearbox = DCMotor.getKrakenX44Foc(1);

  private final SparkMaxConfig motorConfig;
  private final DoubleSolenoidSim solenoidSim;
  private final ClosedLoopConfig closedLoopConfig;
  private final SparkMaxSim motorSim;
  private final DCMotorSim dcMotorSim;
  private double currentSetpoint = 0.0;
  private double requestedPercentage = 0.0;

  // maple-sim OverTheBumperIntake logic cloned onto FuelPhysicsSim: "running" = the OTB
  // rectangle is extended past the bumper (touch it, get it); heldFuel = pieces in the robot
  private boolean running = false;
  private int heldFuel = 0;

  public IntakeSubsystemIOSim() {
    int motorId = Constants.Subsystems.Intake.Id.Motor.LEFT_ROLLER;
    int forwardChannel = Constants.Subsystems.Intake.Id.Pneumatics.SINGLE_FORWARDS;
    int reverseChannel = Constants.Subsystems.Intake.Id.Pneumatics.SINGLE_REVERSE;

    this.motor = new SparkMax(motorId, MotorType.kBrushless);
    this.solenoidSim =
        new DoubleSolenoidSim(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    this.closedLoopConfig =
        new ClosedLoopConfig()
            .p(Constants.Subsystems.Intake.ClosedLoop.KP)
            .i(Constants.Subsystems.Intake.ClosedLoop.KI)
            .d(Constants.Subsystems.Intake.ClosedLoop.KD)
            .apply(
                new FeedForwardConfig()
                    .kV(Constants.Subsystems.Intake.ClosedLoop.KV)
                    .kS(Constants.Subsystems.Intake.ClosedLoop.KS));
    this.motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(
            Constants.Subsystems.Intake.ROLLER_BREAK
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

  /** True when the OTB intake is extended and collecting (lowered + rollers on). */
  public boolean isIntakeRunning() {
    return running;
  }

  public boolean canHoldMore() {
    return heldFuel < FUEL_CAPACITY;
  }

  /** Called by FuelPhysicsSim when a field ball touches the extended intake. */
  public void addGamePieceToIntake() {
    if (heldFuel < FUEL_CAPACITY) {
      heldFuel++;
    }
  }

  /** Pull one fuel out for shooting; false when empty. */
  public boolean obtainGamePiece() {
    if (heldFuel > 0) {
      heldFuel--;
      return true;
    }
    return false;
  }

  public int getHeldCount() {
    return heldFuel;
  }

  public void setHeldCount(int count) {
    heldFuel = Math.max(0, Math.min(count, FUEL_CAPACITY));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    this.motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(this.dcMotorSim.getAngularVelocityRadPerSec()),
        this.motor.getBusVoltage(),
        0.02);
    this.dcMotorSim.setInputVoltage(this.motorSim.getAppliedOutput() * this.motor.getBusVoltage());
    this.dcMotorSim.update(0.02);

    inputs.velocityRPM = this.motorSim.getVelocity() / Constants.Subsystems.Intake.GEAR_RATIO;
    inputs.appliedVolts = this.motorSim.getAppliedOutput() * this.motor.getBusVoltage();
    inputs.supplyCurrentAmps = this.motorSim.getMotorCurrent();
    inputs.lowered = (this.solenoidSim.get() == DoubleSolenoid.Value.kForward);
    // Extend/retract the OTB intake with the mechanism state. Gate on commanded output as
    // well as measured RPM - the SparkMax sim can report 0 RPM when bus voltage sim is off.
    boolean rollersOn =
        this.requestedPercentage > 0.05
            || this.currentSetpoint > 0
            || inputs.velocityRPM > INTAKING_MIN_RPM;
    this.running = inputs.lowered && rollersOn;
    inputs.connected = true;
    inputs.setpointRPM = this.currentSetpoint;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - inputs.setpointRPM)
            <= Constants.Subsystems.Intake.RPM_TOLERANCE;
    inputs.requestedPercentage = this.requestedPercentage;
  }

  @Override
  public void setTargetRPM(double rpm) {
    this.currentSetpoint = rpm;
    this.requestedPercentage = 0.0;
    this.motor
        .getClosedLoopController()
        .setSetpoint(rpm * Constants.Subsystems.Intake.GEAR_RATIO, ControlType.kVelocity);
  }

  @Override
  public void setVoltage(double volts) {
    this.requestedPercentage = 0.0;
    this.motor.setVoltage(volts);
  }

  @Override
  public void setPercentage(double percentage) {
    this.requestedPercentage = percentage;
    this.motor.set(percentage);
  }

  @Override
  public void setLowered(boolean lowered) {
    this.solenoidSim.set(lowered ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void stop() {
    setVoltage(0);
    this.currentSetpoint = 0.0;
    this.requestedPercentage = 0.0;
  }
}
