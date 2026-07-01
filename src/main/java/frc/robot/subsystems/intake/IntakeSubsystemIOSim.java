package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

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
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeSubsystemIOSim implements IntakeSubsystemIO {
  // ponytail: hopper capacity is a guess, tune to the real spindexer
  public static final int FUEL_CAPACITY = 30;

  // am-3462 2in compliant wheel (black 60A)
  private static final double WHEEL_RADIUS_METERS = 0.0254;
  // Wheel RPM above which the rollers count as actually intaking
  private static final double INTAKING_MIN_RPM = 50.0;

  private final SparkMax motor;
  DCMotor maxGearbox = DCMotor.getKrakenX44Foc(1);

  private final SparkMaxConfig motorConfig;
  private final DoubleSolenoidSim solenoidSim;
  private final ClosedLoopConfig closedLoopConfig;
  private final SparkMaxSim motorSim;
  private final DCMotorSim dcMotorSim;
  private final IntakeSimulation intakeSimulation;
  private double currentSetpoint = 0.0;
  private double requestedPercentage = 0.0;
  private double wheelRpm = 0.0;

  public IntakeSubsystemIOSim(AbstractDriveTrainSimulation driveTrain) {
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

    // From CAD: mouth spans 708mm (full frame width), rollers reach ~260mm past the front
    // bumper when lowered. Registers itself with the SimulatedArena as a collision fixture.
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrain,
            Meters.of(0.708),
            Meters.of(0.26),
            IntakeSimulation.IntakeSide.FRONT,
            FUEL_CAPACITY);
    // FuelPhysicsSim owns the visible field balls and feeds pickups in via
    // addGamePieceToIntake(); never collect maple-sim's invisible arena fuel (double counting).
    this.intakeSimulation.setCustomIntakeCondition(gamePiece -> false);
  }

  /** Wheel surface speed in m/s. Positive = pulling fuel in. */
  public double getRollerSurfaceSpeedMps() {
    return wheelRpm * 2.0 * Math.PI * WHEEL_RADIUS_METERS / 60.0;
  }

  public boolean canHoldMore() {
    return intakeSimulation.getGamePiecesAmount() < FUEL_CAPACITY;
  }

  /** Called by FuelPhysicsSim when a field ball is swallowed. */
  public void addGamePieceToIntake() {
    intakeSimulation.addGamePieceToIntake();
  }

  /** Pull one fuel out for shooting; false when empty. */
  public boolean obtainGamePiece() {
    return intakeSimulation.obtainGamePieceFromIntake();
  }

  public int getHeldCount() {
    return intakeSimulation.getGamePiecesAmount();
  }

  public void setHeldCount(int count) {
    intakeSimulation.setGamePiecesCount(count);
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
    this.wheelRpm = inputs.velocityRPM;

    // Extend/retract the maple-sim collision fixture with the real mechanism state
    if (inputs.lowered && this.wheelRpm > INTAKING_MIN_RPM) {
      this.intakeSimulation.startIntake();
    } else {
      this.intakeSimulation.stopIntake();
    }
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
