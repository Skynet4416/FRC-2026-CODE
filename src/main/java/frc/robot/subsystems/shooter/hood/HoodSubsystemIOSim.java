package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class HoodSubsystemIOSim implements HoodSubsystemIO {

  private final TalonFX talonFX;
  private final TalonFXSimState talonFXSim;
  private final DCMotorSim motorSim;

  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private double targetAngle = 0.0;

  public HoodSubsystemIOSim() {
    talonFX = new TalonFX(Constants.Subsystems.Shooter.Hood.Id.MOTOR_ID);
    talonFXSim = talonFX.getSimState();

    DCMotor gearbox = DCMotor.getKrakenX44Foc(1);

    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, 0.05, Constants.Subsystems.Shooter.Hood.GEAR_RATIO),
            gearbox);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Shooter.Hood.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Shooter.Hood.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Shooter.Hood.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Shooter.Hood.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Shooter.Hood.ClosedLoop.KV;
    config.Slot0.kA = Constants.Subsystems.Shooter.Hood.ClosedLoop.KA;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.SUPPLY_LIMIT_AMPS;

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Shooter.Hood.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted =
        Constants.Subsystems.Shooter.Hood.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFX.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double motorVoltage = talonFXSim.getMotorVoltage();

    // Simulated friction and damping
    // This stabilizes the simulation with the real-world tuned PID values (high kP and kS)
    double velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    double frictionVolts = Constants.Subsystems.Shooter.Hood.ClosedLoop.KS;
    double dampingVolts = velocityRadPerSec * 0.1;

    if (Math.abs(velocityRadPerSec) > 1e-3) {
      motorVoltage -= Math.signum(velocityRadPerSec) * frictionVolts;
    } else {
      if (Math.abs(motorVoltage) < frictionVolts) {
        motorVoltage = 0;
      } else {
        motorVoltage -= Math.signum(motorVoltage) * frictionVolts;
      }
    }
    motorVoltage -= dampingVolts;

    motorSim.setInputVoltage(motorVoltage);
    motorSim.update(0.02);

    // Hard stop simulation
    double currentAngleDeg = Units.radiansToDegrees(motorSim.getAngularPositionRad());
    if (currentAngleDeg <= Constants.Subsystems.Shooter.Hood.MIN_ANGLE_DEG && motorVoltage < 0) {
      motorSim.setState(
          Units.degreesToRadians(Constants.Subsystems.Shooter.Hood.MIN_ANGLE_DEG), 0.0);
    } else if (currentAngleDeg >= Constants.Subsystems.Shooter.Hood.MAX_ANGLE_DEG
        && motorVoltage > 0) {
      motorSim.setState(
          Units.degreesToRadians(Constants.Subsystems.Shooter.Hood.MAX_ANGLE_DEG), 0.0);
    }

    talonFXSim.setRotorVelocity(
        RotationsPerSecond.of(
            motorSim.getAngularVelocityRadPerSec()
                * Constants.Subsystems.Shooter.Hood.GEAR_RATIO
                / (2 * Math.PI)));

    talonFXSim.setRawRotorPosition(
        Rotations.of(
            motorSim.getAngularPositionRad()
                * Constants.Subsystems.Shooter.Hood.GEAR_RATIO
                / (2 * Math.PI)));

    inputs.angle =
        Rotations.of(
                talonFX.getPosition().getValueAsDouble()
                    / Constants.Subsystems.Shooter.Hood.GEAR_RATIO)
            .in(Degrees);
    inputs.velocityRPM =
        RotationsPerSecond.of(
                talonFX.getVelocity().getValueAsDouble()
                    / Constants.Subsystems.Shooter.Hood.GEAR_RATIO)
            .in(RPM);
    inputs.appliedVolts = talonFX.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = talonFX.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelsius = talonFX.getDeviceTemp().getValueAsDouble();
    inputs.connected = true;
    inputs.targetAngle = targetAngle;
    inputs.atSetpoint = Math.abs(targetAngle - inputs.angle) <= HoodSubsystem.toleranceDeg.get();
  }

  @Override
  public void setTargetAngle(double degrees) {
    targetAngle = degrees;
    talonFX.setControl(
        positionRequest.withPosition(
            Rotations.convertFrom(
                degrees * Constants.Subsystems.Shooter.Hood.GEAR_RATIO, Degrees)));
  }

  @Override
  public void setTargetAngleWithVelocity(double degrees, double velocityRPM) {
    targetAngle = degrees;
    talonFX.setControl(
        positionRequest
            .withPosition(
                Rotations.convertFrom(
                    degrees * Constants.Subsystems.Shooter.Hood.GEAR_RATIO, Degrees))
            .withVelocity(velocityRPM));
  }

  @Override
  public void configPID(double kP, double kI, double kD, double kS, double kV, double kA) {
    var slot0 = new Slot0Configs();
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    talonFX.getConfigurator().apply(slot0);
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void set(double percentOutput) {
    talonFX.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  @Override
  public void stop() {
    talonFX.stopMotor();
  }

  @Override
  public void setAngle(double degrees) {
    double rotorPos =
        Rotations.convertFrom(degrees * Constants.Subsystems.Shooter.Hood.GEAR_RATIO, Degrees);
    talonFX.setPosition(rotorPos);
    talonFXSim.setRawRotorPosition(rotorPos);
  }
}
