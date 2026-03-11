package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelSubsystemIOSim implements FlywheelSubsystemIO {

  private final TalonFX talonFX;
  private final TalonFXSimState talonFXSim;
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(2), 0.004, Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO),
          DCMotor.getKrakenX60(2));

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double targetRPM = 0.0;

  public FlywheelSubsystemIOSim() {
    talonFX = new TalonFX(Constants.Subsystems.Shooter.Flywheel.Id.LEADER_ID);
    talonFXSim = talonFX.getSimState();

    // Apply the same configuration as the real hardware
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KV;
    config.Slot0.kA = Constants.Subsystems.Shooter.Flywheel.ClosedLoop.KA;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.SUPPLY_LIMIT_AMPS;

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Shooter.Flywheel.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted =
        Constants.Subsystems.Shooter.Flywheel.SHOOTER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    talonFX.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Get the motor voltage applied by the TalonFX object
    double motorVoltage = talonFXSim.getMotorVoltage();

    // Update the physics sim
    motorSim.setInputVoltage(motorVoltage);
    motorSim.update(0.02);

    // Update the SimState with the new physics state
    // DCMotorSim returns radians, TalonFX expects rotations
    double rotorVelocityRPM =
        motorSim.getAngularVelocityRPM() * Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO;
    double rotorPositionRotations =
        (motorSim.getAngularPosition().in(edu.wpi.first.units.Units.Radians) / (2 * Math.PI))
            * Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO;

    // Convert to rotations/sec for Phoenix 6
    talonFXSim.setRotorVelocity(rotorVelocityRPM / 60.0);
    talonFXSim.setRawRotorPosition(rotorPositionRotations);

    inputs.velocityRPM =
        talonFX.getVelocity().getValueAsDouble()
            * 60.0
            / Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO;
    inputs.appliedVolts = talonFX.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = talonFX.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelsius = talonFX.getDeviceTemp().getValueAsDouble();
    inputs.connected = true;
    inputs.setpointRPM = targetRPM;
    inputs.atSetpoint =
        Math.abs(inputs.velocityRPM - targetRPM)
            <= Constants.Subsystems.Shooter.Flywheel.RPM_TOLERANCE;
  }

  @Override
  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
    double motorRPS = (rpm / 60.0) * Constants.Subsystems.Shooter.Flywheel.GEAR_RATIO;
    talonFX.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setTargetRADS(double radiansPerSecond) {
    setTargetRPM(Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond));
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
  public void stop() {
    setVoltage(0);
    this.targetRPM = 0.0;
  }
}
