package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelSubsystemIOSim implements FlywheelSubsystemIO {

  private final TalonFX talonFX;
  private final TalonFXSimState talonFXSim;
  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(2), 0.004, Constants.Subsystems.Shooter.GEAR_RATIO),
          DCMotor.getKrakenX60(2));

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private double targetRPM = 0.0;

  public FlywheelSubsystemIOSim() {
    talonFX = new TalonFX(Constants.Subsystems.Shooter.Id.LEADER_ID);
    talonFXSim = talonFX.getSimState();

    // Apply the same configuration as the real hardware
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = Constants.Subsystems.Shooter.ClosedLoop.KP;
    config.Slot0.kI = Constants.Subsystems.Shooter.ClosedLoop.KI;
    config.Slot0.kD = Constants.Subsystems.Shooter.ClosedLoop.KD;
    config.Slot0.kS = Constants.Subsystems.Shooter.ClosedLoop.KS;
    config.Slot0.kV = Constants.Subsystems.Shooter.ClosedLoop.KV;
    config.Slot0.kA = Constants.Subsystems.Shooter.ClosedLoop.KA;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Subsystems.Shooter.CurrentLimits.SUPPLY_ENABLED;
    config.CurrentLimits.SupplyCurrentLimit =
        Constants.Subsystems.Shooter.CurrentLimits.SUPPLY_LIMIT_AMPS;

    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Shooter.CurrentLimits.STATOR_ENABLED;
    config.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Shooter.CurrentLimits.STATOR_LIMIT_AMPS;

    config.MotorOutput.Inverted =
        Constants.Subsystems.Shooter.SHOOTER_INVERTED
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
        motorSim.getAngularVelocityRPM() * Constants.Subsystems.Shooter.GEAR_RATIO;
    double rotorPositionRotations =
        (motorSim.getAngularPosition().in(edu.wpi.first.units.Units.Radians) / (2 * Math.PI))
            * Constants.Subsystems.Shooter.GEAR_RATIO;

    // Convert to rotations/sec for Phoenix 6
    talonFXSim.setRotorVelocity(rotorVelocityRPM / 60.0);
    talonFXSim.setRawRotorPosition(rotorPositionRotations);

    inputs.velocityRPM =
        talonFX.getVelocity().getValueAsDouble() * 60.0 / Constants.Subsystems.Shooter.GEAR_RATIO;
    inputs.appliedVolts = talonFX.getMotorVoltage().getValueAsDouble();
    inputs.supplyCurrentAmps = talonFX.getSupplyCurrent().getValueAsDouble();
    inputs.tempCelsius = talonFX.getDeviceTemp().getValueAsDouble();
    inputs.connected = true;
    inputs.setpointRPM = targetRPM;
  }

  @Override
  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
    double motorRPS = (rpm / 60.0) * Constants.Subsystems.Shooter.GEAR_RATIO;
    talonFX.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts));
  }
}
