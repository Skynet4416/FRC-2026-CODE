package frc.robot.subsystems.drive.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelSubsystemIOSim implements FlywheelSubsystemIO {

  private final DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.004, Constants.ShooterConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    inputs.velocityRPM = motorSim.getAngularVelocityRPM();

    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = motorSim.getCurrentDrawAmps();
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void setTargetRPM(double rpm) {}

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
