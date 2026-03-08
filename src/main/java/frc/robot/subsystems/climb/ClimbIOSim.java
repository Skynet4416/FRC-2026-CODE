package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimbIOSim implements ClimbIO {
  private final SingleJointedArmSim neoSim =
      new SingleJointedArmSim(DCMotor.getNEO(2), 8.0, 0.5, 1.0, -Math.PI, Math.PI, true, 0.0);
  private final SingleJointedArmSim krakenSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), 240.0, 0.5, 1.0, -Math.PI, Math.PI, true, 0.0);

  private boolean isLowered = false;

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    neoSim.update(0.02);
    krakenSim.update(0.02);

    inputs.neoPositionRads = neoSim.getAngleRads();
    inputs.neoVelocityRadsPerSec = neoSim.getVelocityRadPerSec();
    inputs.neoAppliedVolts = 0.0;
    inputs.neoCurrentAmps = neoSim.getCurrentDrawAmps();

    inputs.krakenPositionRads = krakenSim.getAngleRads();
    inputs.krakenVelocityRadsPerSec = krakenSim.getVelocityRadPerSec();
    inputs.krakenAppliedVolts = 0.0;
    inputs.krakenCurrentAmps = krakenSim.getCurrentDrawAmps();

    inputs.lowered = isLowered;
  }

  @Override
  public void setNeoAngle(double rads) {
    // simplified mock control
    neoSim.setInputVoltage((rads - neoSim.getAngleRads()) * 10.0);
  }

  @Override
  public void setKrakenAngle(double rads) {
    krakenSim.setInputVoltage((rads - krakenSim.getAngleRads()) * 10.0);
  }

  @Override
  public void setLowered(boolean lowered) {
    this.isLowered = lowered;
  }

  @Override
  public void stop() {
    neoSim.setInputVoltage(0.0);
    krakenSim.setInputVoltage(0.0);
  }
}
