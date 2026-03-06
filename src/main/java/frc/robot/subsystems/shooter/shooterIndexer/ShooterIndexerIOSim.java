package frc.robot.subsystems.shooter.shooterIndexer;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class ShooterIndexerIOSim implements ShooterIndexerIO {
  private final DCMotor maxGearbox = DCMotor.getNeo550(1);
  private final SparkMax motor;
  private final SparkMaxSim motorSim;
  private final DCMotorSim dcMotorSim;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Shooter Indexer motor disconnected!", AlertType.kWarning);

  private double requestedPercentage = 0.0;

  public ShooterIndexerIOSim() {
    this.motor =
        new SparkMax(Constants.Subsystems.ShooterIndexer.Id.Motor.MOTOR, MotorType.kBrushless);
    this.motorSim = new SparkMaxSim(this.motor, this.maxGearbox);
    this.dcMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(maxGearbox, 0.005, 1.0), maxGearbox);
  }

  @Override
  public void updateInputs(ShooterIndexerIOInputs inputs) {
    this.dcMotorSim.setInputVoltage(this.motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    this.dcMotorSim.update(0.02);

    this.motorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(this.dcMotorSim.getAngularVelocityRadPerSec()),
        RoboRioSim.getVInVoltage(),
        0.02);

    inputs.velocityRPM = this.motorSim.getVelocity();
    inputs.appliedVolts = this.motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    inputs.requestedPercentage = this.requestedPercentage;
    inputs.supplyCurrentAmps = this.motorSim.getMotorCurrent();

    inputs.connected = true;
    motorDisconnectedAlert.set(!inputs.connected);
  }

  @Override
  public void setPercentage(double percentage) {
    this.motor.set(percentage);
    this.requestedPercentage = percentage;
  }

  @Override
  public void setVoltage(double volts) {
    this.motor.setVoltage(volts);
    this.requestedPercentage = 0.0;
  }

  @Override
  public void stop() {
    this.motor.stopMotor();
    this.requestedPercentage = 0.0;
  }
}
