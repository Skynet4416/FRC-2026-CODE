package frc.robot.subsystems.shooter.shooterIndexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

public class ShooterIndexerIOSparkMax implements ShooterIndexerIO {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Shooter Indexer motor disconnected!", AlertType.kWarning);

  public ShooterIndexerIOSparkMax() {
    this.motor =
        new SparkMax(Constants.Subsystems.ShooterIndexer.Id.Motor.MOTOR, MotorType.kBrushless);
    this.motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(
            Constants.Subsystems.ShooterIndexer.ROLLER_BREAK
                ? SparkBaseConfig.IdleMode.kBrake
                : SparkBaseConfig.IdleMode.kCoast)
        .voltageCompensation(12);

    if (Constants.Subsystems.ShooterIndexer.CurrentLimits.SUPPLY_ENABLED) {
      motorConfig.smartCurrentLimit(
          (int) Constants.Subsystems.ShooterIndexer.CurrentLimits.SUPPLY_LIMIT_AMPS);
    }

    this.motor.configure(
        motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ShooterIndexerIOInputs inputs) {
    inputs.velocityRPM = this.motor.getEncoder().getVelocity();
    inputs.appliedVolts = this.motor.getAppliedOutput() * this.motor.getBusVoltage();
    inputs.requestedPercentage = this.motor.get();
    inputs.supplyCurrentAmps = this.motor.getOutputCurrent();

    inputs.connected = motorConnectedDebouncer.calculate(this.motor.getFirmwareVersion() != 0);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  @Override
  public void setVoltage(double volts) {
    this.motor.setVoltage(volts);
  }

  @Override
  public void setPercentage(double percentage) {
    this.motor.set(percentage);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }
}
