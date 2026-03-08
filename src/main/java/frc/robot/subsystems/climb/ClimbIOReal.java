package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class ClimbIOReal implements ClimbIO {
  private final SparkMax leftNeo;
  private final SparkMax rightNeo; // Follows left
  private final TalonFX kraken;
  private final DoubleSolenoid solenoid;

  private final PositionVoltage krakenPositionControl = new PositionVoltage(0).withSlot(0);

  private final Debouncer neoLeaderConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer neoFollowerConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer krakenConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final Alert neoLeaderDisconnectedAlert =
      new Alert("Climb NEO Leader disconnected!", AlertType.kWarning);
  private final Alert neoFollowerDisconnectedAlert =
      new Alert("Climb NEO Follower disconnected!", AlertType.kWarning);
  private final Alert krakenDisconnectedAlert =
      new Alert("Climb Kraken disconnected!", AlertType.kWarning);

  public ClimbIOReal() {
    leftNeo = new SparkMax(Constants.Subsystems.Climber.Id.Motor.LEFT_NEO, MotorType.kBrushless);
    rightNeo = new SparkMax(Constants.Subsystems.Climber.Id.Motor.RIGHT_NEO, MotorType.kBrushless);

    SparkMaxConfig neoConfig = new SparkMaxConfig();
    neoConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Subsystems.Climber.CurrentLimits.NEO_SUPPLY_LIMIT_AMPS);
    neoConfig
        .closedLoop
        .p(Constants.Subsystems.Climber.NeoClosedLoop.KP)
        .i(Constants.Subsystems.Climber.NeoClosedLoop.KI)
        .d(Constants.Subsystems.Climber.NeoClosedLoop.KD);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.apply(neoConfig).follow(leftNeo);

    leftNeo.configure(neoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightNeo.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    kraken = new TalonFX(Constants.Subsystems.Climber.Id.Motor.KRAKEN);
    TalonFXConfiguration krakenConfig = new TalonFXConfiguration();
    krakenConfig.Slot0.kP = Constants.Subsystems.Climber.KrakenClosedLoop.KP;
    krakenConfig.Slot0.kI = Constants.Subsystems.Climber.KrakenClosedLoop.KI;
    krakenConfig.Slot0.kD = Constants.Subsystems.Climber.KrakenClosedLoop.KD;
    krakenConfig.CurrentLimits.StatorCurrentLimit =
        Constants.Subsystems.Climber.CurrentLimits.KRAKEN_SUPPLY_LIMIT_AMPS;
    krakenConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Subsystems.Climber.CurrentLimits.STATOR_ENABLED;
    kraken.getConfigurator().apply(krakenConfig);

    solenoid =
        new DoubleSolenoid(
            4,
            PneumaticsModuleType.REVPH,
            Constants.Subsystems.Climber.Id.Pneumatics.CLIMB_FORWARDS,
            Constants.Subsystems.Climber.Id.Pneumatics.CLIMB_REVERSE);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    double neoRotations = leftNeo.getEncoder().getPosition();
    inputs.neoPositionRads =
        Units.rotationsToRadians(neoRotations) / Constants.Subsystems.Climber.NEO_GEAR_RATIO;
    inputs.neoVelocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftNeo.getEncoder().getVelocity())
            / Constants.Subsystems.Climber.NEO_GEAR_RATIO;
    inputs.neoAppliedVolts = leftNeo.getAppliedOutput() * leftNeo.getBusVoltage();
    inputs.neoCurrentAmps = leftNeo.getOutputCurrent();

    double krakenRotations = kraken.getPosition().getValueAsDouble();
    inputs.krakenPositionRads =
        Units.rotationsToRadians(krakenRotations) / Constants.Subsystems.Climber.KRAKEN_GEAR_RATIO;
    inputs.krakenVelocityRadsPerSec =
        Units.rotationsToRadians(kraken.getVelocity().getValueAsDouble())
            / Constants.Subsystems.Climber.KRAKEN_GEAR_RATIO;
    inputs.krakenAppliedVolts = kraken.getMotorVoltage().getValueAsDouble();
    inputs.krakenCurrentAmps = kraken.getStatorCurrent().getValueAsDouble();

    inputs.lowered = (solenoid.get() == DoubleSolenoid.Value.kForward);

    inputs.neoLeaderConnected =
        neoLeaderConnectedDebouncer.calculate(leftNeo.getFirmwareVersion() != 0);
    neoLeaderDisconnectedAlert.set(!inputs.neoLeaderConnected);

    inputs.neoFollowerConnected =
        neoFollowerConnectedDebouncer.calculate(rightNeo.getFirmwareVersion() != 0);
    neoFollowerDisconnectedAlert.set(!inputs.neoFollowerConnected);

    inputs.krakenConnected = krakenConnectedDebouncer.calculate(kraken.isAlive());
    krakenDisconnectedAlert.set(!inputs.krakenConnected);
  }

  @Override
  public void setNeoAngle(double rads) {
    double targetRotations =
        Units.radiansToRotations(rads) * Constants.Subsystems.Climber.NEO_GEAR_RATIO;
    leftNeo.getClosedLoopController().setReference(targetRotations, ControlType.kPosition);
  }

  @Override
  public void setKrakenAngle(double rads) {
    double targetRotations =
        Units.radiansToRotations(rads) * Constants.Subsystems.Climber.KRAKEN_GEAR_RATIO;
    kraken.setControl(krakenPositionControl.withPosition(targetRotations));
  }

  @Override
  public void setLowered(boolean lowered) {
    solenoid.set(lowered ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void stop() {
    leftNeo.stopMotor();
    kraken.stopMotor();
  }
}
