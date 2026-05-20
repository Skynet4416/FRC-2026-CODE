package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.shooterIndexer.ShooterIndexerSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class LedSubsystem extends SubsystemBase {
  private final ledSubsystemIO io;
  protected final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();
  private int ledMode = -1;
  private double setTime = 0;
  private ShooterIndexerSubsystem shooterIndexerSubsystem;
  private FlywheelSubsystem flywheelSubsystem;
  private SpindexerSubsystem spindexerSubsystem;
  private IntakeSubsystem intakeLeft;
  private IntakeSubsystem intakeRight;

  public LedSubsystem(
      ledSubsystemIO io,
      ShooterIndexerSubsystem shooterIndexer,
      FlywheelSubsystem flywheelSubsystem,
      SpindexerSubsystem spindexerSubsystem,
      IntakeSubsystem intakeLeft,
      IntakeSubsystem intakeRight) {
    this.io = io;
    this.shooterIndexerSubsystem = shooterIndexer;
    this.flywheelSubsystem = flywheelSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.intakeLeft = intakeLeft;
    this.intakeRight = intakeRight;
    SetIdle();
  }

  private boolean SetValue(int val) {
    if (val != 0) setTime = Timer.getFPGATimestamp();
    if (this.ledMode == val) return true;
    this.ledMode = val;
    return false;
  }

  public void SetIdle() {
    if (SetValue(0)) return;
    ControlRequest ledRequest;
    ledRequest = new SolidColor(0, 64).withColor(new RGBWColor(255, 120, 0));
    this.io.setAnimation(ledRequest);
  }

  public void SetAiming() {
    if (SetValue(1)) return;
    ControlRequest ledRequest;
    ledRequest = new SolidColor(0, 64).withColor(new RGBWColor(0, 255, 0));
    this.io.setAnimation(ledRequest);
  }

  public void SetShooting(boolean spindexer, boolean shooter) {
    int val = 10;
    if (spindexer) {
      val = 11;
      if (shooter) val = 12;
    }
    if (SetValue(val)) return;
    ControlRequest ledRequest;
    ledRequest =
        new StrobeAnimation(0, 64)
            .withColor(
                spindexer
                    ? (shooter ? new RGBWColor(0, 255, 0) : new RGBWColor(0, 0, 255))
                    : new RGBWColor(255, 0, 0))
            .withFrameRate(8);
    this.io.setAnimation(ledRequest);
  }

  public void SetIntaking(boolean reversed, boolean stuck) {
    int val = 20;
    if (stuck) val = 21;
    else if (reversed) val = 22;
    if (SetValue(val)) return;
    ControlRequest ledRequest;
    ledRequest =
        new StrobeAnimation(0, 64)
            .withColor(new RGBWColor(255, 0, 165))
            .withFrameRate(stuck ? 0 : reversed ? 2 : 8);
    this.io.setAnimation(ledRequest);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (flywheelSubsystem.getSetpoint() > 200) {
      if (Math.abs(spindexerSubsystem.getVelocityRPM())
              + Math.abs(shooterIndexerSubsystem.getVelocityRPM())
          > 200)
        SetShooting(
            shooterIndexerSubsystem.getVelocityRPM() > 200,
            spindexerSubsystem.getVelocityRPM() > 200);
      else SetAiming();
    } else {
      if (intakeLeft.isLowered() || intakeRight.isLowered()) {
        IntakeSubsystem desired = intakeLeft.isLowered() ? intakeLeft : intakeRight;
        SetIntaking(desired.isReversed(), desired.isStuck());
      } else SetIdle();
    }
  }
}
