// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Controller-agnostic wrapper for the driver, allowing an Xbox or a PS5 controller to be used
 * interchangeably on the same port.
 *
 * <p>Both an Xbox and a PS5 controller are constructed on the same Driver Station port (safe, since
 * each is a thin wrapper over the same underlying HID). Every button/axis accessor delegates to
 * whichever type {@code typeSupplier} currently reports, so switching the selection (e.g. via a
 * dashboard chooser) re-routes input live without redeploying.
 *
 * <p>Button names follow the WPILib/Xbox convention; the equivalent PS5 button is noted in each
 * method's docs. Platform-specific buttons (PS5 touchpad/PS/Create-Share, Xbox View/Back) are
 * intentionally not exposed.
 */
public class DriverController {
  /** The physical controller type in use. */
  public enum Type {
    XBOX,
    PS5
  }

  private final CommandXboxController xbox;
  private final CommandPS5Controller ps5;
  private final Supplier<Type> typeSupplier;

  /**
   * @param port The Driver Station port the driver controller is plugged into.
   * @param typeSupplier Supplies the active controller type. May return {@code null}; treated as
   *     {@link Type#XBOX}.
   */
  public DriverController(int port, Supplier<Type> typeSupplier) {
    this.xbox = new CommandXboxController(port);
    this.ps5 = new CommandPS5Controller(port);
    this.typeSupplier = typeSupplier;
  }

  /**
   * Returns whether the Xbox controller is the currently selected type (null-safe, defaults Xbox).
   */
  private boolean isXbox() {
    return typeSupplier.get() != Type.PS5;
  }

  /**
   * Returns a Trigger that reports the active controller's button, re-evaluated every poll so the
   * selection can change at runtime.
   */
  private Trigger select(BooleanSupplier isXbox, Trigger xboxButton, Trigger ps5Button) {
    return new Trigger(
        () -> isXbox.getAsBoolean() ? xboxButton.getAsBoolean() : ps5Button.getAsBoolean());
  }

  // ---- Face buttons ----

  /** Xbox A / PS5 Cross (bottom face button). */
  public Trigger a() {
    return select(this::isXbox, xbox.a(), ps5.cross());
  }

  /** Xbox B / PS5 Circle (right face button). */
  public Trigger b() {
    return select(this::isXbox, xbox.b(), ps5.circle());
  }

  /** Xbox X / PS5 Square (left face button). */
  public Trigger x() {
    return select(this::isXbox, xbox.x(), ps5.square());
  }

  /** Xbox Y / PS5 Triangle (top face button). */
  public Trigger y() {
    return select(this::isXbox, xbox.y(), ps5.triangle());
  }

  // ---- Bumpers ----

  /** Xbox Left Bumper / PS5 L1. */
  public Trigger leftBumper() {
    return select(this::isXbox, xbox.leftBumper(), ps5.L1());
  }

  /** Xbox Right Bumper / PS5 R1. */
  public Trigger rightBumper() {
    return select(this::isXbox, xbox.rightBumper(), ps5.R1());
  }

  // ---- Triggers (digital, past default threshold) ----

  /** Xbox Left Trigger / PS5 L2. */
  public Trigger leftTrigger() {
    return select(this::isXbox, xbox.leftTrigger(), ps5.L2());
  }

  /** Xbox Right Trigger / PS5 R2. */
  public Trigger rightTrigger() {
    return select(this::isXbox, xbox.rightTrigger(), ps5.R2());
  }

  // ---- Stick clicks ----

  /** Xbox Left Stick press / PS5 L3. */
  public Trigger leftStick() {
    return select(this::isXbox, xbox.leftStick(), ps5.L3());
  }

  /** Xbox Right Stick press / PS5 R3. */
  public Trigger rightStick() {
    return select(this::isXbox, xbox.rightStick(), ps5.R3());
  }

  // ---- Menu ----

  /** Xbox Start / PS5 Options. */
  public Trigger start() {
    return select(this::isXbox, xbox.start(), ps5.options());
  }

  // ---- D-pad / POV (identical on both; delegate to the active controller) ----

  /** D-pad up. */
  public Trigger povUp() {
    return select(this::isXbox, xbox.povUp(), ps5.povUp());
  }

  /** D-pad down. */
  public Trigger povDown() {
    return select(this::isXbox, xbox.povDown(), ps5.povDown());
  }

  /** D-pad left. */
  public Trigger povLeft() {
    return select(this::isXbox, xbox.povLeft(), ps5.povLeft());
  }

  /** D-pad right. */
  public Trigger povRight() {
    return select(this::isXbox, xbox.povRight(), ps5.povRight());
  }

  /** D-pad at the given angle (degrees). */
  public Trigger pov(int angle) {
    return select(this::isXbox, xbox.pov(angle), ps5.pov(angle));
  }

  // ---- Axes ----

  /** Left stick X (right positive). */
  public double getLeftX() {
    return isXbox() ? xbox.getLeftX() : ps5.getLeftX();
  }

  /** Left stick Y (back positive). */
  public double getLeftY() {
    return isXbox() ? xbox.getLeftY() : ps5.getLeftY();
  }

  /** Right stick X (right positive). */
  public double getRightX() {
    return isXbox() ? xbox.getRightX() : ps5.getRightX();
  }

  /** Right stick Y (back positive). */
  public double getRightY() {
    return isXbox() ? xbox.getRightY() : ps5.getRightY();
  }

  /** Left trigger analog value, [0, 1]. Xbox Left Trigger / PS5 L2. */
  public double getLeftTriggerAxis() {
    return isXbox() ? xbox.getLeftTriggerAxis() : ps5.getL2Axis();
  }

  /** Right trigger analog value, [0, 1]. Xbox Right Trigger / PS5 R2. */
  public double getRightTriggerAxis() {
    return isXbox() ? xbox.getRightTriggerAxis() : ps5.getR2Axis();
  }

  // ---- Misc ----

  /** Sets rumble on the active controller. */
  public void setRumble(GenericHID.RumbleType type, double value) {
    if (isXbox()) {
      xbox.getHID().setRumble(type, value);
    } else {
      ps5.getHID().setRumble(type, value);
    }
  }

  /** The Driver Station port this controller is on. */
  public int getPort() {
    return xbox.getHID().getPort();
  }

  /** Whether a controller is currently connected on this port. */
  public boolean isConnected() {
    return DriverStation.isJoystickConnected(getPort());
  }
}
