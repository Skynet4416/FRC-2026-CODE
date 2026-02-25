// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class LauncherConstants {
  public static Transform3d robotToLauncher =
      new Transform3d(-0.276, 0.09, 0.599, new Rotation3d(0.0, 0.0, Math.PI));

  private LauncherConstants() {}
}
