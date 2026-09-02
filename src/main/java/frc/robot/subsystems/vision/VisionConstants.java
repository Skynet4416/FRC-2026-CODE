// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-right";
  public static String camera1Name = "limelight-left";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
  public static Transform3d robotToCamera1 =
      new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Simulation camera properties (PhotonVision sim). These drive both the AprilTag pose
  // estimates and the MJPEG streams published on CameraServer ports 1181+.
  public static int simCameraWidthPx = 640;
  public static int simCameraHeightPx = 480;
  public static double simCameraFovDeg = 90.0;
  public static double simCameraAvgErrorPx = 0.25;
  public static double simCameraErrorStdDevPx = 0.08;
  public static double simCameraFps = 20.0;
  public static double simCameraAvgLatencyMs = 35.0;
  public static double simCameraLatencyStdDevMs = 5.0;

  // Wireframe overlay makes the streams easier to read but is expensive enough to cause loop
  // overruns with three simulated cameras. Off by default; turn it on when debugging a camera.
  public static boolean simDrawWireframe = false;

  // Simulated game piece ("fuel") detection camera. Only exists in simulation; it publishes an
  // object detection stream so an external consumer can see where the game pieces are.
  public static String gamePieceCameraName = "fuel-cam";
  public static Transform3d robotToGamePieceCamera =
      new Transform3d(0.25, 0.0, 0.55, new Rotation3d(0.0, Math.toRadians(20.0), 0.0));
  public static double gamePieceDiameterMeters = 0.1501; // Game manual 5.10.1
  public static int gamePieceClassId = 0;
  public static double gamePieceMaxSightRangeMeters = 8.0;
}
