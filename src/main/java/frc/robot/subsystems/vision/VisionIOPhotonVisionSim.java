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

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * IO implementation for physics sim using the PhotonVision simulator.
 *
 * <p>Every camera created here feeds the shared {@link VisionSystemSim} that holds the AprilTag
 * layout, and publishes both a raw and a processed MJPEG stream (CameraServer ports 1181+). Those
 * streams are what an external vision consumer - a driver, AdvantageScope, or an LLM driving the
 * robot through {@link frc.robot.subsystems.AIControlBridge} - actually looks at, so they are
 * enabled unconditionally in simulation.
 */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Add sim camera
    cameraSim = new PhotonCameraSim(camera, simCameraProperties());

    // Publish the camera feeds so external consumers (AdvantageScope, an AI operator) can watch
    // what the robot sees.
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(simDrawWireframe);

    getVisionSim().addCamera(cameraSim, robotToCamera);
  }

  /** Returns the shared AprilTag vision sim, creating it (with the field layout) on first use. */
  public static VisionSystemSim getVisionSim() {
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }
    return visionSim;
  }

  /** Camera properties shared by every simulated camera. */
  private static SimCameraProperties simCameraProperties() {
    var properties = new SimCameraProperties();
    properties.setCalibration(
        simCameraWidthPx, simCameraHeightPx, Rotation2d.fromDegrees(simCameraFovDeg));
    properties.setCalibError(simCameraAvgErrorPx, simCameraErrorStdDevPx);
    properties.setFPS(simCameraFps);
    properties.setAvgLatencyMs(simCameraAvgLatencyMs);
    properties.setLatencyStdDevMs(simCameraLatencyStdDevMs);
    return properties;
  }

  /** Returns the simulated camera, for tuning or for adding extra target types. */
  public PhotonCameraSim getCameraSim() {
    return cameraSim;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    getVisionSim().update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
