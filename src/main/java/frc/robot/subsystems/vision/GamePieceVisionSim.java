// Copyright (c) 2026 Skynet 4416
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/**
 * Simulated object-detection camera for game pieces ("fuel").
 *
 * <p>The AprilTag cameras give an AI operator the robot's pose; this gives it the other half of the
 * picture - where the game pieces are. Every loop the live game piece positions are re-published as
 * spherical vision targets, the camera is processed against them, and the detections are both
 * streamed over CameraServer (as a normal PhotonVision object-detection pipeline) and logged for
 * AdvantageScope.
 *
 * <p>Simulation only: {@link #update()} is a no-op on real hardware, where a real coprocessor owns
 * this camera name.
 */
public class GamePieceVisionSim {
  private static final String TARGET_TYPE = "gamepiece";

  /** Cap on how many pieces are simulated at once, nearest first. */
  private static final int MAX_SIMULATED_TARGETS = 24;

  private final VisionSystemSim visionSim = new VisionSystemSim("gamepiece");
  private final PhotonCamera camera = new PhotonCamera(gamePieceCameraName);
  private final PhotonCameraSim cameraSim;
  private final TargetModel targetModel = new TargetModel(gamePieceDiameterMeters);

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<List<Translation3d>> gamePieceSupplier;

  public GamePieceVisionSim(
      Supplier<Pose2d> robotPoseSupplier, Supplier<List<Translation3d>> gamePieceSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.gamePieceSupplier = gamePieceSupplier;

    var properties = new SimCameraProperties();
    properties.setCalibration(
        simCameraWidthPx, simCameraHeightPx, Rotation2d.fromDegrees(simCameraFovDeg));
    properties.setCalibError(simCameraAvgErrorPx, simCameraErrorStdDevPx);
    properties.setFPS(simCameraFps);
    properties.setAvgLatencyMs(simCameraAvgLatencyMs);
    properties.setLatencyStdDevMs(simCameraLatencyStdDevMs);

    cameraSim = new PhotonCameraSim(camera, properties);
    cameraSim.setMaxSightRange(gamePieceMaxSightRangeMeters);
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(simDrawWireframe);

    visionSim.addCamera(cameraSim, robotToGamePieceCamera);
  }

  /**
   * Re-publishes the current game piece positions and processes one camera frame.
   *
   * <p>Only the pieces the camera could plausibly see are handed to the simulator: a field full of
   * fuel is hundreds of targets, and rebuilding all of them every loop costs more than it shows.
   */
  public void update() {
    Pose2d robotPose = robotPoseSupplier.get();
    List<Translation3d> visible =
        gamePieceSupplier.get().stream()
            .filter(
                piece ->
                    piece.toTranslation2d().getDistance(robotPose.getTranslation())
                        <= gamePieceMaxSightRangeMeters)
            .sorted(
                Comparator.comparingDouble(
                    piece -> piece.toTranslation2d().getDistance(robotPose.getTranslation())))
            .limit(MAX_SIMULATED_TARGETS)
            .toList();

    visionSim.removeVisionTargets(TARGET_TYPE);
    List<VisionTargetSim> targets = new ArrayList<>(visible.size());
    List<Pose3d> targetPoses = new ArrayList<>(visible.size());
    for (Translation3d piece : visible) {
      Pose3d pose = new Pose3d(piece, new Rotation3d());
      targets.add(new VisionTargetSim(pose, targetModel, gamePieceClassId, 1.0f));
      targetPoses.add(pose);
    }
    if (!targets.isEmpty()) {
      visionSim.addVisionTargets(TARGET_TYPE, targets.toArray(new VisionTargetSim[0]));
    }

    visionSim.update(robotPose);

    // Drain the camera and log what it saw, so the detections are visible in AdvantageScope
    // alongside the raw stream.
    for (var result : camera.getAllUnreadResults()) {
      double[] yaws = new double[result.targets.size()];
      double[] pitches = new double[result.targets.size()];
      double[] areas = new double[result.targets.size()];
      for (int i = 0; i < result.targets.size(); i++) {
        var target = result.targets.get(i);
        yaws[i] = target.getYaw();
        pitches[i] = target.getPitch();
        areas[i] = target.getArea();
      }
      Logger.recordOutput("Vision/GamePieces/VisibleCount", yaws.length);
      Logger.recordOutput("Vision/GamePieces/Yaws", yaws);
      Logger.recordOutput("Vision/GamePieces/Pitches", pitches);
      Logger.recordOutput("Vision/GamePieces/Areas", areas);
    }

    Logger.recordOutput("Vision/GamePieces/Connected", camera.isConnected());
    Logger.recordOutput(
        "Vision/GamePieces/TargetPoses", targetPoses.toArray(new Pose3d[targetPoses.size()]));
  }
}
