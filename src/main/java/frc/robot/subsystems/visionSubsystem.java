// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.commands.visionTurnToTag;

public class visionSubsystem extends SubsystemBase {
  /** Creates a new visionSubsystem. */
  public final PhotonCamera camera = new PhotonCamera(CameraConstants.USBcam1);
  public static PhotonPipelineResult result;
  public static boolean hasTarget;

  public visionSubsystem() {
    
  }

  @Override
  public void periodic() {
      PhotonPipelineResult result = camera.getLatestResult(); // Query the latest result from PhotonVision
      hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
      if (hasTarget) {
          this.result = result;
      }
  }

  public static boolean getHasTarget() {
    return hasTarget; // Returns whether or not a target was found
  }

  public static PhotonTrackedTarget getBestTarget() {
    if (hasTarget) {
    return result.getBestTarget(); // Returns the best (closest) target
    }
    else {
        return null; // Otherwise, returns null if no targets are currently found
    }
}
}
