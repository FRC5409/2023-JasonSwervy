// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.kPhotonCamera;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;
import com.fasterxml.jackson.databind.introspect.TypeResolutionContext.Empty;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  private final PhotonCamera photonCamera;  
  private static PhotonPipelineResult result; 
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private static Transform3d cameraPos; 
  private static PhotonPoseEstimator poseEstimator;
  private static Optional<EstimatedRobotPose> lastEstimatedPose;
  //TODO sort objs

  public PhotonVision() {
    //Constructor
    photonCamera = new PhotonCamera("OV5647");

    try {
      aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch(IOException e) {
        System.out.println("FAILED TO LOAD JSON");
    }

    cameraPos = new Transform3d(new Translation3d(kPhotonCamera.kCameraPos.kCameraXOffset, kPhotonCamera.kCameraPos.kCameraYOffset, kPhotonCamera.kCameraPos.kCameraZOffset), new Rotation3d(0, 0, 0)); //TODO add rotation constants
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, photonCamera, cameraPos);
    //TODO initialize lastEstmimatedPose with current odometry location
  }

  @Override
  public void periodic() {
    //retrieving latest data
    result = getLatestResult(); 
    if (result.hasTargets()){
      System.out.println(result.getBestTarget().getFiducialId());
      Optional<EstimatedRobotPose> fieldPose = getFieldPosition(); //missing last estimation param
      if (!fieldPose.isEmpty()){
        System.out.println(fieldPose.get());
      } else {
        System.out.println("OPTIONAL IS EMPTY");
      }
    }
  }

  //----------UPDATE METHODS----------\\\
  /**
   * updates the robot position in fieldspace if there is an available target with high enough accuracy
   */
  public Optional<EstimatedRobotPose> getFieldPosition(){
    if (result.hasTargets()){
      PhotonTrackedTarget bestTarget = result.getBestTarget(); //returns the target of highest quality/accuracy
      double targetAmbiguity = bestTarget.getPoseAmbiguity();

      //Updating position only if ambiguity is above threshold
      if (targetAmbiguity <= kPhotonCamera.kFLAmbiguityThreshold){
        return poseEstimator.update();
      }
    }
    return Optional.empty();
  }

  //----------GETTER METHODS----------\\\
  /**
   * Returns the lastest available data from the current photonvision camera.
   * @return latest camera result.
   */
  public PhotonPipelineResult getLatestResult(){
    //TODO if more than 1 photonvision instance is used, pass down photonCamera as a parameter for reusability
    return photonCamera.getLatestResult();
  }

  /**
   * Returns the distance to a specified target of known height.
   * @param targetHeight Height of desired target. 
   * @return Distance to target. 
   */
  public double getTargetDistance(double targetHeight){
    return PhotonUtils.calculateDistanceToTargetMeters(kPhotonCamera.kCameraPos.kCameraZOffset, targetHeight, kPhotonCamera.kCameraPos.kCameraPitch, Units.degreesToRadians(result.getBestTarget().getPitch()));
  }

  /**
   * Returns the camera latency in milliseconds
   * @return camera latency
   */
  public double getLatency(){
    return result.getLatencyMillis();
  }

  /**
   * returns the index of the currently selected pipeline
   * @return pipeline index
   */
  public int getPipeline(){
    return photonCamera.getPipelineIndex();
  }

  //----------SETTER METHODS----------\\\
  /**
   * Turns on the camera LEDs
   */
  public void setLEDOn(){
    photonCamera.setLED(VisionLEDMode.kOn);
  }

  /**
   * Turns off the camera LEDs
   */
  public void setLEDOff(){
    photonCamera.setLED(VisionLEDMode.kOff);
  }

  /**
   * Sets the camera LED mode to a specified mode
   * @param mode The desired LED mode
   */
  public void setLEDMode(VisionLEDMode mode){
    photonCamera.setLED(mode);
  }

  /**
   * Selects pipeline to specified indexs
   * @param pipelineIndex Index of desired pipeline
   */
  public void setPipeline(int pipelineIndex){
    photonCamera.setPipelineIndex(pipelineIndex);
  }
}
