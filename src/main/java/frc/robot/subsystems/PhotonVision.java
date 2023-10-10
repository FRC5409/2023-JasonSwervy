// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.invoke.VarHandle;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kPhotonCamera;

public class PhotonVision extends SubsystemBase {
  private final PhotonCamera photonCamera;  
  private static PhotonPipelineResult result; 

  public PhotonVision() {
    //Constructor
    photonCamera = new PhotonCamera("photonvision");
  }

  @Override
  public void periodic() {
    //retrieving latest data
    result = getLatestResult();
  }

  //Get methods
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
    return PhotonUtils.calculateDistanceToTargetMeters(kPhotonCamera.kCameraHeight, targetHeight, kPhotonCamera.kCameraPitch, Units.degreesToRadians(result.getBestTarget().getPitch()));
  }

  /**
   * Returns the camera latency in milliseconds
   * @return camera latency
   */
  public double getLatency(){
    return result.getLatencyMillis();
  }

  //Set methods
  
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
