// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.lib.Telemetry;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //the linear regression values for limelight distance formula
  double tuningLM;
  double tuningLB;

  double distance;
  double yaw;
  double pitch;
  double area;
  double skew;
  Transform2d pose;

  PhotonCamera camera;
  
  

  public Vision(String cameraName) {
    camera = new PhotonCamera(cameraName);
    Telemetry.setValue("vision/limelight/slope", 1);
    Telemetry.setValue("vision/limelight/offset", 1);
  }

  public void limelight() {
    
    tuningLM = Telemetry.getValue("vision/limelight/slope", 1);
    tuningLB = Telemetry.getValue("vision/limelight/offset", 1);

    camera.setPipelineIndex(1);
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    distance = tuningLM * target.getArea() + tuningLB;

  }

  public void apriltag() {
    camera.setPipelineIndex(2);

  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
