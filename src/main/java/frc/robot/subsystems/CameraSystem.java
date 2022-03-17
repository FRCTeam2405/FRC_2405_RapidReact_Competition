// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSystem extends SubsystemBase {
  /** Creates a new CameraSystem. */
  //UsbCamera camera1 = null;
  //VideoSink cameraVideo = null;
  public CameraSystem() {
  CameraServer.startAutomaticCapture(0);
  //  camera1.setResolution(640, 480);
  //  cameraVideo = CameraServer.getServer();
  //  camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  //  cameraVideo.setSource(camera1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
