// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;




public class Vision extends SubsystemBase {

  private final NetworkTable limelightTable;
  @SuppressWarnings("unused")
  private final DriveSubsystem driveSubsystem;
  PIDController rotatePid = new PIDController(0.125, 0, 0);
  PIDController xPid = new PIDController(1, 0, 0.005);
  PIDController yPid = new PIDController(0.0605, 0, 0.0055);

  public Pose2d limePose2d; 

  /** Creates a new vision. */
  public Vision(DriveSubsystem driveSubsystem) {
     limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    SmartDashboard.putString("Limelight Camera Feed", "http://limelight.local:5800/stream.mjpg");
    int[] validIDs = {1,7,8};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    this.driveSubsystem = driveSubsystem;
  }

public double getTargetX(){
  return limelightTable.getEntry("tx").getDouble(0.0);
}
public double getTargetY(){
  return limelightTable.getEntry("ty").getDouble(0.0);
}
public double getArea(){
  return limelightTable.getEntry("ta").getDouble(0.0);
}
public boolean isTargetVisible(){
  return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
}
public void setPipeline(int pipeline){
  limelightTable.getEntry("pipeline").setNumber(pipeline);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
