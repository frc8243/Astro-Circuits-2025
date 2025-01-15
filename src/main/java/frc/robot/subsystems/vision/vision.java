// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers;




public class vision extends SubsystemBase {
  private final NetworkTable limelightTable;
  private final DriveSubsystem driveSubsystem;
  private boolean doRejectUpdate;
  private final Field2d m_field = new Field2d();

 public Pose2d limePose2d; 

  /** Creates a new vision. */
  public vision(DriveSubsystem driveSubsystem) {
     limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    SmartDashboard.putString("Limelight Camera Feed", "http://limelight.local:5800/stream.mjpg");
    int[] validIDs = {7,8};
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
