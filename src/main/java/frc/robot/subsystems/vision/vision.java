// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;


import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers;




public class Vision extends SubsystemBase {
  private static final Angle CAMERA_PITCH = Degrees.of(0);
  private static final Distance X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
  private static final Distance Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
  private static final Distance Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
  private static final Angle CAMERA_YAW = Degrees.of(0);

  private final NetworkTable limelightTable;
  private final DriveSubsystem driveSubsystem;
  // private boolean doRejectUpdate;
  private final Field2d m_field = new Field2d();
  PIDController rotatePid = new PIDController(0.125, 0, 0);
  PIDController xPid = new PIDController(1, 0, 0.005);
  PIDController yPid = new PIDController(0.0605, 0, 0.0055);

  // private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
  // //     AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  // // private final Transform3d ROBOTTOCAM =
  // //     new Transform3d(
  // //         new Translation3d(0.5, 0.0, 0.5),
  // //         new Rotation3d(
  // //             0, 0,
  // //             0)); 

//  public Pose2d limePose2d; 

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
