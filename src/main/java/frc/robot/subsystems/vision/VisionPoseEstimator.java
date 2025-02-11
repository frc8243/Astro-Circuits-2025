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

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;

public class VisionPoseEstimator extends SubsystemBase {
  private static final Angle CAMERA_PITCH = Degrees.of(0);
  private static final Distance X_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
  private static final Distance Y_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
  private static final Distance Z_ROBOT_TO_CAMERA_OFFSET = Inches.of(0);
  private static final Angle CAMERA_YAW = Degrees.of(0);

   private static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
            X_ROBOT_TO_CAMERA_OFFSET.in(Meters), Y_ROBOT_TO_CAMERA_OFFSET.in(Meters),
            Z_ROBOT_TO_CAMERA_OFFSET.in(Meters),
            new Rotation3d(0, CAMERA_PITCH.in(Radians), CAMERA_YAW.in(Radians)));

    // reject new poses if spinning too fast
    private static final AngularVelocity MAX_ROTATIONS_PER_SECOND = RotationsPerSecond.of(2);
    private static final LinearVelocity MAX_DRIVETRAIN_SPEED_FOR_VISION_UPDATE = MetersPerSecond
            .of(0.5 /*TODO: SpeedConstants.DRIVETRAIN_MAX_SPEED_MPS*/);

    private final StructPublisher<Pose2d> mt2Publisher;
    private final DriveSubsystem driveSubsystem;
    private final String limelightName, limelightHostname;

  /** Creates a new VisionPoseEstimator. */
  public VisionPoseEstimator(DriveSubsystem driveSubsystem, String limelightName) {
    this.limelightName = limelightName;
    this.driveSubsystem = driveSubsystem;
    this.limelightHostname = "limelight" + (limelightName != "" ? "-" + limelightName : "");
    mt2Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("VisionPoseEstimator/" + this.limelightName, Pose2d.struct).publish();
    mt2Publisher.setDefault(new Pose2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
