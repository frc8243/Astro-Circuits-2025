// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.naming.LimitExceededException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class MoveToTarget extends Command {
  private final DriveSubsystem swerveDrive;
  private final Vision limelight;
  public static final double kP = 0.05;
  private static final double targetDistance = .8; //meters away
  private static final double aprilTagHeight = 0.15; //Meters tag height
  private double angleFromGround;
  private double distance;
  
  /** Creates a new TurnToAprilTag. */
  public MoveToTarget(DriveSubsystem swerveDrive, Vision limelight) {
    this.swerveDrive = swerveDrive;
    this.limelight = limelight;
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.isTargetVisible() ){
      angleFromGround = limelight.getTargetY();
      distance = Math.abs(aprilTagHeight/(Math.tan(Math.toRadians(angleFromGround))));
      
        if(distance > targetDistance){
            System.out.println("MOVING TO APRIL TAG");
            swerveDrive.drive(0.2,0, 0, false);
        }
        else{
            swerveDrive.drive(0, 0, 0, false);
        }

    } else{
      System.out.println("NO TARGET");
      swerveDrive.drive(0,0,0, false);
    }
    SmartDashboard.putNumber("targetDistance", distance);
    SmartDashboard.putNumber("angle", angleFromGround);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //limelight.isTargetVisible() && distance < targetDistance;
  }
}
