// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class TurnToTarget extends Command {
  private final DriveSubsystem swerveDrive;
  private final Vision limelight;
  public static final double kP = 0.01;
  private static final double tolerance = 1.0;
  
  /** Creates a new TurnToTarget. */
  public TurnToTarget(DriveSubsystem swerveDrive, Vision limelight) {
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
    if (limelight.isTargetVisible()){
      double tx = limelight.getTargetX();
      double turnSpeed = kP * tx;

      System.out.println("TURNING TO APRIL TAG");
      swerveDrive.drive(0,0, -turnSpeed, true);
    } else{
      System.out.println("NO TARGET");
      swerveDrive.drive(0,0,0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return limelight.isTargetVisible() && Math.abs(limelight.getTargetX()) < tolerance;
  }
}