// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveToTarget;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.Vision;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private SendableChooser<Command> autoChooser;
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final coralHandler m_coralHandler = new coralHandler();
  //public final elevator m_elevator = new elevator();
  private final Vision m_vision = new Vision(m_robotDrive);
  //private final AlgaeSubsystem m_AlgaeSubsystem = new AlgaeSubsystem();
  //public final AlgaeWrist m_Algaewrist = new AlgaeWrist();
  // The driver's controller
 public static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);// port 0
 public static CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);// port 1
 private ButtonBinder driverButtonBinder = new ButtonBinder(m_driverController);
 private ButtonBinder operatorButtonBinder = new ButtonBinder(m_operatorController);
 //hello

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //NamedCommands.registerCommand("Outake Coral", m_coralHandler.coralOutake(0.2).withTimeout(5));

    System.out.println(driverButtonBinder.getButtonUsageReport());
    driverButtonBinder.makeStatusDashboardWidgets("Driver Buttons");
    operatorButtonBinder.makeStatusDashboardWidgets("Operator Buttons");
    RobotContainer.m_operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);

  // An example trajectory to follow. All units in meters.
  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      config);


  // // Reset odometry to the starting pose of the trajectory.
  m_robotDrive.gyroReset();

   m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
   // m_robotDrive.
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    //   m_coralHandler.setDefaultCommand(
    //     new RunCommand(
    //         () -> m_coralHandler.setAutoIntakeMotors(0.075)
    //         , m_coralHandler));
    //     m_AlgaeSubsystem.setDefaultCommand(
    //         new RunCommand(
    //             () -> m_AlgaeSubsystem.stop(), m_AlgaeSubsystem)
    //     );

      
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
     m_driverController.rightBumper()
         .whileTrue(new RunCommand(
             () -> m_robotDrive.setX(),
             m_robotDrive));

    driverButtonBinder.getButton("x", "Reset Gyro")
    .whileTrue(new RunCommand(
        () -> m_robotDrive.gyroReset(),
            m_robotDrive));

    // operatorButtonBinder.getButton("x", "Coral Intake")
    //    .whileTrue(m_coralHandler.coralIntake(-0.2));

    //   operatorButtonBinder.getButton("leftBumper", "Coral Level 1")
    //   .whileTrue(m_coralHandler.coralBaseOutake(0.2));

    //   operatorButtonBinder.getButton("y", "Coral Outake")
    //    .whileTrue(m_coralHandler.coralIntake(0.2));

    //  operatorButtonBinder.getButton("start", "Algae Intake")
    //  .whileTrue(m_AlgaeSubsystem.AlgaeIntake(-0.2)).onFalse(m_AlgaeSubsystem.AlgaeIntake(0));

    //  operatorButtonBinder.getButton("back", "Algae outtake")
    //  .whileTrue(m_AlgaeSubsystem.AlgaeIntake(0.2)).onFalse(m_AlgaeSubsystem.AlgaeIntake(0));

//     driverButtonBinder.getButton("povLeft", "Go to L2")
//     .whileTrue(m_elevator.goToLiftL2Command());

//     driverButtonBinder.getButton("povRight", "Go to L3")
//     .whileTrue(m_elevator.goToLiftL3Command());

//     driverButtonBinder.getButton("povDown", "Go to bottom")
//     .whileTrue(m_elevator.goToLiftStowCommand());
// operatorButtonBinder.getButton("a", "Wrist to A1")
//     .whileTrue(m_Algaewrist.goToWristAngleCommand(WristAngle.A1));

//  operatorButtonBinder.getButton("rightBumper", "Wrist to Stow")
//      .whileTrue(m_Algaewrist.goToWristAngleCommand(WristAngle.STOW));

// operatorButtonBinder.getButton("b", "Wrist to A2")
//     .whileTrue(m_Algaewrist.goToWristAngleCommand(WristAngle.A2));

// // operatorButtonBinder.getButton("y", "algae Intake")
// //     .whileTrue(m_AlgaeSubsystem.AlgaeIntake(-.2));
// operatorButtonBinder.getButton("povUp", "Algae Outake")
//     .whileTrue(m_AlgaeSubsystem.AlgaeOutake(0.2));
//      driverButtonBinder.getButton("povUp", "Go to L4")
//      .whileTrue(m_elevator.goToLiftL4Command());
    // m_operatorController.a().whileTrue(m_AlgaeSubsystem.AlgaeIntake(0.2));
    // m_operatorController.b().whileTrue(m_AlgaeSubsystem.AlgaeOutake(.2));
    //m_operatorController.rightBumper().whileTrue
    //m_operatorController.leftBumper().whileTrue
//public void setRumble(GenericHID.RumbleType leftRumble,
//double 0.9 );
     driverButtonBinder.getButton("rightBumper", "Turn To Target").whileTrue(new TurnToTarget(m_robotDrive, m_vision));
     driverButtonBinder.getButton("leftBumper", "Move To Target").whileTrue(new MoveToTarget(m_robotDrive, m_vision));
  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory


    // Run path following command, then stop at the end.

    return autoChooser.getSelected();
  }
}
