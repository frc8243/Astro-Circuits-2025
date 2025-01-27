package frc.robot.subsystems.Algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.subsystems.elevator.elevator.ElevatorState;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;




public class AlgaeSubsystem extends SubsystemBase {
  
  
  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  private PeriodicIO m_PeriodicIO;

  private final SparkMax algaeIntakeMotor;
  private final SparkMax algaeLiftMotor;

  private static final SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
  private static final SparkMaxConfig algaeLiftConfig = new SparkMaxConfig();

  private static final boolean intakeInverted = false;
  private static final boolean liftInverted = false;

  private static final double intakePositionFactor = 1;
  private static final double liftPositionFactor = 1;

  private static final double intake_P = 0.01;
  private static final double intake_I = 0;
  private static final double intake_D = 0;

  private static final double lift_P = 0.01;
  private static final double lift_I = 0;
  private static final double lift_D = 0;

  private static final double minOutput = -1;
  private static final double maxOutput = 1;

  private static final SparkMaxConfig.IdleMode motorIdleMode = SparkBaseConfig.IdleMode.kBrake;

  private final RelativeEncoder intakeRelativeEncoder;
  private final RelativeEncoder liftRelativeEncoder;

  private final SparkClosedLoopController intakePidController;
  private final SparkClosedLoopController liftPidController;

  

  public AlgaeSubsystem() {
    m_PeriodicIO = new PeriodicIO();
    algaeLiftMotor = new SparkMax(61, MotorType.kBrushless);  
    algaeIntakeMotor = new SparkMax(62, MotorType.kBrushless);  
    algaeIntakeConfig.inverted(intakeInverted).idleMode(motorIdleMode);
    algaeIntakeConfig.encoder.positionConversionFactor(intakePositionFactor)
      .velocityConversionFactor(intakePositionFactor);
    algaeIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(intake_P, intake_I, intake_D,ClosedLoopSlot.kSlot1).outputRange(minOutput, maxOutput);

    algaeLiftConfig.inverted(liftInverted).idleMode(motorIdleMode);
    algaeLiftConfig.encoder.positionConversionFactor(liftPositionFactor)
      .velocityConversionFactor(liftPositionFactor/60);
    algaeLiftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(lift_P, lift_I, lift_D, ClosedLoopSlot.kSlot0).outputRange(minOutput, maxOutput);

  

    algaeIntakeMotor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeLiftMotor.configure(algaeLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeRelativeEncoder =  algaeIntakeMotor.getEncoder();
    liftRelativeEncoder = algaeLiftMotor.getEncoder();

    intakePidController = algaeIntakeMotor.getClosedLoopController();
    liftPidController = algaeLiftMotor.getClosedLoopController();

    mProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));


  }
public void intakeAlgae (double speed){
 algaeIntakeMotor.set(speed);
}

public void pidIntake (double targetVelocity){
  liftPidController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);

}

public void outakeAlgae (double speed){
algaeIntakeMotor.set(speed);
}

public void stop(){
algaeIntakeMotor.set(0);
}

public Command AlgaeIntake (double velocity){
  return this.startEnd(
    ()->{pidIntake(velocity); 
      System.out.println("Grabbing Algae.");},
    ()-> stop());
    }

public Command AlgaeOutake (double speed){
  return this.startEnd(
    ()->{outakeAlgae(speed); 
      System.out.println("Algae Outaking");},
    ()-> stop());
    }

    
  @Override
  public void periodic() {
    writePeriodicOutputs();
  }
  public enum AlgaeState {
    NONE,
    STOW,
    A1,
    A2,
  }

  private static class PeriodicIO {
    double algae_target = 0.0;
    double algae_power = 0.0;

    boolean is_algae_pos_control = false;

    AlgaeState state = AlgaeState.STOW;
  }

  public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (m_PeriodicIO.is_algae_pos_control) {
      // Update goal
      mGoalState.position = m_PeriodicIO.algae_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      liftPidController.setReference(
          mCurState.position,
          SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
           Constants.Elevator.kG,
          ArbFFUnits.kVoltage);



      
    } else {
      mCurState.position = liftRelativeEncoder.getPosition();
      mCurState.velocity = 0;
      algaeLiftMotor.set(m_PeriodicIO.algae_power);
      
    }

  }  
  public void reset() {
    liftRelativeEncoder.setPosition(0.0);
  }
  public AlgaeState getState() {
    return m_PeriodicIO.state;
  }

  public void setLiftPower(double power) {
    SmartDashboard.putNumber("setLiftPower", power);
    m_PeriodicIO.is_algae_pos_control = false;
    m_PeriodicIO.algae_power = power;
  }
  public void goToAlgaeStow() {
    m_PeriodicIO.is_algae_pos_control = true;
    m_PeriodicIO.algae_target = Constants.Algae.kStowHeight;
    m_PeriodicIO.state = AlgaeState.STOW;
    System.out.println("Going to Stow");
  }
  public Command goToLiftStowCommand (){
    return this.run(
      ()->{goToAlgaeStow(); 
        System.out.println("Elevate to Stow");}
    );
  }

  public void goToAlgaeA1() {
    m_PeriodicIO.is_algae_pos_control = true;
    m_PeriodicIO.algae_target = Constants.Algae.kA1Height;
    m_PeriodicIO.state = AlgaeState.A1;
    System.out.println("Elevate to A1");
  }

  public Command goToAlgaeA1Command (){ 
    return this.run(
      ()->{goToAlgaeA1();}
    );
  }

  public void goToAlgaeA2() {
    m_PeriodicIO.is_algae_pos_control = true;
    m_PeriodicIO.algae_target = Constants.Algae.kA2Height;
    m_PeriodicIO.state = AlgaeState.A2;
  }
  public Command goToAlgaeA2Command (){
    return this.run(
      ()->{goToAlgaeA2(); 
        System.out.println("Elevate to A2");}
    );
  }
}
