// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class elevator extends SubsystemBase {
  /** Creates a new elevator. */

  private PeriodicIO m_PeriodicIO;
  
  private final SparkMax leftSparkMax;
  private final SparkMax rightSparkMax;

  private final RelativeEncoder leftRelativeEncoder;
  private final RelativeEncoder rightRelativeEncoder;

  private final SparkClosedLoopController leftPidController;
  private final SparkClosedLoopController rightPidController;  

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  private static final SparkMaxConfig sparkMaxConfigLeft = new SparkMaxConfig();
  private static final SparkMaxConfig sparkMaxConfigRight = new SparkMaxConfig();

  private static final boolean leftEncoderInverted = false;
  private static final boolean rightEncoderInverted = true;

  private static final double leftEncoderPositionFactor = 1.12;
  private static final double rightEncoderPositionFactor = 1.12;

  private static final double leftP = 0.1;
  private static final double leftI = 0;
  private static final double leftD = 0;
  //private static final double LeftFF = 1 / ;
  private static final double leftMinOutput = -.5;
  private static final double leftMaxOutput = .5;

  private static final double rightP = 0.1;
  private static final double rightI = 0;
  private static final double rightD = 0.001;
  //private static final double rightFF = 1.0;
  private static final double rightMinOutput = -.5;
  private static final double rightMaxOutput = .5;

  private static final SparkMaxConfig.IdleMode leftMotorIdleMode = SparkBaseConfig.IdleMode.kBrake;
  private static final SparkMaxConfig.IdleMode rightMotorIdleMode = SparkBaseConfig.IdleMode.kBrake;
  
  
  
    public elevator() {
      m_PeriodicIO = new PeriodicIO();
      leftSparkMax = new SparkMax(8, MotorType.kBrushless);  
      rightSparkMax = new SparkMax(7,  MotorType.kBrushless);

      sparkMaxConfigLeft.inverted(leftEncoderInverted).idleMode(leftMotorIdleMode);
      sparkMaxConfigLeft.encoder.positionConversionFactor(leftEncoderPositionFactor)
        .velocityConversionFactor(leftEncoderPositionFactor/60);
      sparkMaxConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(leftP, leftI, leftD, ClosedLoopSlot.kSlot0).outputRange(leftMinOutput, leftMaxOutput);
     
        sparkMaxConfigRight.inverted(rightEncoderInverted).idleMode(rightMotorIdleMode);
      sparkMaxConfigRight.encoder.positionConversionFactor(rightEncoderPositionFactor)
        .velocityConversionFactor(rightEncoderPositionFactor/60);
      sparkMaxConfigRight.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(rightP, rightI, rightD, ClosedLoopSlot.kSlot0).outputRange(rightMinOutput, rightMaxOutput);
  
  

    leftSparkMax.configure(sparkMaxConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightSparkMax.configure(sparkMaxConfigRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftRelativeEncoder = leftSparkMax.getEncoder();
    rightRelativeEncoder = rightSparkMax.getEncoder();
    leftPidController = leftSparkMax.getClosedLoopController();
    rightPidController = rightSparkMax.getClosedLoopController();

    mProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));


    //goToElevatorStow();
  }
  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }
   
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator/Left Encoder", leftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Right Encoder", rightRelativeEncoder.getPosition());
    outputTelemetry();
    writePeriodicOutputs();
    // This method will be called once per scheduler run

  }


  public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (m_PeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = m_PeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      leftPidController.setReference(
          mCurState.position,
          SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
           Constants.Elevator.kG,
          ArbFFUnits.kVoltage);

      rightPidController.setReference(
          mCurState.position,
          SparkMax.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
           Constants.Elevator.kG,
          ArbFFUnits.kVoltage);

      
    } else {
      mCurState.position = leftRelativeEncoder.getPosition();
      mCurState.velocity = 0;
      leftSparkMax.set(m_PeriodicIO.elevator_power);
      rightSparkMax.set(m_PeriodicIO.elevator_power);
    }

  }
  
  public void stop() {
    m_PeriodicIO.is_elevator_pos_control = false;
    m_PeriodicIO.elevator_power = 0.0;

    leftSparkMax.set(0.0);
  }
  
  public void outputTelemetry() {
    SmartDashboard.putNumber("Elevator/Position/Current", leftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Elevator/Position/Target", m_PeriodicIO.elevator_target);
    SmartDashboard.putNumber("Elevator/Velocity/Current", leftRelativeEncoder.getVelocity());

    SmartDashboard.putNumber("Elevator/Position/Setpoint", mCurState.position);
    SmartDashboard.putNumber("Elevator/Velocity/Setpoint", mCurState.velocity);

    SmartDashboard.putNumber("Elevator/Current/Left", leftSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Elevator/Current/Right", rightSparkMax.getOutputCurrent());

    SmartDashboard.putNumber("Elevator/Output/Left", leftSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Elevator/Output/Right", rightSparkMax.getAppliedOutput());

    SmartDashboard.putString("Elevator/State", "" + getState());
  }

  
  public void reset() {
    leftRelativeEncoder.setPosition(0.0);
  }


  public ElevatorState getState() {
    return m_PeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    SmartDashboard.putNumber("setElevatorPower", power);
    m_PeriodicIO.is_elevator_pos_control = false;
    m_PeriodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    m_PeriodicIO.is_elevator_pos_control = true;
    m_PeriodicIO.elevator_target = Constants.Elevator.kStowHeight;
    m_PeriodicIO.state = ElevatorState.STOW;
    System.out.println("Going to Stow");
  }
  public Command goToLiftStowCommand (){
    return this.run(
      ()->{goToElevatorStow(); 
        System.out.println("Elevate to Stow");}
    );
  }

  public void goToElevatorL2() {
    m_PeriodicIO.is_elevator_pos_control = true;
    m_PeriodicIO.elevator_target = Constants.Elevator.kL2Height;
    m_PeriodicIO.state = ElevatorState.L2;
    System.out.println("Elevate to L2");
  }

  public Command goToLiftL2Command (){ 
    return this.run(
      ()->{goToElevatorL2();}
    );
  }

  public void goToElevatorL3() {
    m_PeriodicIO.is_elevator_pos_control = true;
    m_PeriodicIO.elevator_target = Constants.Elevator.kL3Height;
    m_PeriodicIO.state = ElevatorState.L3;
  }
  public Command goToLiftL3Command (){
    return this.run(
      ()->{goToElevatorL3(); 
        System.out.println("Elevate to L3");}
    );
  }
  public void goToElevatorL4() {
    m_PeriodicIO.is_elevator_pos_control = true;
    m_PeriodicIO.elevator_target = Constants.Elevator.kL4Height;
    m_PeriodicIO.state = ElevatorState.L4;
  }
  public Command goToLiftL4Command (){
    return this.run(
      ()->{goToElevatorL4();
        System.out.println("Elevate to L4");}
    );
  }
  public void goToAlgaeLow() {
    m_PeriodicIO.is_elevator_pos_control = true;
    m_PeriodicIO.elevator_target = Constants.Elevator.kLowAlgaeHeight;
    m_PeriodicIO.state = ElevatorState.A1;
  }

  public void goToAlgaeHigh() {
    m_PeriodicIO.is_elevator_pos_control = true;
    m_PeriodicIO.elevator_target = Constants.Elevator.kHighAlgaeHeight;
    m_PeriodicIO.state = ElevatorState.A2;
  }




}
