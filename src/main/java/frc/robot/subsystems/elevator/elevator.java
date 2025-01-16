// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

  private PeriodicIO mPeriodicIO;
  
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

  private static final boolean leftEncoderInverted = true;
  private static final boolean rightEncoderInverted = false;

  private static final double leftEncoderPositionFactor = 0.0;
  private static final double rightEncoderPositionFactor = 0.0;

  private static final double leftP = 0.01;
  private static final double leftI = 0;
  private static final double leftD = 0;
  //private static final double LeftFF = 1 / ;
  private static final double leftMinOutput = -1;
  private static final double leftMaxOutput = 1;

  private static final double rightP = 0.01;
  private static final double rightI = 0;
  private static final double rightD = 0.001;
  //private static final double rightFF = 1.0;
  private static final double rightMinOutput = -1;
  private static final double rightMaxOutput = 1;

  private static final SparkMaxConfig.IdleMode leftMotorIdleMode = SparkBaseConfig.IdleMode.kBrake;
  private static final SparkMaxConfig.IdleMode rightMotorIdleMode = SparkBaseConfig.IdleMode.kBrake;
  
  
  
    public elevator() {

      sparkMaxConfigLeft.inverted(leftEncoderInverted).idleMode(leftMotorIdleMode);
      sparkMaxConfigLeft.encoder.positionConversionFactor(leftEncoderPositionFactor)
        .velocityConversionFactor(leftEncoderPositionFactor);
      sparkMaxConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(leftP, leftI, leftD).outputRange(leftMinOutput, leftMaxOutput);
     
        sparkMaxConfigRight.inverted(rightEncoderInverted).idleMode(rightMotorIdleMode);
      sparkMaxConfigRight.encoder.positionConversionFactor(rightEncoderPositionFactor)
        .velocityConversionFactor(rightEncoderPositionFactor);
      sparkMaxConfigRight.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(rightP, rightI, rightD).outputRange(rightMinOutput, rightMaxOutput);
  
  
    leftSparkMax = new SparkMax(16, MotorType.kBrushless);  
    rightSparkMax = new SparkMax(29,  MotorType.kBrushless);
    leftSparkMax.configure(sparkMaxConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightSparkMax.configure(sparkMaxConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftRelativeEncoder = leftSparkMax.getEncoder();
    rightRelativeEncoder = rightSparkMax.getEncoder();
    leftPidController = leftSparkMax.getClosedLoopController();
    rightPidController = rightSparkMax.getClosedLoopController();

    mProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));



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
    // This method will be called once per scheduler run
  }


  public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = mPeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      leftPidController.setReference(
          mCurState.position,
          SparkMax.ControlType.kPosition,
          null,//TODO: CHECK THIS
           Constants.Elevator.kG,
          ArbFFUnits.kVoltage);
    } else {
      mCurState.position = leftRelativeEncoder.getPosition();
      mCurState.velocity = 0;
      leftSparkMax.set(mPeriodicIO.elevator_power);
    }

  }
  
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    leftSparkMax.set(0.0);
  }
  
  public void outputTelemetry() {
    SmartDashboard.putNumber("Position/Current", leftRelativeEncoder.getPosition());
    SmartDashboard.putNumber("Position/Target", mPeriodicIO.elevator_target);
    SmartDashboard.putNumber("Velocity/Current", leftRelativeEncoder.getVelocity());

    SmartDashboard.putNumber("Position/Setpoint", mCurState.position);
    SmartDashboard.putNumber("Velocity/Setpoint", mCurState.velocity);

    SmartDashboard.putNumber("Current/Left", leftSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("Current/Right", rightSparkMax.getOutputCurrent());

    SmartDashboard.putNumber("Output/Left", leftSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("Output/Right", rightSparkMax.getAppliedOutput());

    SmartDashboard.putString("State", ""+mPeriodicIO.state);
  }

  
  public void reset() {
    leftRelativeEncoder.setPosition(0.0);
  }


  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    SmartDashboard.putNumber("setElevatorPower", power);
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kStowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }
  public Command goToLiftStowCommand (){
    return this.startEnd(
      ()->{goToElevatorStow();; 
        System.out.println("Elevate to Stow");},
      ()-> stop()
    );
  }

  public void goToElevatorL2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL2Height;
    mPeriodicIO.state = ElevatorState.L2;
  }

  public Command goToLiftL2Command (){ 
    return this.startEnd(
      ()->{goToElevatorL2();; 
        System.out.println("Elevate to L2");},
      ()-> stop()
    );
  }

  public void goToElevatorL3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }
  public Command goToLiftL3Command (){
    return this.startEnd(
      ()->{goToElevatorL3();; 
        System.out.println("Elevate to L3");},
      ()-> stop()
    );
  }
  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kL4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }
  public Command goToLiftL4Command (){
    return this.startEnd(
      ()->{goToElevatorL4();
        System.out.println("Elevate to L4");},
      ()-> stop()
    );
  }
  public void goToAlgaeLow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kLowAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A1;
  }

  public void goToAlgaeHigh() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.kHighAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A2;
  }




}
