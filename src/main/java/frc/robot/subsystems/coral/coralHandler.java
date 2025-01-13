// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoMotorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ColorSensorV3.RawColor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class coralHandler extends SubsystemBase {
  private final SparkMax leftSparkMax;
  private final SparkMax rightSparkMax;

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

  private final RelativeEncoder leftRelativeEncoder;
  private final RelativeEncoder rightRelativeEncoder;

  private final SparkClosedLoopController leftPidController;
  private final SparkClosedLoopController rightPidController;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private boolean hasCoral = false;

  /** Creates a new coral. */
  public coralHandler() {
 

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

    leftSparkMax = new SparkMax(0, MotorType.kBrushless);  
    rightSparkMax = new SparkMax(1,  MotorType.kBrushless);


    leftSparkMax.configure(sparkMaxConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightSparkMax.configure(sparkMaxConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftRelativeEncoder = leftSparkMax.getEncoder();
    rightRelativeEncoder = rightSparkMax.getEncoder();

    leftPidController = leftSparkMax.getClosedLoopController();
    rightPidController = rightSparkMax.getClosedLoopController();
    
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    int proximity = colorSensor.getProximity();

    System.out.println("Proximity: " + proximity);
    
    // Proximity is higher when the object is closer
    // Proximity value was ~200 when PVC pipe was 2 inches away
    // Proximity value was ~60-70 when nothing was in front of the sensor
    if (proximity > 100){
      System.out.println("PVC visible");
    }
    else{
      System.out.println("PVC not visible");
    } 
  }
  public void setIntakeMotors (double speed){
    if(!hasCoral){
      leftSparkMax.set(speed);
      rightSparkMax.set(speed);
    }
    
  }
  public void pidSetIntakeMotors (double targetVelocity){
    if(!hasCoral){
      leftPidController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
      rightPidController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }
    
  }

  public void setOutakeMotors (double speed){
    if(hasCoral){
      leftSparkMax.set(speed);
      rightSparkMax.set(speed);
    }
    
  }

  public void stopMotors(){
    leftSparkMax.set(0);
    rightSparkMax.set(0);
  }

  // private int getRed(){
  //   return colorSensor.getRed();
  // }

  // private int getBlue(){
  //   return colorSensor.getBlue();
  // }

  // private int getGreen(){
  //   return colorSensor.getGreen();
  // }


  // public Command coralIntake (double speed){
  //   return this.run(
  //     ()->{setIntakeMotors(speed); 
  //          System.out.println("Coral Intaking");}
  //        ).finallyDo(interrupted->stopMotors()); 
  // }

  public Command coralIntake (double velocity){
    return this.startEnd(
      ()->{pidSetIntakeMotors(velocity); 
        System.out.println("Coral Intaking");},
      ()-> stopMotors()
    );
  }

    public Command coralOutake (double speed){
      return this.startEnd(
        ()->{setOutakeMotors(speed); 
          System.out.println("Coral Outaking");},
        ()-> stopMotors()
      );
  }


}
