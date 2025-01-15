package frc.robot.subsystems.Algae;

import edu.wpi.first.math.controller.PIDController;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




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




public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */

  private final SparkMax algaeSparkMax;

  private static final SparkMaxConfig algaeSparkMaxConfig = new SparkMaxConfig();

  private static final boolean encoderInverted = false;

  private static final double encoderPositionFactor = 1;
  private static final double P = 0.01;
  private static final double I = 0;
  private static final double D = 0;
  private static final double minOutput = -1;
  private static final double maxOutput = 1;

  private static final SparkMaxConfig.IdleMode motorIdleMode = SparkBaseConfig.IdleMode.kBrake;
  private final RelativeEncoder relativeEncoder ;
  private final SparkClosedLoopController pidController ;

  

  public AlgaeSubsystem() {

    algaeSparkMax = new SparkMax(2, MotorType.kBrushless);  
    algaeSparkMaxConfig.inverted(encoderInverted).idleMode(motorIdleMode);
    algaeSparkMaxConfig.encoder.positionConversionFactor(encoderPositionFactor)
      .velocityConversionFactor(encoderPositionFactor);
    algaeSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(P, I, D).outputRange(minOutput, maxOutput);


  

    algaeSparkMax.configure(algaeSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    relativeEncoder =  algaeSparkMax.getEncoder();
    pidController = algaeSparkMax.getClosedLoopController();


  }
public void intakealgaeDoohickey (double speed){
 algaeSparkMax.set(speed);
}

public void PIDactivatealgaeDoohickey (double targetVelocity){
  pidController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);

}

public void reversealgaeDoohickey (double speed){
algaeSparkMax.set(speed);
}

public void stopthealgaeDoohickey(){
algaeSparkMax.set(0);
}

public Command AlgaeIntake (double velocity){
  return this.startEnd(
    ()->{PIDactivatealgaeDoohickey(velocity); 
      System.out.println("Grabbing Algae.");},
    ()-> stopthealgaeDoohickey());
    }

public Command AlgaeOutake (double speed){
  return this.startEnd(
    ()->{reversealgaeDoohickey(speed); 
      System.out.println("Algae Outaking");},
    ()-> stopthealgaeDoohickey());
    }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
