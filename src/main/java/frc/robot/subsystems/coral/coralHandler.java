package frc.robot.subsystems.coral;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeoMotorConstants;

public class coralHandler extends SubsystemBase {
  private final SparkMax leftSparkMax;
  private final SparkMax rightSparkMax;

  private static final SparkMaxConfig sparkMaxConfigLeft = new SparkMaxConfig();
  private static final SparkMaxConfig sparkMaxConfigRight = new SparkMaxConfig();

  private static final boolean leftEncoderInverted = true;
  private static final boolean rightEncoderInverted = false;

  private static final double leftEncoderPositionFactor = 1.0;
  private static final double rightEncoderPositionFactor = 1.0;

  private static final double leftP = 0.095;
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

  @SuppressWarnings("unused")
  private final RelativeEncoder leftRelativeEncoder;
  @SuppressWarnings("unused")
  private final RelativeEncoder rightRelativeEncoder;

  private final SparkClosedLoopController leftPidController;
  @SuppressWarnings("unused")
  private final SparkClosedLoopController rightPidController;

  private boolean hasCoral = false;
  
  private LaserCan firstLaser;
  
  /** Creates a new coralHandler. */
  public coralHandler() {
    rightSparkMax = new SparkMax(51,  MotorType.kBrushless);
    leftSparkMax = new SparkMax(52, MotorType.kBrushless); 
 

    sparkMaxConfigLeft.inverted(leftEncoderInverted).idleMode(leftMotorIdleMode).smartCurrentLimit(NeoMotorConstants.NEOCurrentLimit);
    sparkMaxConfigLeft.encoder.positionConversionFactor(leftEncoderPositionFactor)
      .velocityConversionFactor(leftEncoderPositionFactor/60);
    sparkMaxConfigLeft.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(leftP, leftI, leftD, ClosedLoopSlot.kSlot1).outputRange(leftMinOutput, leftMaxOutput);
    sparkMaxConfigRight.inverted(rightEncoderInverted).idleMode(rightMotorIdleMode).smartCurrentLimit(NeoMotorConstants.NEOCurrentLimit);
    sparkMaxConfigRight.encoder.positionConversionFactor(rightEncoderPositionFactor)
      .velocityConversionFactor(rightEncoderPositionFactor/60);
    sparkMaxConfigRight.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(rightP, rightI, rightD, ClosedLoopSlot.kSlot1).outputRange(rightMinOutput, rightMaxOutput);

    leftSparkMax.configure(sparkMaxConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightSparkMax.configure(sparkMaxConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftRelativeEncoder = leftSparkMax.getEncoder();
    rightRelativeEncoder = rightSparkMax.getEncoder();

    leftPidController = leftSparkMax.getClosedLoopController();
    rightPidController = rightSparkMax.getClosedLoopController();
    
     firstLaser=new LaserCan(10);

     try {
       firstLaser.setRangingMode(LaserCan.RangingMode.SHORT);
       firstLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8 , 8));
       firstLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
     } catch (ConfigurationFailedException e) {
       System.out.println("Configuration failed " + e);
     }
  }

  @Override
  public void periodic() {
  LaserCan.Measurement measurement1 = firstLaser.getMeasurement();
 


  if ( ( measurement1 !=null && measurement1.distance_mm < 50)) {
  
   System.out.println("the target is " + measurement1.distance_mm + "mm away!");
   System.out.println("has coral");
   hasCoral=true;
 }
 else {
   System.out.println("no coral");
   hasCoral=false;
 }

     SmartDashboard.putBoolean("Sensor Eyes:",measurement1.status ==LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
     SmartDashboard.putNumber("sensor distance", measurement1.distance_mm); 

    
    
  }
  public void setIntakeMotors (double speed){
      leftSparkMax.set(speed);
      rightSparkMax.set(-speed);    
  }

  /**
   * Moves Intake Motors when Detecting Coral
   * @param speed
   */
  public void setAutoIntakeMotors (double speed){
      if(hasCoral){
       leftSparkMax.set(speed);
       rightSparkMax.set(-speed);
       System.out.print("Trying to intake");
      }
       else{
         leftSparkMax.set(0);
         rightSparkMax.set(0);
       }
     
   }
  public void pidSetIntakeMotors (double targetVelocity){
      leftPidController.setReference(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void setOutakeMotors (double speed){
    if(hasCoral){
      leftSparkMax.set(speed);
      rightSparkMax.set(speed);
    }
    else{
      leftSparkMax.set(0);
      rightSparkMax.set(0);
    }
  }
  public void setOutakeBaseMotor (double speed){
      leftSparkMax.set(speed);
      rightSparkMax.set(-speed/2);
  }

  public void stopMotors(){
    leftSparkMax.set(0);
  }

  public Command coralIntake (double velocity){
    return this.run(
      ()->{setIntakeMotors(velocity); 
        System.out.println("Coral Intaking");}
    );
  }

    public Command coralOutake (double speed){
      return this.run(
        ()->{setOutakeMotors(speed); 
          System.out.println("Coral Outaking");}
      );
  }
  public Command coralBaseOutake (double speed){
      return this.run(
        ()->{setOutakeBaseMotor(speed);
        System.out.println("Coral Outaking at Base");}
      );
  }
  public Command defaultCoralIntake (double velocity){
    return this.run(
      ()->{setAutoIntakeMotors(velocity); 
        System.out.println("Coral Intaking");}
    );
  }
}
