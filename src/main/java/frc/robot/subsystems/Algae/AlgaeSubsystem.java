package frc.robot.subsystems.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NeoMotorConstants;




public class AlgaeSubsystem extends SubsystemBase {
  
  
  @SuppressWarnings("unused")
  private TrapezoidProfile mProfile;
  @SuppressWarnings("unused")
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  @SuppressWarnings("unused")
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  @SuppressWarnings("unused")
  private double prevUpdateTime = Timer.getFPGATimestamp();



  private final SparkMax algaeIntakeMotor;

  private static final SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();

  private static final boolean intakeInverted = false;

  private static final double intakePositionFactor = 1;

  private static final double intake_P = 0.01;
  private static final double intake_I = 0;
  private static final double intake_D = 0;

  private static final double minOutput = -.2;
  private static final double maxOutput = .2;

  private static final SparkMaxConfig.IdleMode motorIdleMode = SparkBaseConfig.IdleMode.kBrake;

  @SuppressWarnings("unused")
  private final RelativeEncoder intakeRelativeEncoder;

  @SuppressWarnings("unused")
  private final SparkClosedLoopController intakePidController;


  
/** Creates a new Algae Subsystem */
  public AlgaeSubsystem() {

    algaeIntakeMotor = new SparkMax(61, MotorType.kBrushless);  

    algaeIntakeConfig.inverted(intakeInverted).idleMode(motorIdleMode).smartCurrentLimit(NeoMotorConstants.NEOCurrentLimit);
    algaeIntakeConfig.encoder.positionConversionFactor(intakePositionFactor)
      .velocityConversionFactor(intakePositionFactor);
    algaeIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(intake_P, intake_I, intake_D,ClosedLoopSlot.kSlot1).outputRange(minOutput, maxOutput);

    algaeIntakeMotor.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeRelativeEncoder =  algaeIntakeMotor.getEncoder();

    intakePidController = algaeIntakeMotor.getClosedLoopController();

    mProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(Constants.Elevator.kMaxVelocity, Constants.Elevator.kMaxAcceleration));


  }

  public void intakeAlgae (double speed){
   algaeIntakeMotor.set(speed);
  }


  public void outakeAlgae (double speed){
    algaeIntakeMotor.set(speed);
  }

  public void stop(){
    algaeIntakeMotor.set(0);
  }

public Command AlgaeIntake (double velocity){
  return this.startEnd(
    ()->{intakeAlgae(velocity); 
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
    
  }

}
