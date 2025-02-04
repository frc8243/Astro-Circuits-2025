package frc.robot.subsystems.Algae;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NeoMotorConstants;

public class AlgaeWrist extends SubsystemBase {

    private final SparkMax algaeWristMotor = new SparkMax(62, MotorType.kBrushless);
    private final SparkClosedLoopController wristPidController = algaeWristMotor.getClosedLoopController();
    private final double kWristKS = 0;
    private final double kWristKG = 0.03;
    private final double kWristKV = 3.01;
    private final ArmFeedforward mWristFeedForward = new ArmFeedforward(
        kWristKS,
        kWristKG,
        kWristKV);
  private final SparkMaxConfig sparkMaxConfigWrist = new SparkMaxConfig();
  private final double wristEncoderPositionFactor = 1/154.166 * 2 * Math.PI;
  private final double wristP = 0.2/(Math.PI/2);
  private final double wristI = 0;
  private final double wristD = 0;
  private final RelativeEncoder wristEncoder = algaeWristMotor.getEncoder();
    /** Creates a new AlgaeWrist. */
    private static final SparkMaxConfig.IdleMode wristIdleMode = SparkBaseConfig.IdleMode.kBrake;
    public AlgaeWrist() { sparkMaxConfigWrist.inverted(false).idleMode(wristIdleMode).smartCurrentLimit(NeoMotorConstants.NEOCurrentLimit);
     sparkMaxConfigWrist.encoder.positionConversionFactor(wristEncoderPositionFactor)
       .velocityConversionFactor(wristEncoderPositionFactor/60);
     sparkMaxConfigWrist.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
       .pid(wristP, wristI, wristD, ClosedLoopSlot.kSlot1).outputRange(-.2, .2);
}
  
    public enum WristAngle {
      NONE(Units.degreesToRadians(0)),
      STOW(Units.degreesToRadians(-90)),
      A1(Units.degreesToRadians(-45.06)),
      A2(Units.degreesToRadians(-45));
  
        private final double m_angle;
  
        WristAngle(double angle) {
          m_angle = angle;
        }
    }
  
    private WristAngle angleEnum = WristAngle.NONE;
  
    @Override
    public void periodic() {
      SmartDashboard.putNumber("AlgaeWrist/Angle", wristEncoder.getPosition());
      SmartDashboard.putString("State", ""+angleEnum);
      // This method will be called once per scheduler run
    }
  
    private void goToWristAngle(double angle)
    {
      double ffCalc = mWristFeedForward.calculate((angle), 0.0);
        wristPidController.setReference(angle,
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0, ffCalc);
    }
    public Command goToWristAngleCommand (WristAngle angleEnum){
      return this.run(
        ()->{goToWristAngle(angleEnum.m_angle); 
          this.angleEnum = angleEnum;
          System.out.println("Move Wrist to "  + angleEnum.toString());}
      );
    }
    public void stop() {
  

        algaeWristMotor.set(0.0);
    
        
      }
    
  
  }