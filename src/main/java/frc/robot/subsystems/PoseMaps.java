package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PoseMaps {

    private final AprilTagFieldLayout aprilTagsLayout =
      AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
    public HashMap<Double, Pose2d> poses = setHashMap();

    private double coralOffset = Units.inchesToMeters(6);
    public PoseMaps(){
        poses = setHashMap();
        SmartDashboard.putString("Plain Pose", ""+plainPose);
        System.out.println(poses);
        
    }

    private Pose2d aprilPose6= new Pose2d(
        aprilTagsLayout.getTagPose(6).get().getX(),
        aprilTagsLayout.getTagPose(6).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(6).get().getRotation().getZ()));
    private Pose2d aprilPose7= new Pose2d(
        aprilTagsLayout.getTagPose(7).get().getX(),
        aprilTagsLayout.getTagPose(7).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(7).get().getRotation().getZ()));
    private Pose2d aprilPose8= new Pose2d(
        aprilTagsLayout.getTagPose(8).get().getX(),
        aprilTagsLayout.getTagPose(8).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(8).get().getRotation().getZ()));
    private Pose2d aprilPose9= new Pose2d(
        aprilTagsLayout.getTagPose(9).get().getX(),
        aprilTagsLayout.getTagPose(9).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(9).get().getRotation().getZ()));        
    private Pose2d aprilPose10= new Pose2d(
        aprilTagsLayout.getTagPose(10).get().getX(),
        aprilTagsLayout.getTagPose(10).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(10).get().getRotation().getZ()));
    private Pose2d aprilPose11= new Pose2d(
        aprilTagsLayout.getTagPose(11).get().getX(),
        aprilTagsLayout.getTagPose(11).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(11).get().getRotation().getZ()));
    private Pose2d aprilPose17= new Pose2d(
        aprilTagsLayout.getTagPose(17).get().getX(),
        aprilTagsLayout.getTagPose(17).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(17).get().getRotation().getZ()));            
    private Pose2d aprilPose18= new Pose2d(
        aprilTagsLayout.getTagPose(18).get().getX(),
        aprilTagsLayout.getTagPose(18).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(18).get().getRotation().getZ()));  
    private Pose2d aprilPose19= new Pose2d(
        aprilTagsLayout.getTagPose(19).get().getX(),
        aprilTagsLayout.getTagPose(19).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(19).get().getRotation().getZ()));  
    private Pose2d aprilPose20= new Pose2d(
        aprilTagsLayout.getTagPose(20).get().getX(),
        aprilTagsLayout.getTagPose(20).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(20).get().getRotation().getZ())); 
    private Pose2d aprilPose21= new Pose2d(
        aprilTagsLayout.getTagPose(21).get().getX(),
        aprilTagsLayout.getTagPose(21).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(21).get().getRotation().getZ()));    
    private Pose2d aprilPose22= new Pose2d(
        aprilTagsLayout.getTagPose(22).get().getX(),
        aprilTagsLayout.getTagPose(22).get().getY(),
        new Rotation2d(aprilTagsLayout.getTagPose(22).get().getRotation().getZ()));         
    public enum Direction{
        LEFT,
        RIGHT,
        NONE
    }
    private Direction direction = Direction.NONE;
    public HashMap<Double, Pose2d> setHashMap(){
        HashMap<Double, Pose2d> hi = new HashMap<Double, Pose2d>();
        List<Double> doubleList = Arrays.asList(6.0,7.0,8.0,9.0,10.0,11.0,17.0,18.0,19.0,20.0,21.0,22.0);

        // Using for-each loop to iterate through the list
        for (double value : doubleList) {
            hi.put(value, new Pose2d(
                aprilTagsLayout.getTagPose(((int)value)).get().getX(),
                aprilTagsLayout.getTagPose((int)value).get().getY(),
                new Rotation2d(aprilTagsLayout.getTagPose((int)value).get().getRotation().getZ())));
        }
        //System.out.println(hi);
        return hi;
    }
    /**
     * Returns a Pose2d using cosine and sine to calculate offset to correct bot pose for coral scoring
     * @param aprilTag
     * @param direction
     * @return botPose2d
     */
   public Pose2d plainPose;
   public double plainTheta;
    public double thetaCos ;
    public double thetaSin ;
    public double xAprilTag ;
    public double yAprilTag ;
    public double xBot;
    public double yBot;
    public double thetaBot;
    public Pose2d getPose2d(double aprilTag, Direction direction){
        SmartDashboard.putString("aprilTag", ""+aprilTag);
         plainPose = poses.get(aprilTag);
         if(plainPose == null){
            SmartDashboard.putString("plainPose Null?", "Yes");
         }
          
        plainTheta = plainPose.getRotation().getRadians();
         thetaCos = plainPose.getRotation().getCos();
         thetaSin = plainPose.getRotation().getSin();
         xAprilTag = plainPose.getX();
         yAprilTag = plainPose.getY();
        
         xBot = 0;
         yBot = 0;
         thetaBot = 0;
         SmartDashboard.putString("Plain Pose", ""+plainPose);
         
        //SmartDashboard.putString("plain theta", ""+plainPose.getRotation().getRadians());
        
        if(direction == Direction.RIGHT){
            xBot = (xAprilTag + thetaCos * coralOffset) + 
            Math.cos(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            yBot = (yAprilTag + thetaSin * coralOffset) + 
            Math.sin(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            thetaBot = plainTheta - 3.14;
            SmartDashboard.putString("Direction Directing", "Right");

        }   
        else if(direction == Direction.LEFT){
            xBot = (xAprilTag - thetaCos * coralOffset) + 
            Math.cos(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            yBot = (yAprilTag - thetaSin * coralOffset) + 
            Math.sin(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            thetaBot = plainTheta - 3.14;
            SmartDashboard.putString("Direction Directing", "Left");
        }
        else{
            SmartDashboard.putString("Direction Directing", "None");
        }

        Pose2d botPose2d = new Pose2d(xBot, yBot, new Rotation2d(thetaBot));
        System.out.println("///////////////////////////////////////");
        System.out.println(botPose2d);
        System.out.println("///////////////////////////////////////");
        return botPose2d;
    }
}
