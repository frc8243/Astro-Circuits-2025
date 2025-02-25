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
    private HashMap<Double, Pose2d> poses = setHashMap();

    private double coralOffset = Units.inchesToMeters(6);
    

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
        HashMap<Double, Pose2d> poses = new HashMap<Double, Pose2d>();
        List<Double> doubleList = Arrays.asList(6.0,7.0,8.0,9.0,10.0,11.0,17.0,18.0,19.0,20.0,21.0,22.0);

        // Using for-each loop to iterate through the list
        for (double value : doubleList) {
            poses.put(value, new Pose2d(
                aprilTagsLayout.getTagPose(((int)value)).get().getX(),
                aprilTagsLayout.getTagPose((int)value).get().getY(),
                new Rotation2d(aprilTagsLayout.getTagPose((int)value).get().getRotation().getZ())));
        }
        System.out.println(poses);
        return poses;
    }
    /**
     * Returns a Pose2d using cosine and sine to calculate offset to correct bot pose for coral scoring
     * @param aprilTag
     * @param direction
     * @return botPose2d
     */
    public Pose2d getPose2d(double aprilTag, Direction direction){
        
        Pose2d plainPose = poses.get(aprilTag);
        double plainTheta = plainPose.getRotation().getRadians();
        double thetaCos = plainPose.getRotation().getCos();
        double thetaSin = plainPose.getRotation().getSin();
        double xAprilTag = plainPose.getX();
        double yAprilTag = plainPose.getY();
        
        double xBot = 0;
        double yBot = 0;
        double thetaBot = 0;
        
        if(direction == Direction.RIGHT){
            xBot = (xAprilTag + thetaCos * coralOffset) + 
            Math.cos(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            yBot = (yAprilTag + thetaSin * coralOffset) + 
            Math.sin(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            thetaBot = plainTheta - Math.PI;
            SmartDashboard.putString("Direction Directing", "Right");

        }   
        else if(direction == Direction.LEFT){
            xBot = (xAprilTag - thetaCos * coralOffset) + 
            Math.cos(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            yBot = (yAprilTag - thetaSin * coralOffset) + 
            Math.sin(-(Math.PI/2) + plainTheta) *  Units.inchesToMeters(18.5);

            thetaBot = plainTheta - Math.PI;
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
