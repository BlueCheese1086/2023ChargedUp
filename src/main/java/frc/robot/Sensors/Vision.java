package frc.robot.Sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    
    //private final NetworkTable camera;
    
    private static final Camera frontFollowRight = new Camera(
        "rightcam",
        new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(-13.9375)));
    private static final Camera frontPrimLeft = new Camera(
        "leftcam", 
        new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(13.9375)));

    private final static Camera[] cameras = new Camera[]{
        frontPrimLeft, frontFollowRight
    };

    //private final ArrayList<PhotonPipelineResult> results = new ArrayList<>();
    private static final ArrayList<Target> targets = new ArrayList<>();
    //private final HashMap<Target, Camera> processedTargets = new HashMap<>();
    private static final ArrayList<StereoTarget> stereoTargets = new ArrayList<>();
    private static final HashMap<Integer, AprilTag> aprilTags = new HashMap<>();

    private static final AprilTag[] tags = new AprilTag[]{
        new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.PI))),
        new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.PI))),
        new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0, Math.PI))),
        new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0, 0, Math.PI))),
        new AprilTag(5, new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0, 0, 0))),
        new AprilTag(6, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0))),
        new AprilTag(7, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0))),
        new AprilTag(8, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0, 0, 0))),
    };

    private static final ArrayList<SimVisionTarget> simTargets = new ArrayList<>();

    private static State state = new State(false, Mode.single, 0.0);

    private static Field2d f;

    private static Supplier<Pose2d> r;

    private static ArrayList<Map.Entry<Double, Pose2d>> positions = new ArrayList<>();


    public enum Mode {
        single, stereo;
    }

    public Vision(Supplier<Pose2d> rr) {
        //camera = NetworkTableInstance.getDefault().getTable("photonvision");
        for (AprilTag a : tags) {
            aprilTags.put(a.ID, a);
            simTargets.add(
                new SimVisionTarget(a.pose, 1, 1, a.ID)
            );
        }

        for (Camera c : cameras) {
            for (SimVisionTarget s : simTargets) {
                c.addSim(s);
            }
        }

        //left.setCameraPair(right);
        //right.setCameraPair(left);
        frontPrimLeft.setCameraPair(frontFollowRight);
        //states.add(state);
        r = rr;
    }

    // Currently only gets pose from 1 april tag
    // TODO: Make multi april tag support with weighted averaging
    @Override
    public void periodic() {
        //results.clear();
        targets.clear();
        stereoTargets.clear();
        //processedTargets.clear();
        for (Camera c : cameras) {
            c.clear();
            if (!c.getLatestResult().hasTargets()) continue;
            for (PhotonTrackedTarget target : c.getLatestResult().getTargets()) {
                double xOffset = target.getYaw();
                double yOffset = target.getPitch() + VisionConstants.cameraOffset;
                Integer tagId = target.getFiducialId();
                //if (aprilTags.get(tagId) == null) continue;
                AprilTag currentTag = aprilTags.get(tagId);
                if (currentTag == null) continue;
                double height = currentTag.pose.getZ() - VisionConstants.cameraHeight;
                double hozDistance = (height/Math.tan(Units.degreesToRadians(yOffset)))/Math.cos(Units.degreesToRadians(xOffset));
                //double areaDist = target.getArea();
                //double distance = Math.sqrt(hozDistance*hozDistance + height*height);
                Target t = new Target(xOffset, yOffset, hozDistance, true, tagId, c, target.getArea());
                //processedTargets.add(new Target(xOffset, yOffset, hozDistance, hasTarget, tagId));
                //processedTargets.put(new Target(xOffset, yOffset, hozDistance, true, tagId), c);
                c.add(t);
                targets.add(t);
            }
            c.periodic(r.get());
        }
        state = new State(targets.size() > 0, Mode.single, 0.0);
        for (Camera c : cameras) {
            if (c.getCameraPair() == null) continue;
            for (Target t1 : c.getTargets()) {
                for (Target t2 : c.getCameraPair().getTargets()) {
                    if (t1.tagId == t2.tagId) {
                        stereoTargets.add(new StereoTarget(t1, t2));
                        state = new State(true, Mode.stereo, 0.0);
                    }
                }
            }
        }

        f.getObject("leftCam").setPose(new Pose2d(r.get().getTranslation(), r.get().getRotation()).plus(
            new Transform2d(frontPrimLeft.getTransform2d().getTranslation(), new Rotation2d())
        ));
        f.getObject("rightCam").setPose(new Pose2d(r.get().getTranslation(), r.get().getRotation()).plus(
            new Transform2d(frontFollowRight.getTransform2d().getTranslation(), new Rotation2d())
        ));

        cleanPositions();

        if (stereoTargets.size() > 0) {
            Pose2d currentEstimation = estimatePosition(r.get().getRotation());
            positions.add(Map.entry((double) System.currentTimeMillis(), currentEstimation));
        }

        // f.getObject("leftCamPOV").setTrajectory(
        //     PathPlanner.generatePath(new PathConstraints(1, 1),
        //         new PathPoint(new Translation2d(10, Rotation2d.fromDegrees(left.getPose().getRotation().getDegrees()+75/2)), new Rotation2d()),
        //         new PathPoint(left.getPose().getTranslation(), new Rotation2d()),
        //         new PathPoint(new Translation2d(10, Rotation2d.fromDegrees(left.getPose().getRotation().getDegrees()-75/2)), new Rotation2d())
        // ));
        /*targets.forEach((Target t) -> {
            System.out.println(t.tagId);
        });*/
        SmartDashboard.putNumber("Targets", targets.size());
    }

    private static void cleanPositions() {
        while (positions.size() > VisionConstants.POSE_ESTIMATIONS) {
            positions.remove(0);
        }
        for (int i = 0; i < positions.size(); i++) {
            if (System.currentTimeMillis() - positions.get(0).getKey() > VisionConstants.TIME_KEPT) {
                positions.remove(i);
                i--;
            }
        }
    }

    public static Pose2d getEstimatedPosition() {
        if (positions.size() == 0) return new Pose2d();
        double x = 0;
        double y = 0;
        for (Map.Entry<Double, Pose2d> e : positions) {
            x += e.getValue().getX();
            y += e.getValue().getY();
        }
        x/=(double)positions.size();
        y/=(double)positions.size();
        return new Pose2d(x, y, r.get().getRotation());
    }

    public static State getState() {
        return state;
    }

    public static Pose2d estimatePosition(Rotation2d gyro) {
        if (state.mode == Mode.single) return new Pose2d();
        // if (state.mode == Mode.single && state.hasTarget) {
        //     Target bestTarget = targets.get(0);
        //     for (Target t : targets) {
        //         if (t.area > bestTarget.area) {
        //             bestTarget = t;
        //         }
        //     }
        //     Pose2d relativePose = new Pose2d(new Translation2d(
        //         bestTarget.distance*Math.cos(Units.degreesToRadians(bestTarget.xOffset)-gyro.getRadians()),
        //         bestTarget.distance*Math.sin(Units.degreesToRadians(bestTarget.xOffset)-gyro.getRadians())
        //         ), gyro);
        //     Pose2d fieldPose = aprilTags.get(bestTarget.tagId).pose.toPose2d().plus(
        //         new Transform2d(
        //             new Translation2d(
        //                 relativePose.getX(), 
        //                 relativePose.getY()), 
        //             gyro
        //         ));
        //     //fieldPose.plus(bestTarget.parentCamera.getTransform2d());
        //     return fieldPose;        
        // }

        if (state.mode == Mode.stereo && state.hasTarget) {
            StereoTarget bestTarget = stereoTargets.get(0);
            AprilTag t = aprilTags.get(bestTarget.closestTarget.tagId);
            double theta1 = bestTarget.leftTarget.xOffset;
            double theta2 = bestTarget.rightTarget.xOffset;
            //double k = Math.signum(gyro.getRadians())*gyro.getRadians()%Math.PI;

            double phi = gyro.getRadians();
            SmartDashboard.putNumber("Theta1", theta1);
            SmartDashboard.putNumber("Theta2", theta2);
            SmartDashboard.putNumber("Phi", Units.radiansToDegrees(phi));

            double A = Units.degreesToRadians(90 - theta1) + phi;
            double B = Units.degreesToRadians(90 + theta2) - phi;
            // double C = Units.degreesToRadians(180 - A - B);

            double AB = bestTarget.leftCam.getPairDistance();
            
            double B_X = AB*Math.cos(phi);
            double B_Y = AB*Math.sin(phi);

            double X = (B_X*Math.tan(B)+B_Y) / (Math.tan(A)+Math.tan(B));
            double Y = Math.sin(A)*(1/Math.sin(A+B))*(B_X*Math.sin(B) + B_Y*Math.cos(B));

            double m1 = bestTarget.leftCam.getTransform2d().getY() / bestTarget.leftCam.getTransform2d().getX();
            double m2 = bestTarget.rightCam.getTransform2d().getY() / bestTarget.rightCam.getTransform2d().getX();
            double robotCenterX = (B_Y-B_X*Math.tan(Math.atan(m2)+phi)) / (Math.tan(Math.atan(m1)+phi)-Math.tan(Math.atan(m2)+phi));
            double robotCenterY = robotCenterX*Math.tan(Math.atan(m1)+phi);

            Pose2d robotCenterRelative = new Pose2d(robotCenterY, robotCenterX, new Rotation2d());

            // System.out.println("A: " + Units.radiansToDegrees(A) + "\t B: " + Units.radiansToDegrees(B) + "\t PHI: " + Units.radiansToDegrees(phi));
            // System.out.println("Theta1: " + theta1 + "\tTheta 2: " + theta2 + "\tAB: " + AB);

            double d = Math.sqrt(X*X+Y*Y);
            SmartDashboard.putNumber("Distance", d);

            Pose2d leftCameraPose = t.pose.toPose2d().plus(
                new Transform2d(
                    new Translation2d(Y, -X),
                    new Rotation2d()
                )                
            );

            SmartDashboard.putString("Left Cam", leftCameraPose.toString());

            Pose2d finalPose = leftCameraPose.plus(new Transform2d(
                new Translation2d(robotCenterY, robotCenterX),
                new Rotation2d()
            ));

            return new Pose2d(finalPose.getX(), finalPose.getY(), gyro);//new Pose2d(X, Y, gyro);
            /*return aprilTags.get(bestTarget.closestTarget.tagId).pose.toPose2d().plus(
                new Transform2d(relativePose.getTranslation(), new Rotation2d()));*/

        }
        
        return new Pose2d();
    }

    public static void setField(Field2d field) {
        f = field;
        for (AprilTag a : tags) {
            f.getObject(String.valueOf(a.ID)).setPose(a.pose.toPose2d());
        }
    }

    public static class Target {
        
        public final double xOffset;
        public final double yOffset;
        public final double distance;
        public final double area;

        public final int tagId;

        public final boolean hasTarget;

        public final Camera parentCamera;

        public Target(double xOffset, double yOffset, double distance, boolean hasTarget, int tagId, Camera parentCamera, double area) {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.distance = distance;
            this.hasTarget = hasTarget;
            this.tagId = tagId;
            this.parentCamera = parentCamera;
            this.area = area;
        }
    }

    public static class StereoTarget {

        public final Camera leftCam, rightCam;

        public final Target leftTarget, rightTarget, closestTarget;

        public StereoTarget(Target leftTarget, Target rightTarget) {
            this.leftTarget = leftTarget;
            this.rightTarget = rightTarget;
            this.leftCam = this.leftTarget.parentCamera;
            this.rightCam = this.rightTarget.parentCamera;
            if (leftTarget.distance > rightTarget.distance) {
                closestTarget = this.leftTarget;
            } else {
                closestTarget = this.rightTarget;
            }
        }

    }

    public static class State {
        
        public final boolean hasTarget;

        public final Mode mode;

        public final double distance;

        public State(boolean hasTarget, Mode mode, double d) {
            this.hasTarget = hasTarget;
            this.mode = mode;
            this.distance = d;
        }

    }
}