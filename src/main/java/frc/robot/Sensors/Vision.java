package frc.robot.Sensors;

import java.util.ArrayList;
import java.util.HashMap;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    
    //private final NetworkTable camera;
    
    private static final Camera frontFollowRight = new Camera(
        "leftcamera",
        new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(-14.5)));
    private static final Camera frontPrimLeft = new Camera(
        "rightcamera", 
        new Translation2d(Units.inchesToMeters(14), Units.inchesToMeters(13)));

    private final static Camera[] cameras = new Camera[]{
        frontPrimLeft, frontFollowRight
    };

    //private final ArrayList<PhotonPipelineResult> results = new ArrayList<>();
    private static final ArrayList<Target> targets = new ArrayList<>();
    //private final HashMap<Target, Camera> processedTargets = new HashMap<>();
    private static final ArrayList<StereoTarget> stereoTargets = new ArrayList<>();
    private static final HashMap<Integer, AprilTag> aprilTags = new HashMap<>();

    private static final AprilTag[] tags = new AprilTag[]{
        new AprilTag(/*1*/6, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d())),
        new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d())),
        new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d())),
        new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d())),
        new AprilTag(5, new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d())),
        new AprilTag(/*6*/1, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d())),
        new AprilTag(7, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d())),
        new AprilTag(8, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d())),
    };

    private static State state = new State(false, Mode.single, 0.0);
    //private static ArrayList<State> states = new ArrayList<>();

    /*private static final Suppress xSuppressor = new Suppress(5, () -> geState().xOffset, new Config(10));
    private static final Suppress ySuppressor = new Suppress(5, () -> geState().yOffset, new Config(10));*/

    public enum Mode {
        single, stereo;
    }

    public Vision() {
        //camera = NetworkTableInstance.getDefault().getTable("photonvision");
        for (AprilTag a : tags) {
            aprilTags.put(a.ID, a);
        }
        //left.setCameraPair(right);
        //right.setCameraPair(left);
        frontPrimLeft.setCameraPair(frontFollowRight);
        //states.add(state);
        Shuffleboard.getTab("Cameras").addDoubleArray("Left Cam", 
            () -> {
                PhotonTrackedTarget t = frontPrimLeft.getLatestResult().getBestTarget();
                if (t == null) return new double[3];
                return new double[]{
                    t.getYaw(),
                    t.getPitch(),
                    t.getFiducialId()
                };
            }
        );
        Shuffleboard.getTab("Cameras").addDoubleArray("Right Cam", 
            () -> {
                PhotonTrackedTarget t = frontFollowRight.getLatestResult().getBestTarget();
                if (t == null) return new double[3];
                return new double[]{
                    t.getYaw(),
                    t.getPitch(),
                    t.getFiducialId()
                };
            }
        );
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
                double xOffset = -target.getYaw();
                double yOffset = -target.getPitch() + VisionConstants.cameraOffset;
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
            c.periodic();
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
        /*targets.forEach((Target t) -> {
            System.out.println(t.tagId);
        });*/
    }

    public static State geState() {
        return state;
    }

    public static Pose2d estimatePosition(Rotation2d gyro) {
        if (state.mode == Mode.single && state.hasTarget) {
            Target bestTarget = targets.get(0);
            for (Target t : targets) {
                if (t.area > bestTarget.area) {
                    bestTarget = t;
                }
            }
            Pose2d relativePose = new Pose2d(new Translation2d(
                bestTarget.distance*Math.cos(Units.degreesToRadians(bestTarget.xOffset)-gyro.getRadians()),
                bestTarget.distance*Math.sin(Units.degreesToRadians(bestTarget.xOffset)-gyro.getRadians())
                ), gyro);
            Pose2d fieldPose = aprilTags.get(bestTarget.tagId).pose.toPose2d().plus(
                new Transform2d(
                    new Translation2d(
                        relativePose.getX(), 
                        relativePose.getY()), 
                    gyro
                ));
            //fieldPose.plus(bestTarget.parentCamera.getTransform2d());
            return fieldPose;        
        }

        if (state.mode == Mode.stereo && state.hasTarget) {
            StereoTarget bestTarget = stereoTargets.get(0);
            double theta1 = bestTarget.leftTarget.xOffset;
            double theta2 = bestTarget.rightTarget.xOffset;
            double k = gyro.getRadians();

            double A = 90 + theta1;
            double B = 90 - theta2;
            double C = 180 - A - B;

            double AB = bestTarget.leftCam.getPairDistance();
            double AC = AB*Math.sin(A);
            double CH = AC*Math.sin(180-(theta1+k));
            double AH = AB*Math.sin(90+theta2)/Math.sin(-theta1-theta2)*Math.cos(180-theta1-k);
            double DH = AH + AB*Math.cos(k);

            Pose2d relativePose = new Pose2d(AH, CH, new Rotation2d());

            return aprilTags.get(bestTarget.closestTarget.tagId).pose.toPose2d().plus(
                new Transform2d(relativePose.getTranslation(), new Rotation2d()));

        }
        return new Pose2d(0, 0, gyro);
    }

    public static double getAngle() {
        if (targets.size() == 0) return 0.0;
        Target best = targets.get(0);
        if (stereoTargets.size() == 0) {
            for (Camera c : cameras) {
                for (Target t : c.getTargets()) {
                    if (t.area > best.area) best = t;
                }
            }
            return best.xOffset;
        }
        return (stereoTargets.get(0).leftTarget.xOffset + stereoTargets.get(0).rightTarget.xOffset)/2;
    }

    /*public static double[] math(Rotation2d gyro, Mode mode) {
        if (mode == Mode.trig && state.hasTarget) {
            return new double[]{
                state.distance*Math.cos(Units.degreesToRadians(state.xOffset)-gyro.getRadians()),
                state.distance*Math.sin(Units.degreesToRadians(state.xOffset)-gyro.getRadians())
            };
        }
        return new double[2];
    }*/

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