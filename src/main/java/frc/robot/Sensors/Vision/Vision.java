package frc.robot.Sensors.Vision;

import java.util.ArrayList;

import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.Field.FieldElements;

public class Vision extends SubsystemBase {

    // private final NetworkTable camera;

    private final Camera frontFollowRight = new Camera(
            "rightcam",
            new Transform3d(new Translation3d(
                    Units.inchesToMeters(14), Units.inchesToMeters(-11.9375), Units.inchesToMeters(2.125)),
                    new Rotation3d(0, 0, 0)));
    private final Camera frontPrimLeft = new Camera(
            "leftcam",
            new Transform3d(new Translation3d(
                    Units.inchesToMeters(14), Units.inchesToMeters(13.9375), Units.inchesToMeters(4.875)),
                    new Rotation3d(0, 0, 0)));

    private Camera[] cameras = new Camera[] {
            frontPrimLeft, frontFollowRight
    };

    // private final ArrayList<PhotonPipelineResult> results = new ArrayList<>();
    private final ArrayList<Target> targets = new ArrayList<>();
    // private final HashMap<Target, Camera> processedTargets = new HashMap<>();
    private final ArrayList<StereoTarget> stereoTargets = new ArrayList<>();

    protected Vision() {
        // camera = NetworkTableInstance.getDefault().getTable("photonvision");

        for (Camera c : cameras) {
            for (AprilTag t : FieldElements.tags) {
                c.addSim(
                    new SimVisionTarget(t.pose, 0.2, 0.2, t.ID)
                );
            }
        }

        // left.setCameraPair(right);
        // right.setCameraPair(left);
        frontPrimLeft.setCameraPair(frontFollowRight);
        // states.add(state);
        // r = rr;
    }

    // Currently only gets pose from 1 april tag
    // TODO: Make multi april tag support with weighted averaging
    @Override
    public void periodic() {
        targets.clear();
        stereoTargets.clear();
        for (Camera c : cameras) {
            if (c.getCameraPair() == null) continue;
            c.periodic();
            c.getCameraPair().periodic();
            targets.addAll(c.getTargets());
            targets.addAll(c.getCameraPair().getTargets());
            for (Target t1 : c.getTargets()) {
                for (Target t2 : c.getCameraPair().getTargets()) {
                    if (t1.tagId == t2.tagId) {
                        stereoTargets.add(new StereoTarget(t1, t2));
                    }
                }
            }
        }
        // f.getObject("leftCam").setPose(new Pose2d(r.get().getTranslation(), r.get().getRotation()).plus(
        //         new Transform2d(frontPrimLeft.getTransform3d().getTranslation().toTranslation2d(), new Rotation2d())));
        // f.getObject("rightCam").setPose(new Pose2d(r.get().getTranslation(), r.get().getRotation()).plus(
        //         new Transform2d(frontFollowRight.getTransform3d().getTranslation().toTranslation2d(),
        //                 new Rotation2d())));
    }

    public ArrayList<Target> getTargets() {
        return targets;
    }
    
    public ArrayList<StereoTarget> getStereoTargets() {
        return stereoTargets;
    }

    public Pose2d estimateStereoTargetPos(StereoTarget bestTarget, Rotation2d gyro) {
        AprilTag t = bestTarget.closestTarget.tag;
        double theta1 = -bestTarget.leftTarget.xOffset;
        double theta2 = -bestTarget.rightTarget.xOffset;

        double phi = gyro.getRadians();
        SmartDashboard.putNumber("Theta1", theta1);
        SmartDashboard.putNumber("Theta2", theta2);
        SmartDashboard.putNumber("Phi", Units.radiansToDegrees(phi));

        double A = Math.PI/2 - theta1 + phi;
        double B = Math.PI/2 + theta2 - phi;
        // double C = Units.degreesToRadians(180 - A - B);

        double AB = bestTarget.leftCam.getPairDistance();

        double B_X = AB * Math.cos(phi);
        double B_Y = AB * Math.sin(phi);

        double X = (B_X * Math.tan(B) + B_Y) / (Math.tan(A) + Math.tan(B));
        double Y = Math.sin(A) * (1 / Math.sin(A + B)) * (B_X * Math.sin(B) + B_Y * Math.cos(B));

        double m1 = bestTarget.leftCam.getTransform3d().getY() / bestTarget.leftCam.getTransform3d().getX();
        double m2 = bestTarget.rightCam.getTransform3d().getY() / bestTarget.rightCam.getTransform3d().getX();
        double robotCenterX = (B_Y - B_X * Math.tan(Math.atan(m2) + phi))
                / (Math.tan(Math.atan(m1) + phi) - Math.tan(Math.atan(m2) + phi));
        double robotCenterY = robotCenterX * Math.tan(Math.atan(m1) + phi);
        
        SmartDashboard.putNumber("m1", -m1);
        SmartDashboard.putNumber("m2", -m2);

        Pose2d leftCameraPose = t.pose.toPose2d().plus(new Transform2d(
            new Translation2d(
                Math.signum(Math.cos(phi))*Y, 
                Math.signum(Math.cos(phi))*-X
            ),
            new Rotation2d()//gyro.minus(t.pose.getRotation().toRotation2d())
        ));
        Pose2d finalPose = leftCameraPose.plus(new Transform2d(
            new Translation2d(
                Math.signum(Math.cos(phi))*robotCenterX,
                Math.signum(Math.cos(phi))*robotCenterY
            ), 
            gyro.minus(t.pose.getRotation().toRotation2d())
        ));

        // return new Pose2d(finalPose.getX(), finalPose.getY(), gyro);
        //return new Pose2d(leftCameraPose.getX(), leftCameraPose.getY(), gyro);
        return finalPose;
    }

    public Rotation2d estimateGyroPose() {
        for (Camera c : cameras) {
            if (c.getCameraPair() == null) continue;
            if (c.getTargets().size() < 2 || c.getCameraPair().getTargets().size() < 2) continue;
            Target c1t1 = c.getTargets().get(0);
            Target c1t2 = c.getTargets().get(1);
            AprilTag t1 = c1t1.tag;
            AprilTag t2 = c1t2.tag;
            double[] c1pos = new double[]{
                (t2.pose.getX()*Math.tan(c1t2.xOffset)-t1.pose.getX()*Math.tan(c1t1.xOffset)+t1.pose.getY()-t2.pose.getY())/(Math.tan(c1t2.xOffset)-Math.tan(c1t1.xOffset)),
                ((t2.pose.getX()*Math.tan(c1t2.xOffset)-t1.pose.getX()*Math.tan(c1t1.xOffset)+t1.pose.getY()-t2.pose.getY())/(Math.tan(c1t2.xOffset)-Math.tan(c1t1.xOffset)-t2.pose.getX()))*Math.tan(c1t2.xOffset)+t2.pose.getY()
            };

            Target c2t1 = c.getTargets().get(0);
            Target c2t2 = c.getTargets().get(1);
            t1 = c2t1.tag;
            t2 = c2t2.tag;
            double[] c2pos = new double[]{
                (t1.pose.getX()*Math.tan(c2t1.xOffset)-t2.pose.getX()*Math.tan(c2t2.xOffset)-t1.pose.getY()+t2.pose.getY())/(Math.tan(c2t1.xOffset)-Math.tan(c2t2.xOffset)),
                ((t1.pose.getX()*Math.tan(c2t1.xOffset)-t2.pose.getX()*Math.tan(c2t2.xOffset)-t1.pose.getY()+t2.pose.getY())/(Math.tan(c2t1.xOffset)-Math.tan(c2t2.xOffset))-t1.pose.getX())*Math.tan(c1t1.xOffset)+t1.pose.getY()
            };
            return new Rotation2d((c1pos[1]-c2pos[1])/(c1pos[0]-c2pos[0]));
        }
        return new Rotation2d();
    }

    public Pose2d estimateSingleTargetPose(Target t, Rotation2d gyro) {
        return new Pose2d();
    }

    public class StereoTarget {

        public final Camera leftCam, rightCam;

        public final Target leftTarget, rightTarget, closestTarget;

        public StereoTarget(Target leftTarget, Target rightTarget) {
            this.leftTarget = leftTarget;
            this.rightTarget = rightTarget;
            this.leftCam = this.leftTarget.parentCamera;
            this.rightCam = this.rightTarget.parentCamera;
            if (leftTarget.distance < rightTarget.distance) {
                closestTarget = this.leftTarget;
            } else {
                closestTarget = this.rightTarget;
            }
        }

    }

    public static class Target {

        public final double xOffset;
        public final double yOffset;
        public final double distance;
        public final double area;

        public final int tagId;

        public final AprilTag tag;

        public final Camera parentCamera;


        public Target(PhotonTrackedTarget t, Camera c) {
            xOffset = Units.degreesToRadians(t.getYaw());
            yOffset = Units.degreesToRadians(t.getPitch());
            area = t.getArea();

            tag = FieldElements.aprilTags.get(t.getFiducialId());
            tagId = tag.ID;

            distance = -1.0;

            parentCamera = c;
        }

    }
}