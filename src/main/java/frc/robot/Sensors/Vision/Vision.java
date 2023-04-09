package frc.robot.Sensors.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final PhotonCamera left, right;

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) instance = new Vision();
        return instance;
    }

    private Vision() {
        left = new PhotonCamera("leftcamera");
        right = new PhotonCamera("rightcamera");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("/Vision/RightCam", getRightCamTargetRotation());
        SmartDashboard.putNumber("/Vision/LeftCam", getLeftCamTargetRotation());
        SmartDashboard.putNumber("/Vision/Avg", getAverage());
    }

    public double getRightCamTargetRotation() {
        if (!right.getLatestResult().hasTargets()) return Double.NaN;
        PhotonTrackedTarget t = right.getLatestResult().getBestTarget();
        if (t == null) return Double.NaN;
        return t.getYaw();
    }

    public double getLeftCamTargetRotation() {
        if (!left.getLatestResult().hasTargets()) return Double.NaN;
        PhotonTrackedTarget t = left.getLatestResult().getBestTarget();
        if (t == null) return Double.NaN;
        return t.getYaw();
    }

    public double getAverage() {
        if (Double.isNaN(getRightCamTargetRotation()) && !Double.isNaN(getLeftCamTargetRotation())) return getLeftCamTargetRotation();
        if (!Double.isNaN(getRightCamTargetRotation()) && Double.isNaN(getLeftCamTargetRotation())) return getRightCamTargetRotation();
        if (Double.isNaN(getRightCamTargetRotation()) && Double.isNaN(getLeftCamTargetRotation())) return 0.0;
        return (getRightCamTargetRotation()+getLeftCamTargetRotation())/2.0;
    }

}