package frc.robot.Sensors;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.Sensors.Vision.Target;

public class Camera extends PhotonCamera {

    private final Translation2d trans;
    private Camera pair;

    private final ArrayList<Target> targets = new ArrayList<>();

    private final SimVisionSystem simCam;

    public Camera(String name) {
        super(name);
        trans = new Translation2d();
        this.pair = null;
        simCam = null;
    }

    /**
     * @param name Photon camera name
     * @param trans Camera position relative to the center
     */
    public Camera(String name, Translation2d trans) {
        super(name);
        this.trans = trans;
        this.pair = null;
        simCam = new SimVisionSystem(
            name, 
            55,
            new Transform3d(new Translation3d(this.trans.getX(), this.trans.getY(), VisionConstants.cameraHeight), new Rotation3d()),
            10,
            960,
            480,
            0);
    }

    /**
     * @param name Photon camera name
     * @param trans Camera position relative to the center
     * @param pair Stereo camera pair object
     */
    public Camera(String name, Translation2d trans, Camera pair) {
        super(name);
        this.trans = trans;
        this.pair = pair;

        simCam = new SimVisionSystem(
            name, 
            55,
            new Transform3d(new Translation3d(this.trans.getX(), this.trans.getY(), VisionConstants.cameraHeight), new Rotation3d()),
            10,
            960,
            480,
            0);
    }

    public void periodic(Pose2d p) {
        if (simCam != null) simCam.processFrame(p);
    }

    /* Setters */

    /**
     * Sets camera pair
     * MAKE SURE THE LEFT CAMERA IS THE PRIMARY CAMERA
     * DO NOT SET A PAIR FOR THE RIGHT CAMERA
     * @param c The camera object to pair with
     */
    public void setCameraPair(Camera c) {
        this.pair = c;
    }

    public void add(Target t) {
        this.targets.add(t);
    }

    public void addSim(SimVisionTarget t) {
        simCam.addSimVisionTarget(t);
    }

    public void clear() {
        this.targets.clear();
    }

    /* Getters */
    
    public Transform2d getTransform2d() {
        return new Transform2d(trans, new Rotation2d());
    }

    public Camera getCameraPair() {
        return pair;
    }

    public double getPairDistance() {
        return trans.getDistance(pair.trans);
    }

    public ArrayList<Target> getTargets() {
        return this.targets;
    }

}
