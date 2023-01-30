package frc.robot.Sensors;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Sensors.Vision.Target;

public class Camera extends PhotonCamera {

    private final Translation2d trans;
    private Camera pair;

    private final ArrayList<Target> targets = new ArrayList<>();

    public Camera(String name) {
        super(name);
        trans = new Translation2d();
        this.pair = null;
    }

    /**
     * @param name Photon camera name
     * @param trans Camera position relative to the center
     */
    public Camera(String name, Translation2d trans) {
        super(name);
        this.trans = trans;
        this.pair = null;
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
    }

    public void periodic() {
        if (this.targets.size() > 0) {
            super.setLED(VisionLEDMode.kOn);
        } else {
            super.setLED(VisionLEDMode.kOff);
        }
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
