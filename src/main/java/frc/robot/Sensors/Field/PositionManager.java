package frc.robot.Sensors.Field;

import java.time.LocalDate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.BoundingTangle;

public class PositionManager extends SubsystemBase {

    private static PositionManager instance;

    private Field2d field;
    private Pose2d position;
    
    private Location l;

    public static PositionManager getInstance() {
        if (instance == null) {
            instance = new PositionManager();
        }
        return instance;
    }

    private PositionManager() {
        field = new Field2d();
        position = new Pose2d();
        Shuffleboard.getTab("Field").add("Field", field);
        l = DriverStation.getAlliance() == Alliance.Blue ? Location.blueScoring : Location.redScoring;
    }

    @Override
    public void periodic() {



    }
    
    public Field2d getField() {
        return field;
    }

    public Rotation2d getRobotRotation() {
        return position.getRotation();
    }

    public Pose2d getRobotPose() {
        return position;
    }

    public void setRobotPose(Pose2d p) {
        position = p;
        field.getRobotObject().setPose(p);
    }

    public FieldObject2d getObject(String n) {
        return field.getObject(n);
    }

    public Pose2d poseOfClosestScoring() {
        Pose2d closest = new Pose2d();
        Pose2d currentPos = getRobotPose();
        for (Pose2d p : FieldElements.scoringStations.values()) {
            if (currentPos.getTranslation().getDistance(p.getTranslation()) < currentPos.getTranslation().getDistance(closest.getTranslation())) {
                closest = p;
            }
        }
        return closest;
    }

    public Location getLocation() {
        return l;
    }

    public enum Location {
        bluePickup(
            new BoundingTangle(new Translation2d(Units.inchesToMeters(388), Units.inchesToMeters(267.5)),
                                new Translation2d(Units.inchesToMeters(637), Units.inchesToMeters(313.8))),
            new BoundingTangle(new Translation2d(Units.inchesToMeters(520.6), Units.inchesToMeters(219.6)),
                                new Translation2d(Units.inchesToMeters(637.5), Units.inchesToMeters(267.5)))
        ),
        redPickup(
            new BoundingTangle(new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(267.5)),
                                new Translation2d(Units.inchesToMeters(263.9), Units.inchesToMeters(313.8))),
            new BoundingTangle(new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(219.6)),
                                new Translation2d(Units.inchesToMeters(131.9), Units.inchesToMeters(267.5)))
        ),
        blueScoring(
            new BoundingTangle(new Translation2d(Units.inchesToMeters(131.9), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(190), Units.inchesToMeters(157))),
            new BoundingTangle(new Translation2d(Units.inchesToMeters(55), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(131.9), Units.inchesToMeters(216.3)))
        ),
        redScoring(
            new BoundingTangle(new Translation2d(Units.inchesToMeters(457.7), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(520), Units.inchesToMeters(157))),
            new BoundingTangle(new Translation2d(Units.inchesToMeters(520), Units.inchesToMeters(0)),
                                new Translation2d(Units.inchesToMeters(594.6), Units.inchesToMeters(216.3)))
        ),
        field,
        blueCharger(
            new BoundingTangle(new Translation2d(Units.inchesToMeters(128.7), Units.inchesToMeters(60.2)),
                                new Translation2d(Units.inchesToMeters(176.6), Units.inchesToMeters(156.1)))
        ),
        redCharger(
            new BoundingTangle(new Translation2d(Units.inchesToMeters(473.9), Units.inchesToMeters(60.265)),
                                new Translation2d(Units.inchesToMeters(521.8), Units.inchesToMeters(156.265))
            )
        );

        private final BoundingTangle[] tangles;

        Location(BoundingTangle... tangles) {
            this.tangles = tangles;
        }

        public boolean isInside(Pose2d p) {
            for (BoundingTangle t : tangles) {
                if (t.isInside(p)) return true;
            }
            return false;
        }

    }



}