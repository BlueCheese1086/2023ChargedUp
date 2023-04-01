package frc.robot.Sensors.Gyro;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.GyroConstants;

public class Gyro {

    private static Gyro instance;

    private Pigeon2 pigeon;

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(GyroConstants.GYRO_ID);
        }
        return instance;
    }

    public Gyro(int id) {
        pigeon = new Pigeon2(id);
        pigeon.configFactoryDefault();
        // The gyro is offset by 180deg
        pigeon.setYaw(DriverStation.getAlliance() == Alliance.Red ? 0 : 180);
    }

    /**
     * Gets the pigeon angle
     * [-180 deg, 180 deg] 0 IS POINTING AWAY FROM BLUE ALLIANCE
     * @return Rotation2d of the gyro angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(((pigeon.getYaw()%360)+360)%360-180);
    }

    public Rotation2d getPitchAtHeading(Rotation2d heading) {

        double pitch = pigeon.getPitch();
        double roll = pigeon.getRoll();
        Rotation2d angleDelta = getAngle().minus(heading);

        double pitchAtHeading = pitch*angleDelta.getCos() + roll*angleDelta.getSin();

        return Rotation2d.fromDegrees(pitchAtHeading);
    }

    public void setAngle(double a) {
        pigeon.setYaw(a);
    }

    public Pigeon2 getGyro() {
        return pigeon;
    }
    
}