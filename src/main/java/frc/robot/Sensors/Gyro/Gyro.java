package frc.robot.Sensors.Gyro;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GyroConstants;

public class Gyro extends SubsystemBase {

    private static Gyro instance;

    private Pigeon2 pigeon;

    private double rollOffset, pitchOffset;

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(GyroConstants.GYRO_ID);
        }
        return instance;
    }

    public Gyro(int id) {
        pigeon = new Pigeon2(id);
    }

    public void initGyro() {
        pigeon.configFactoryDefault();
        // The gyro is offset by 180deg
        pigeon.setYaw(DriverStation.getAlliance() == Alliance.Red ? 0 : 180);
        rollOffset = pigeon.getRoll();
        pitchOffset = pigeon.getPitch();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Gyro/Pitch", getPitch());
        SmartDashboard.putNumber("Gyro/Roll", getRoll());
        SmartDashboard.putNumber("Gyro/Yaw", pigeon.getYaw());
        SmartDashboard.putNumber("Gyro/RotationAtHeading0", getPitchAtHeading(new Rotation2d()).getDegrees());
    }

    /**
     * Gets the pigeon angle
     * [-180 deg, 180 deg] 0 IS POINTING AWAY FROM BLUE ALLIANCE
     * @return Rotation2d of the gyro angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(((pigeon.getYaw()%360)+360)%360-180);
    }

    public double getYaw() {
        return pigeon.getYaw();
    }

    public double getPitch() {
        return pigeon.getPitch() - pitchOffset;
    }

    public double getRoll() {
        return pigeon.getRoll() - rollOffset;
    }

    public Rotation2d getPitchAtHeading(Rotation2d heading) {

        // These two are swapped I hate it too
        double pitch = pigeon.getRoll();
        double roll = pigeon.getPitch();
        Rotation2d angleDelta = getAngle().minus(heading);
        // System.out.println(angleDelta.getDegrees());

        double pitchAtHeading = pitch*angleDelta.getCos()*angleDelta.getCos() + roll*angleDelta.getSin()*angleDelta.getSin();

        return Rotation2d.fromDegrees(pitchAtHeading);
    }

    public void setAngle(double a) {
        pigeon.setYaw(a);
    }

    public Pigeon2 getGyro() {
        return pigeon;
    }
    
}