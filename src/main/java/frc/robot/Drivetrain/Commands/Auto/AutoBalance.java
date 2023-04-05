package frc.robot.Drivetrain.Commands.Auto;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Sensors.Gyro.Gyro;

public class AutoBalance extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final Gyro gyro;

    private final Rotation2d angle = new Rotation2d(DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI);

    private final ArrayList<Map.Entry<Double, Rotation2d>> measurements = new ArrayList<>();

    public AutoBalance(DrivetrainSubsystem d) {
        drivetrain = d;
        gyro = Gyro.getInstance();
        addRequirements(d);
    }

    @Override
    public void execute() {
        Rotation2d pitchAtAngle = gyro.getPitchAtHeading(angle);
        measurements.add(
            Map.entry(
                (double) System.currentTimeMillis(), 
                Math.abs(pitchAtAngle.getDegrees()) > 0.1 ? pitchAtAngle : new Rotation2d()
        ));

        while (measurements.size() > 175) {
            measurements.remove(0);
        }

        double elapsedTime = (measurements.get(measurements.size()-1).getKey() - measurements.get(0).getKey())/1000.0;
        Rotation2d deltaA = measurements.get(measurements.size()-1).getValue().minus(measurements.get(0).getValue());
        Rotation2d currentAngle = measurements.get(measurements.size()-1).getValue();

        double angle = currentAngle.getRadians();
        double angularVelo = deltaA.getRadians()/elapsedTime;
        double angularAcc = deltaA.getRadians()/Math.pow(elapsedTime, 2);

        

    }

    @Override
    public void end(boolean interr) {
        drivetrain.stop();
        drivetrain.frictionBrake();
    }
    
}
