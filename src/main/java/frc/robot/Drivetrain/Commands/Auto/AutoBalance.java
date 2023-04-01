package frc.robot.Drivetrain.Commands.Auto;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        measurements.add(
            Map.entry((double) System.currentTimeMillis(), gyro.getPitchAtHeading(angle)));
        while (measurements.size() > 20) {
            measurements.remove(0);
        }
        double elapsedTime = measurements.get(measurements.size()-1).getKey() - measurements.get(0).getKey();
        Rotation2d deltaA = measurements.get(measurements.size()-1).getValue().minus(measurements.get(0).getValue());

        // double angularVelo = deltaA.getRadians()/elapsedTime;
        double angularAcc = deltaA.getRadians()/(Math.pow(elapsedTime, 2));

        if (angularAcc < 0.1) {
            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                Math.signum(deltaA.getDegrees()),
                0,
                0,
                gyro.getAngle() 
            ));
        } else {
            this.cancel();
        }
    }

    @Override
    public void end(boolean interr) {
        drivetrain.stop();
        drivetrain.frictionBrake();
    }
    
}
