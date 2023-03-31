package frc.robot.Drivetrain.Commands.Auto;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.Util.Kalman1D;

public class AutoBalance extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final Pigeon2 gyro;

    private final Rotation2d offset = new Rotation2d(DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI);

    private final Kalman1D angleFilter;

    private final ArrayList<Double> measurements = new ArrayList<>();

    public AutoBalance(DrivetrainSubsystem d) {
        drivetrain = d;
        gyro = Gyro.getInstance().getGyro();
        angleFilter = new Kalman1D(Gyro.getInstance().getAngle().getRadians(), 1, 0.1, 0.1);

        addRequirements(d);
    }

    @Override
    public void execute() {
        while (measurements.size() > 20) {
            measurements.remove(0);
        }
        
        for (Double d : measurements) {

        }
    }
    
}
