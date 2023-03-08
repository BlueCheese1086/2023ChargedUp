package frc.robot.Drivetrain.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Sensors.Gyro.Gyro;

public class DefaultDrive extends CommandBase {
    

    private final DoubleSupplier x_trans;
    private final DoubleSupplier y_trans;
    private final DoubleSupplier rot;

    private final DrivetrainSubsystem drive;

    private final Gyro gyro = Gyro.getInstance();

    public DefaultDrive(DrivetrainSubsystem d, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        x_trans = x;
        y_trans = y;
        rot = r;
        drive = d;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
            y_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
            rot.getAsDouble()*DriveConstants.MAX_TURN_VELOCITY,
            gyro.getAngle().plus(new Rotation2d(DriverStation.getAlliance() == Alliance.Red ? 0.0 : Math.PI)));
            // if (speeds.omegaRadiansPerSecond == 0 && speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0) {
            //     drive.frictionBrake();
            // } else {
            //     drive.drive(speeds);
            // }
            drive.drive(speeds);
        
        // drive.drive(new ChassisSpeeds(
        //     y_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
        //     x_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
        //     rot.getAsDouble()*DriveConstants.MAX_TURN_VELOCITY
        // ));
    }

    @Override
    public void end(boolean interr) {
        drive.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }

}
