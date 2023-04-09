package frc.robot.Drivetrain.Commands.Auto;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Sensors.Gyro.Gyro;

public class AutoBalance extends CommandBase {

    private final DrivetrainSubsystem drivetrain;
    private final Gyro gyro;

    private final Rotation2d angleOffset = new Rotation2d(DriverStation.getAlliance() == Alliance.Blue ? 0 : Math.PI);

    private final ArrayList<Map.Entry<Double, Double>> rollingAng = new ArrayList<>();
    private final ArrayList<Map.Entry<Double, Double>> rollingAngVelo = new ArrayList<>();
    private final ArrayList<Map.Entry<Double, Double>> rollingAngAcc = new ArrayList<>();

    private final double ANGLE_TOLERANCE = 0.1;
    private final double ANGLE_VELOCITY_TOLERANCE = 0.5;
    private final double ANGLE_ACCELERATION_TOLERANCE = 0.5;
    private final double DRIVE_DELAY = 0.5;

    private final Timer t = new Timer();

    public AutoBalance(DrivetrainSubsystem d) {
        t.start();
        drivetrain = d;
        gyro = Gyro.getInstance();
        addRequirements(d);
    }

    @Override
    public void execute() {
        t.start();
        rollingAng.add(Map.entry((double) System.currentTimeMillis(), gyro.getPitchAtHeading(angleOffset).getRadians()));
        while (rollingAng.size() > 3) {rollingAng.remove(0);}
        double angle = average(rollingAng);

        rollingAngVelo.add(Map.entry((double) System.currentTimeMillis(), range(rollingAng)/getElapsedTime(rollingAng)));
        while (rollingAngVelo.size() > 10) {rollingAngVelo.remove(0);}
        double angularVelocity = average(rollingAngVelo);

        rollingAngAcc.add(Map.entry((double) System.currentTimeMillis(), range(rollingAngVelo)/getElapsedTime(rollingAngVelo)));
        while (rollingAngAcc.size() > 20) {rollingAngAcc.remove(0);}
        double angularAcceleration = average(rollingAngAcc);

        if (angle < -ANGLE_TOLERANCE) {
            if (angularVelocity < -ANGLE_VELOCITY_TOLERANCE) {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    driveFast(false);
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else {
                    driveFast(false);
                }
            } else if (angularVelocity > ANGLE_VELOCITY_TOLERANCE) {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else {
                    driveFast(true);
                }
            } else {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    driveFast(false);
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else {
                    driveFast(false);
                }
            }
        } else if (angle > ANGLE_TOLERANCE) {
            if (angularVelocity < -ANGLE_VELOCITY_TOLERANCE) {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    driveFast(false);
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else {
                    stopDriving();
                }
            } else if (angularVelocity > ANGLE_VELOCITY_TOLERANCE) {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    driveFast(true);
                } else {
                    driveFast(true);
                }
            } else {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    driveFast(true);
                } else {
                    driveFast(true);
                }
            }
        } else {
            if (angularVelocity < -ANGLE_VELOCITY_TOLERANCE) {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    driveSlow(false);
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else {
                    driveSlow(false);
                }
            } else if (angularVelocity > ANGLE_VELOCITY_TOLERANCE) {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    stopDriving();
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    driveSlow(true);
                } else {
                    driveSlow(true);
                }
            } else {
                if (angularAcceleration < -ANGLE_ACCELERATION_TOLERANCE) {
                    driveSlow(false);
                } else if (angularAcceleration > ANGLE_ACCELERATION_TOLERANCE) {
                    driveSlow(true);
                } else {
                    stopDriving();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interr) {
        drivetrain.stop();
        drivetrain.frictionBrake();
    }
    
    private double average(ArrayList<Map.Entry<Double, Double>> a) {
        double sum = 0;
        for (Map.Entry<Double, Double> e : a) {
            sum+=e.getValue();
        }
        return sum/(double)a.size();
    }

    private double range(ArrayList<Map.Entry<Double, Double>> a) {
        return a.get(a.size()-1).getValue()-a.get(0).getValue();
    }

    private double getElapsedTime(ArrayList<Map.Entry<Double, Double>> a) {
        return (a.get(a.size()-1).getKey()-a.get(0).getKey())/1000;
    }

    private void driveSlow(boolean forward) {
        if (t.get() < DRIVE_DELAY) return;
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            !forward ? 0.09 : -0.09, 
            0.0, 
            0.0, 
            gyro.getAngle().rotateBy(angleOffset)
        ));
        t.reset();
    }

    private void driveFast(boolean forward) {
        if (t.get() < DRIVE_DELAY) return;
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            !forward ? 0.45 : -0.45, 
            0.0, 
            0.0, 
            gyro.getAngle().rotateBy(angleOffset)
        ));
        t.reset();
    }

    private void stopDriving() {
        drivetrain.stop();
        // drivetrain.drive(new ChassisSpeeds(0, 0, 1));
        drivetrain.frictionBrake();
    }


}
