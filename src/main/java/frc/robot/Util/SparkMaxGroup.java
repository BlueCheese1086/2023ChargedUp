package frc.robot.Util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ExternalFollower;

public class SparkMaxGroup {

    private CANSparkMax leader;
    private CANSparkMax[] followers;

    private SparkMaxPIDController controller;
    private RelativeEncoder relEncoder;

    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kFF = 0.0;
    
    public SparkMaxGroup(CANSparkMax lead, CANSparkMax... followers) {

        this.leader = lead;
        this.followers = followers;

        this.controller = leader.getPIDController();
        this.relEncoder = leader.getEncoder();

        this.leader.follow(ExternalFollower.kFollowerDisabled, 0);

        for (CANSparkMax f : followers) {
            f.restoreFactoryDefaults();
            f.setIdleMode(leader.getIdleMode());
            f.getEncoder().setPositionConversionFactor(relEncoder.getPositionConversionFactor());
            f.getEncoder().setVelocityConversionFactor(relEncoder.getVelocityConversionFactor());
            f.enableVoltageCompensation(leader.getVoltageCompensationNominalVoltage());
            f.follow(leader);
        }

    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void setPID(double kP, double kI, double kD, double kFF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kFF = kFF;

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setFF(kFF);
    }

}
