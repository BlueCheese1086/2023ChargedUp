package frc.robot.Elevator;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.SparkMaxUtils.SparkMax;
import frc.robot.SparkMaxUtils.SparkMaxGroup;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxPIDConfiguration;
import frc.robot.SparkMaxUtils.Configurations.SparkMaxRelativeConfiguration;

public class ElevatorSubsystem extends SubsystemBase implements SafeSubsystem {

    private final SparkMax leftMotor = new SparkMax("Elevator", new SparkMaxConfiguration(
        ElevatorConstants.motorID, 
        MotorType.kBrushless, 
        false,
        45, 
        IdleMode.kBrake
    ), new SparkMaxRelativeConfiguration(
        false, 
        ElevatorConstants.conversionFactor, 
        ElevatorConstants.conversionFactor / 60.0
    ), SparkMaxPIDConfiguration.getDefault());

    private final SparkMax rightMotor = new SparkMax("Elevator", new SparkMaxConfiguration(
        ElevatorConstants.motorID, 
        MotorType.kBrushless, 
        true,
        45, 
        IdleMode.kBrake
    ), new SparkMaxRelativeConfiguration(
        false, 
        ElevatorConstants.conversionFactor, 
        ElevatorConstants.conversionFactor / 60.0
    ), SparkMaxPIDConfiguration.getDefault());

    private final SparkMaxGroup group = new SparkMaxGroup(leftMotor, rightMotor);

    public ElevatorSubsystem() {

    }

    public void periodic() {
        
    }

    public boolean isSafe() {
        return leftMotor.isConnected() && rightMotor.isConnected();
    }
    

}
