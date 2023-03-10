package frc.robot.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import frc.robot.Constants.ElevatorConstants;

/*
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀⠀⠀⠠⣤⠀⠀⢀⡀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣀⣀⣀⣀⣁⣀⣀⣉⣁⣀⣀⣀⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠉⠉⢹⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠺⠿⢸⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣤⣤⣼⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿
 * ⣿⣿⣿⣿⣿⣿⣶⣶⣶⣶⣶⣶⣶⣶⣾⣷⣶⣶⣶⣶⣶⣶⣶⣶⣿⣿⣿⣿⣿⣿ 
 */

public class ElevatorSubsystem extends SubsystemBase implements SubChecker {
    
    private final CANSparkMax left;
    private final CANSparkMax right;

    private final RelativeEncoder leftEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController leftPID;


    private ElevatorState currentState = new ElevatorState(0, 0);

    public enum ElevatorPosition {
        Stowed,
        Low,
        Mid,
        High,
        PlayerStation;
    }

    public ElevatorSubsystem() {
        left = new CANSparkMax(ElevatorConstants.leftID, MotorType.kBrushless);
        right = new CANSparkMax(ElevatorConstants.rightID, MotorType.kBrushless);

        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        // right.setInverted(true);
        right.follow(left, true);

        leftEncoder = left.getEncoder();
        absoluteEncoder = left.getAbsoluteEncoder(Type.kDutyCycle);
        //Motor position to elevator height (meters)
        leftEncoder.setPositionConversionFactor(ElevatorConstants.SPOOL_DIAMETER*Math.PI/ElevatorConstants.GEARBOX_RATIO);
        //Motor velocity m/s
        leftEncoder.setVelocityConversionFactor(ElevatorConstants.SPOOL_DIAMETER*Math.PI/ElevatorConstants.GEARBOX_RATIO/60);
        leftEncoder.setPosition(ElevatorConstants.MINIMUM_STARTING_HEIGHT + absoluteEncoder.getPosition()*ElevatorConstants.SPOOL_DIAMETER*Math.PI);

        leftPID = left.getPIDController();
        leftPID.setP(ElevatorConstants.kP);
        leftPID.setI(ElevatorConstants.kI);
        leftPID.setD(ElevatorConstants.kD);
        leftPID.setFF(ElevatorConstants.kFF);

        // if (inStaringPos && bottomSwitch.get()) {
        //     leftEncoder.setPosition(-0.05);
        // }
    }

    @Override
    public void periodic() {
        currentState = new ElevatorState(leftEncoder.getPosition(), leftEncoder.getVelocity());
    }

    /**
     * Set the desired elevator height
     * @param height Height in meters
     */
    public void setDesiredHeight(double height) {
        if (Robot.isSimulation()) {
            leftEncoder.setPosition(height);
            return;
        }
        leftPID.setReference(height, ControlType.kPosition);
    }

    /**
     * Set the desired preset position
     * @param e The desired position
     */
    public void setDesiredPosition(ElevatorPosition e) {
        switch (e) {
            case Stowed:
                setDesiredHeight(0);
                break;
            case Low:
                setDesiredHeight(0);
                break;
            case Mid:
                setDesiredHeight(0);
                break;
            case High:
                setDesiredHeight(0);
                break;
            case PlayerStation:
                setDesiredHeight(0);
                break;
        }
    }

    /**
     * Gets the current state of the elevator
     * @return The current state of the elevator
     */
    public ElevatorState getCurrentState() {
        return this.currentState;
    }

    public Command check(boolean safe) {
        if (!safe) {
          return new InstantCommand();
        }
        return new InstantCommand(() -> {
          double time = System.currentTimeMillis();
          boolean working = false;
          setDesiredHeight(0.1);
          while (System.currentTimeMillis() - time < 2000) {}
          if (leftEncoder.getPosition() > 0.05) {
            working = true;
          }
          SystemsCheck.setSystemStatus(this, working);
        }, this);
      }


}
