package frc.robot.SparkMaxUtils;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxManager extends SubsystemBase {
    
    private static ArrayList<Map.Entry<GenericEntry, SparkMax>> maxes = new ArrayList<>();

    public static void addMax(SparkMax m) {
        GenericEntry overrideEntry = Shuffleboard.getTab("Motor Overrides").add(m.getName(), false).getEntry();
        maxes.add(Map.entry(overrideEntry, m));
    }

    public void periodic() {
        for (Map.Entry<GenericEntry, SparkMax> entry : maxes) {
            entry.getValue().setDisabled(entry.getKey().getBoolean(false));
        }
    }

}
