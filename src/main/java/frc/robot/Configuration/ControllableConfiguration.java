package frc.robot.Configuration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllableConfiguration extends SubsystemBase {
    
    private final HashMap<String, Object> configurations = new HashMap<>();

    private final ArrayList<SuppliedValueWidget<?>> entries = new ArrayList<>();

    private final String name;

    public ControllableConfiguration(String name, Map.Entry<String, Object>... values) {
        this.name = name;
        for (Map.Entry<String, Object> entry : values) {
            configurations.put(entry.getKey(), entry.getValue());
        }
        putOnTelemetry(configurations);
    }

    @Override
    public void periodic() {
        for (SuppliedValueWidget<?> e : entries) {
            System.out.println(e.toString());
        }
    }

    public void putOnTelemetry(HashMap<String, Object> toPut) {
        ShuffleboardTab tab = Shuffleboard.getTab(name + " configuration");
        toPut.forEach((String s, Object o) -> {
            configurations.put(s, o);
            if (o instanceof Boolean) {
                entries.add(tab.addBoolean(s, () -> ((Boolean) o).booleanValue()));
            } else if (o instanceof Integer) {
                entries.add(tab.addNumber(s, () -> ((Integer) o).doubleValue()));
            } else if (o instanceof Double) {
                entries.add(tab.addNumber(s, () -> ((Double) o).doubleValue()));
            } else if (o instanceof String) {
                entries.add(tab.addString(s, () -> ((String) o)));
            }
        });
    }

    // public Object getValue(String key) {

    // }

}