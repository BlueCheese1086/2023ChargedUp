package BlueCheeseUtils;

import java.io.FileWriter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.json.simple.JSONObject;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;

public class BuildData {

    static DecimalFormat f = new DecimalFormat("#####.#####");

    static double elevatorStep = Double.valueOf(
            f.format(1.0 / 42.0 * (ElevatorConstants.SPOOL_DIAMETER * Math.PI / ElevatorConstants.GEARBOX_RATIO)));
    static double armStep = Double.valueOf(f.format(1.0 / 42.0 * (Math.PI * 2.0 / ArmConstants.GEARBOX_RATIO)));

    public static double[] forwardKin(double u, double a) {
        return new double[] {
                u * Math.sin(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.cos(a),
                u * Math.cos(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.sin(a)
        };
    }

    public static Map<String, Double[]> stages(double u, double a, double w) {
        return Map.ofEntries(
                Map.entry("x", new Double[] { u * Math.sin(ElevatorConstants.TOWER_ANGLE_OFFSET),
                        u * Math.sin(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.cos(a),
                        u * Math.sin(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.cos(a)
                                + IntakeConstants.INTAKE_LENGTH * Math.cos(w) }),
                Map.entry("y", new Double[] { u * Math.cos(ElevatorConstants.TOWER_ANGLE_OFFSET),
                        u * Math.cos(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.sin(a),
                        u * Math.cos(ElevatorConstants.TOWER_ANGLE_OFFSET) + ArmConstants.ARM_LENGTH * Math.sin(a)
                                + IntakeConstants.INTAKE_LENGTH * Math.sin(w)
                }));
    }

    public static boolean isValid(Point p) {
        if (p.x < 0.0 || p.y < 0)
            return false;
        if (p.u > ElevatorConstants.MAX_HEIGHT
                || Math.abs(p.a + ElevatorConstants.TOWER_ANGLE_OFFSET) > ArmConstants.RANGE / 2)
            return false;

        double armX = 1.0 / Math.tan(p.a)
                * (Units.inchesToMeters(7) - (p.u * Math.cos(ElevatorConstants.TOWER_ANGLE_OFFSET)))
                + p.u * Math.sin(ElevatorConstants.TOWER_ANGLE_OFFSET);
        if (armX < Units.inchesToMeters(20) && armX >= 0)
            return false;

        return true;
    }

    public static List<Point> getNeighbors(Point p) {
        List<Point> l = new ArrayList<>();
        for (int i1 = -1; i1 <= 1; i1++) {
            for (int i2 = -1; i2 <= 1; i2++) {
                l.add(new Point(p.x + 0, p.y + 0, 0.0, 0.0));
            }
        }
        return l;
    }

    @SuppressWarnings("unchecked")
    public static void main(String[] args) {

        // System.out.println(armStep);

        JSONObject data = new JSONObject();

        int x = 0;

        double elevator = 0.0;
        double start = System.currentTimeMillis();
        while (elevator < ElevatorConstants.MAX_HEIGHT) {
            double arm = -ArmConstants.RANGE / 2 - ElevatorConstants.TOWER_ANGLE_OFFSET;
            while (arm < ArmConstants.RANGE / 2 - ElevatorConstants.TOWER_ANGLE_OFFSET) {
                x += 1;
                // System.out.println(armStep);
                double[] xy = forwardKin(elevator, arm);
                Point p = new Point(xy[0], xy[1], elevator, arm);
                if (isValid(p)) {
                    data.put(p.toString(), Map.ofEntries(
                            Map.entry("neighbors", p.getNeighbors()),
                            Map.entry("u", p.u),
                            Map.entry("a", p.a),
                            Map.entry("value", -1)));
                }
                arm += armStep;
                // System.out.println(arm/ArmConstants.RANGE*100);
            }
            elevator += elevatorStep;
            System.out.println(elevator / ElevatorConstants.MAX_HEIGHT * 100);
        }

        try {
            FileWriter f = new FileWriter("./src/main/deploy/test.txt");
            f.write(data.toJSONString());
            f.close();
        } catch (Exception e) {
        }

        System.out.println(data.size());
        System.out.println((System.currentTimeMillis() - start) / 1000.0);
    }
    
    public static class Point {

        final double x;
        final double y;

        final double u;
        final double a;

        private final List<Point> neighbors = new ArrayList<>();

        public Point(double x, double y, double u, double a) {
            DecimalFormat df = new DecimalFormat("###.######");
            this.x = Double.valueOf(df.format(x));
            this.y = Double.valueOf(df.format(y));

            this.u = Double.valueOf(df.format(u));
            this.a = Double.valueOf(df.format(a));
        }

        public Point(double u, double a) {
            DecimalFormat df = new DecimalFormat("###.######");
            double[] fkin = forwardKin(u, a);
            this.x = Double.valueOf(df.format(fkin[0]));
            this.y = Double.valueOf(df.format(fkin[1]));

            this.u = Double.valueOf(df.format(u));
            this.a = Double.valueOf(df.format(a));
        }

        public List<Double> getEntry() {
            return List.of(x, y);
        }

        public String toString() {
            return JSONObject.toJSONString(Map.of("x", x, "y", y));
        }

        public List<Point> getNeighbors() {
            neighbors.clear();
            DecimalFormat df = new DecimalFormat("###.######");
            for (int i1 = -1; i1 <= 1; i1++) {
                for (int i2 = -1; i2 <= 1; i2++) {

                    Point p = new Point(Double.valueOf(df.format(u + elevatorStep * i1)), Double.valueOf(df.format(a + armStep * i2)));
                    if (isValid(p)) {
                        neighbors.add(p);
                    }
                }
            }
            return neighbors;
        }

        public double distance(Point x) {
            return Math.sqrt(Math.pow(x.x-this.x, 2)+Math.pow(x.y-this.y, 2));
        }

    }

}