package BlueCheeseUtils;

import java.io.FileReader;
import java.util.List;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import BlueCheeseUtils.BuildData.Point;

public class TestPathfinding {

    public static void main(String[] args) {

        JSONObject j = new JSONObject();

        try {
            j = (JSONObject) new JSONParser().parse(new FileReader("./src/main/deploy/test.txt"));
        } catch (Exception e) {

        }

        astar(new Point(0, 0), new Point(1, 0), j);

    }

    // @SuppressWarnings("unchecked")
    public static Point getNearestPoint(Point in, JSONObject data) {
        Point closest = new Point(1, 1);
        for (var s : data.keySet()) {
            JSONObject o = new JSONObject();
            try {
                o = (JSONObject) new JSONParser().parse((String) s);
            } catch (Exception e) {}
            JSONObject currentObj = (JSONObject) data.get(s);
            Point current = new Point(
                (double) o.get("x"), 
                (double) o.get("y"), 
                (double) currentObj.get("u"),
                (double) currentObj.get("a")
            );
            if (in.distance(current) < in.distance(closest)) {
                closest = current;
            }
        }
        return closest;
    }

    public static List<Point> astar(Point start, Point goal, JSONObject data) {
    
        Point current = start;

        while (current != goal) {
            for (Point g : current.getNeighbors()) {
                System.out.println(g);
                System.out.println(getNearestPoint(g, data));
                System.out.println(getNearestPoint(g, data));
                double neighborValue = (double) ((JSONObject) data.get(g.toString())).get("value");
                double currentValue = (double) ((JSONObject) data.get(current.toString())).get("value");

                System.out.println(neighborValue);
            }
        }

        return List.of();
    }
    
}
