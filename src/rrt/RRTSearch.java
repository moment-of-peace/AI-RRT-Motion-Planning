package rrt;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.HashSet;
import java.util.List;

public class RRTSearch {

    private static int clockwise;
    final private static double PI = Math.PI;
    public static void main(String[] args) throws IOException {
        // TODO Auto-generated method stub
        String srcFile = args[0];
        String outFile = args[1];
        
        Test tester = new Test(srcFile);
        int asvCount = tester.ps.getASVCount();
        int dimensions = asvCount * 2;
        
        // HashSets used to store found configurations in cspace from init and goal sides
        HashSet<Config> fromInit = new HashSet<Config>();
        HashSet<Config> fromGoal = new HashSet<Config>();
        
        // get initial and goal coordinates in c space
        Config initConfig = toConfig(tester.ps.getInitialState(),tester);
        Config goalConfig = toConfig(tester.ps.getGoalState(),tester);
        clockwise = isClockWise(initConfig);
                
        // add initial and goal into hashsets
        fromInit.add(initConfig);
        fromGoal.add(goalConfig);
        
        double[] angleRange = getAngleRange(initConfig, goalConfig);
    }

    private static int isClockWise(Config config) {
        // TODO Auto-generated method stub
        double[] coords = config.coords;
        if (coords.length > 5) {
            double diff = getAngleDiff(coords[0],coords[1],coords[2],coords[3],coords[4],coords[5]);
            if ((diff > 0 && diff < PI) || (diff < 0 && diff > -PI)) {  // diff == 0 !
                return 1;
            } else {
                return -1;
            }
        }
        return 0;
    }

    private static double getAngleDiff(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
        double ax = p2x - p1x;
        double ay = p2y - p1y;
        double bx = p3x - p2x;
        double by = p3y - p2y;
        double angleA = Math.atan2(ay, ax);
        double angleB = Math.atan2(bx, by);
        double diff = angleA - angleB;
        return diff;
    }
    private static double[] getAngleRange(Config config1, Config config2) {
        // TODO Auto-generated method stub
        double[] coords1 = config1.coords;
        double[] coords2 = config2.coords;
        int j;
        if (coords1.length > 5) {
            double[] range = new double[coords1.length/2 - 2];
            for (int i = 0; i < range.length; i++) {
                j = 2*i;
                double diff1 = getAngleDiff(coords1[j],coords1[j+1],coords1[j+2],coords1[j+3],coords1[j+4],coords1[j+5]);
            }
        }
        return null;
    }

    private static Config toConfig(ASVConfig asv, Test tester) {
        // TODO Auto-generated method stub
        List<Point2D> positions = asv.getASVPositions();
        //length
        double [] pts = new double [asv.getASVCount() * 2];
        int j = 0;
        for (int i=0;i<positions.size();i++){
            Point2D p = positions.get(i);
            pts[2*j] = p.getX();
            pts[2*j+1] = p.getY();
        }
        return new Config(pts);
    }
}
