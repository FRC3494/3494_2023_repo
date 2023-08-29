package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A set of common helper methods that deal with the combination of Pose2ds
 * 
 * written by kennan, ask him if problems
 */
public class Pose2dHelpers {

    /**
     * Adds more than one Pose 2d together
     * 
     * @param args
     * @return sum of all components of pose 2ds
     */
    public static Pose2d add(Pose2d arg, Pose2d... args) {
        double x = arg.getX();
        double y = arg.getY();
        Rotation2d rotation = arg.getRotation();

        for (Pose2d pose2d : args) {
            x += pose2d.getX();
            y += pose2d.getY();

            rotation.plus(pose2d.getRotation());
        }

        return new Pose2d(x, y, rotation);
    }

    /**
     * Takes the arithmetic mean of one or more Pose2ds
     * 
     * @param args
     * @return
     */
    public static Pose2d mean(Pose2d arg, Pose2d... args) {
        double sumX = arg.getX();
        double sumY = arg.getY();
        Rotation2d sumRotation = arg.getRotation();

        for (Pose2d pose2d : args) {
            sumX += pose2d.getX();
            sumY += pose2d.getY();

            sumRotation.plus(pose2d.getRotation());
        }

        return new Pose2d(sumX / (args.length + 1), sumY / (args.length + 1), sumRotation.div(args.length));
    }

    public static Pose2d meanCorrect(Pose2d pose1, Pose2d pose2) {
        double avrgX = (pose1.getX() + pose2.getY()) / 2;
        double avrgY = (pose1.getY() + pose2.getY()) / 2;
        double avrgR = (pose1.getRotation().getRadians() + pose1.getRotation().getRadians()) / 2;
        return new Pose2d(avrgX, avrgY, new Rotation2d(avrgR));
    }
}
