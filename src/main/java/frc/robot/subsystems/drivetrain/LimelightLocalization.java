package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Pose2dHelpers;

public class LimelightLocalization {
    private Pose2d limelightToFieldOffset = new Pose2d(8.27, 4.01, new Rotation2d());

    private Queue<Double> standardDeviationXLeft = new ArrayBlockingQueue<Double>(11);
    private Queue<Double> standardDeviationYLeft = new ArrayBlockingQueue<Double>(11);

    private Queue<Double> standardDeviationXRight = new ArrayBlockingQueue<Double>(11);
    private Queue<Double> standardDeviationYRight = new ArrayBlockingQueue<Double>(11);

    public LimelightLocalization() {
    }

    private Pose2d getLeftPose() {
        return Pose2dHelpers.add(LimelightHelpers.getBotPose2d("limelight-left"),
                limelightToFieldOffset);
    }

    private Pose2d getRightPose() {
        return Pose2dHelpers.add(LimelightHelpers.getBotPose2d("limelight-right"),
                limelightToFieldOffset);
    }

    public Optional<Pose2d> getCurrentPose() {
        Pose2d limelightBotPoseLeft = getLeftPose();
        Pose2d limelightBotPoseRight = getRightPose();

        boolean leftNeitherXNorYAt0 = limelightBotPoseLeft.getX() != 0 && limelightBotPoseLeft.getY() != 0;
        boolean leftAprilTagIsDetected = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tv")
                .getDouble(0) != 0;
        boolean leftLimelightIsStable = nextStandardDeviation(limelightBotPoseLeft.getX(),
                limelightBotPoseLeft.getY(), standardDeviationXLeft,
                standardDeviationYLeft) <= Constants.Subsystems.Drivetrain.MAX_STANDARD_DEVIATION_LIMELIGHT;

        boolean resetPoseWithLeftLimelight = leftLimelightIsStable
                && leftAprilTagIsDetected
                && leftNeitherXNorYAt0;

        boolean rightNeitherXNorYAt0 = limelightBotPoseRight.getX() != 0 && limelightBotPoseRight.getY() != 0;
        boolean rightAprilTagIsDetected = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tv")
                .getDouble(0) != 0;
        boolean rightLimelightIsStable = nextStandardDeviation(limelightBotPoseRight.getX(),
                limelightBotPoseRight.getY(), standardDeviationXRight,
                standardDeviationYRight) <= Constants.Subsystems.Drivetrain.MAX_STANDARD_DEVIATION_LIMELIGHT;

        boolean resetPoseWithRightLimelight = rightLimelightIsStable
                && rightAprilTagIsDetected
                && rightNeitherXNorYAt0;

        if (resetPoseWithLeftLimelight && resetPoseWithRightLimelight) {
            Pose2d averagedPoses = Pose2dHelpers.mean(limelightBotPoseLeft, limelightBotPoseRight);

            return Optional.of(averagedPoses);
        } else if (resetPoseWithLeftLimelight) {
            return Optional.of(limelightBotPoseLeft);
        } else if (resetPoseWithRightLimelight) {
            return Optional.of(limelightBotPoseRight);
        } else {
            return Optional.empty();
        }
    }

    private double nextStandardDeviation(double nextX, double nextY, Queue<Double> standardDeviationX,
            Queue<Double> standardDeviationY) {
        if (!standardDeviationX.offer(nextX)) {
            standardDeviationX.remove();
            standardDeviationX.add(nextX);
        }

        if (!standardDeviationY.offer(nextX)) {
            standardDeviationY.remove();
            standardDeviationY.add(nextX);
        }

        return Math.sqrt(
                Math.pow(standardDeviation(standardDeviationX), 2)
                        + Math.pow(standardDeviation(standardDeviationY), 2));
    }

    private double standardDeviation(Queue<Double> list) {
        double mean = list
                .stream()
                .mapToDouble(a -> a)
                .average().getAsDouble();

        double midSectionOfTheEquationThatMustBeIterated = list.stream().reduce(
                (double) 0,
                (prev, cur) -> prev + Math.pow(cur - mean, 2));

        return Math.sqrt((midSectionOfTheEquationThatMustBeIterated) / (list.size() - 1));
    }
}
