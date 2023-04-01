package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.NavX;

public final class OI {
    static boolean pickupMenu = false;
    static boolean leftGridMenu = false;
    static boolean middleGridMenu = false;
    static boolean rightGridMenu = false;
    static boolean coneMode = false;
    private static EventLoop eventLoop = new EventLoop();

    private static XboxController primaryController = new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);

    private static double offset = 0;

    public static void zeroControls() {
        offset = -NavX.getYaw() - 90;
    }

    public static double getDriveOffset() {
        return offset;
    }

    public static void setRedOffset() {
        offset = -100;
    }

    public static void setBlueOffset() {
        offset = -270;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            }

            return (value + deadband) / (1.0 - deadband);
        }

        return 0.0;
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.0001);

        // Square the axis
        value = Math.copySign(Math.pow(value, 3), value);

        return value;
    }

    public static void update() {
        eventLoop.poll();
    }

    public static double teleopXVelocity() {
        double forward = primaryController.getLeftY();
        double left = primaryController.getLeftX();
        double dPadPower = ((primaryController.getPOV() == 180) ? Constants.OI.DPAD_SPEED : 0)
                + ((primaryController.getPOV() == 0) ? -Constants.OI.DPAD_SPEED : 0);

        double angle = (Math.atan2(forward, left) + Math.toRadians(offset)) % (2 * Math.PI);
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2)), Constants.OI.MAX_DRIVE_SPEED)
                + dPadPower;

        return modifyAxis(-velocity) * Math.cos(angle) * Constants.OI.MAX_DRIVE_SPEED * driveMultiplier();
    }

    public static double teleopYVelocity() {
        double forward = primaryController.getLeftY();
        double left = primaryController.getLeftX();
        double dPadPower = ((primaryController.getPOV() == 90) ? Constants.OI.DPAD_SPEED : 0)
                + ((primaryController.getPOV() == 270) ? -Constants.OI.DPAD_SPEED : 0);

        double angle = (Math.atan2(forward, left) + Math.toRadians(offset)) % (2 * Math.PI);
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2)), Constants.OI.MAX_DRIVE_SPEED)
                + dPadPower;

        return modifyAxis(velocity) * Math.sin(angle) * Constants.OI.MAX_DRIVE_SPEED * driveMultiplier();
    }

    public static double teleopTurnVelocity() {
        return modifyAxis(primaryController.getRightX()) * Constants.OI.MAX_TURN_SPEED * driveMultiplier();
    }

    public static double driveMultiplier() {
        return (primaryController.getRightTriggerAxis() >= 0.1) ? 0.1
                : ((primaryController.getLeftTriggerAxis() >= 0.1) ? 2 : 1);
    }

    public static double armDirectDrivePower() {
        return deadband(primaryController.getRightY() * 0.3, 0.05);
    }

    public static BooleanEvent resetHeadingEvent() {
        return primaryController.x(eventLoop);
    }

    public static BooleanEvent autoBalanceEvent() {
        return primaryController.y(eventLoop);
    }

    /*
     * public static BooleanEvent autoLineUpEvent() {
     * return primaryController.a(eventLoop);
     * }
     */
    public static BooleanEvent selectDrivePickupMenu() {
        return primaryController.a(eventLoop).and(() -> !pickupMenu);
    }

    public static BooleanEvent resetMenuPickup() {
        return primaryController.a(eventLoop).and(() -> pickupMenu);
    }

    public static BooleanEvent selectDriveSingleSub() {
        return primaryController.rightTrigger(eventLoop).and(() -> pickupMenu);
    }

    public static BooleanEvent selectDriveDoubleSubLeft() {
        return primaryController.rightBumper(eventLoop).and(() -> pickupMenu);
    }

    public static BooleanEvent selectDriveDoubleSubRight() {
        return primaryController.leftBumper(eventLoop).and(() -> pickupMenu);
    }

    public static BooleanEvent selectDriveLeftGridMenu() {
        return primaryController.leftTrigger(eventLoop).and(() -> !leftGridMenu);
    }

    public static BooleanEvent resetMenuLeftGrid() {
        return primaryController.a(eventLoop).and(() -> leftGridMenu);
    }

    public static BooleanEvent selectDriveLeftConeLeftGrid() {
        return primaryController.leftTrigger(eventLoop).and(() -> leftGridMenu);
    }

    public static BooleanEvent selectDriveMiddleCubeLeftGrid() {
        return primaryController.leftBumper(eventLoop).and(() -> leftGridMenu);
    }

    public static BooleanEvent selectDriveRightConeLeftGrid() {
        return primaryController.rightBumper(eventLoop).and(() -> leftGridMenu);
    }

    public static BooleanEvent selectDriveMiddleGridMenu() {
        return primaryController.leftBumper(eventLoop).and(() -> !middleGridMenu);
    }

    public static BooleanEvent resetMenuMiddleGrid() {
        return primaryController.a(eventLoop).and(() -> middleGridMenu);
    }

    public static BooleanEvent selectDriveLeftConeMiddleGrid() {
        return primaryController.leftTrigger(eventLoop).and(() -> middleGridMenu);
    }

    public static BooleanEvent selectDriveMiddleCubeMiddleGrid() {
        return primaryController.leftBumper(eventLoop).and(() -> middleGridMenu);
    }

    public static BooleanEvent selectDriveRightConeMiddleGrid() {
        return primaryController.rightBumper(eventLoop).and(() -> middleGridMenu);
    }

    public static BooleanEvent selectDriveRightGridMenu() {
        return primaryController.rightBumper(eventLoop).and(() -> !rightGridMenu);
    }

    public static BooleanEvent resetMenuRightGrid() {
        return primaryController.a(eventLoop).and(() -> rightGridMenu);
    }

    public static BooleanEvent selectDriveLeftConeRightGrid() {
        return primaryController.leftTrigger(eventLoop).and(() -> rightGridMenu);
    }

    public static BooleanEvent selectDriveMiddleCubeRightGrid() {
        return primaryController.leftBumper(eventLoop).and(() -> rightGridMenu);
    }

    public static BooleanEvent selectDriveRightConeRightGrid() {
        return primaryController.rightBumper(eventLoop).and(() -> rightGridMenu);
    }

    public static BooleanEvent driveTrainLock() {
        return primaryController.b(eventLoop);
    }

    public static BooleanEvent printOdometryEvent() {
        return rightButtonBoard.button(9, eventLoop);
    }

    private static Joystick leftButtonBoard = new Joystick(Constants.OI.SECONDARY_LEFT_CONTROLLER_PORT);
    private static Joystick rightButtonBoard = new Joystick(Constants.OI.SECONDARY_RIGHT_CONTROLLER_PORT);
    // private static Joystick rightButtonBoard = new
    // Joystick(Constants.OI.SECONDARY_RIGHT_CONTROLLER_PORT);

    public static BooleanEvent armUndo() {
        return rightButtonBoard.button(3, eventLoop);
        // .or(rightButtonBoard.button(2, eventLoop))
        // .or(rightButtonBoard.button(3, eventLoop))
        // .or(rightButtonBoard.button(4, eventLoop));
    }

    public static BooleanEvent armStore() {
        return leftButtonBoard.button(4, eventLoop);
    }

    public static BooleanEvent armGroundIntakeCone() {
        return leftButtonBoard.button(7, eventLoop).and(() -> coneMode);
    }

    public static BooleanEvent armGroundIntakeCube() {
        return leftButtonBoard.button(7, eventLoop).and(() -> !coneMode);
    }

    public static BooleanEvent armDoubleSubstationCone() {
        return leftButtonBoard.button(9, eventLoop).and(() -> coneMode);
    }

    public static BooleanEvent armDoubleSubstationCube() {
        return leftButtonBoard.button(9, eventLoop).and(() -> !coneMode);
    }

    public static BooleanEvent armSingleSubstation() {
        return leftButtonBoard.button(8, eventLoop);
    }

    public static BooleanEvent armBase4Cone2() {
        return leftButtonBoard.button(1, eventLoop).and(() -> coneMode);
    }

    public static BooleanEvent armBase4Cube2() {
        return leftButtonBoard.button(1, eventLoop).and(() -> !coneMode);
    }

    public static BooleanEvent armBase4Cone1() {
        return leftButtonBoard.button(2, eventLoop).and(() -> coneMode);
    }

    public static BooleanEvent armBase4Cube1() {
        return leftButtonBoard.button(2, eventLoop).and(() -> !coneMode);
    }

    public static BooleanEvent armBase2Cone1() {
        return leftButtonBoard.button(5, eventLoop).and(() -> coneMode);
    }

    public static BooleanEvent armBase2Cube1() {
        return leftButtonBoard.button(5, eventLoop).and(() -> !coneMode);
    }

    public static BooleanEvent armBase1Hybrid() {
        return leftButtonBoard.button(3, eventLoop);
    }

    public static BooleanEvent clawIntakeCone() {
        // return leftButtonBoard.button(21, eventLoop);
        return leftButtonBoard.axisLessThan(0, -0.1, eventLoop).and(() -> coneMode);
    }

    public static BooleanEvent clawIntakeCube() {
        // return leftButtonBoard.button(20, eventLoop);
        return leftButtonBoard.axisLessThan(0, -0.1, eventLoop).and(() -> !coneMode);
    }

    public static BooleanEvent clawOuttakeCone() {
        // return leftButtonBoard.button(21, eventLoop);
        return leftButtonBoard.axisGreaterThan(0, 0.1, eventLoop).and(() -> coneMode);
    }

    public static BooleanEvent clawOuttakeCube() {
        // return leftButtonBoard.button(20, eventLoop);
        return leftButtonBoard.axisGreaterThan(0, 0.1, eventLoop).and(() -> !coneMode);
    }

    public static BooleanEvent clawIdle() {
        return leftButtonBoard.axisGreaterThan(0, -0.1, eventLoop).and(leftButtonBoard.axisLessThan(0, 0.1, eventLoop));
    }

    public static double forearmFineAdjust() {
        double speed = leftButtonBoard.getRawAxis(1);

        if (Math.abs(speed) < 0.1)
            return 0;

        return speed * Constants.OI.FOREARM_FINE_ADJUST_SPEED;
    }

    public static double wristFineAdjust() {
        double speed = rightButtonBoard.getRawAxis(1);

        if (Math.abs(speed) < 0.1)
            return 0;

        return speed * Constants.OI.WRIST_FINE_ADJUST_SPEED;
    }

    public static BooleanEvent clawFullPower() {
        return rightButtonBoard.axisGreaterThan(0, 0.1, eventLoop);
    }

    public static BooleanEvent ledsIndicateCone() {
        // return leftButtonBoard.button(20, eventLoop);
        return rightButtonBoard.button(1, eventLoop);
    }

    public static BooleanEvent ledsIndicateCube() {
        // return leftButtonBoard.button(20, eventLoop);
        return rightButtonBoard.button(2, eventLoop);
    }

    public static BooleanEvent toZero() {
        return rightButtonBoard.button(5, eventLoop)
                .and(rightButtonBoard.button(6, eventLoop))
                .and(rightButtonBoard.button(7, eventLoop));
    }
}
