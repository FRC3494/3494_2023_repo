package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.NavX;

public final class OI {
    private static EventLoop eventLoop = new EventLoop();

    private static XboxController primaryController = new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);

    private static double offset = 0;

    public static void zeroControls() {
        offset = NavX.getYaw() - 90;
    }

    public static double getDriveOffset() {
        return offset;
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

        double angle = Math.atan2(forward, left) + Math.toRadians(offset);
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2)), Constants.OI.MAX_DRIVE_SPEED);

        return modifyAxis(-velocity * Math.cos(angle)) * Constants.OI.MAX_DRIVE_SPEED * driveMultiplier();
    }

    public static double teleopYVelocity() {
        double forward = primaryController.getLeftY();
        double left = primaryController.getLeftX();

        double angle = Math.atan2(forward, left) + Math.toRadians(offset);
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2)), Constants.OI.MAX_DRIVE_SPEED);

        return modifyAxis(velocity * Math.sin(angle)) * Constants.OI.MAX_DRIVE_SPEED * driveMultiplier();
    }

    public static double teleopTurnVelocity() {
        return modifyAxis(primaryController.getRightX()) * Constants.OI.MAX_TURN_SPEED * driveMultiplier();
    }

    public static double driveMultiplier() {
        return (primaryController.getRightTriggerAxis() >= 0.1) ? 0.1 : ((primaryController.getLeftTriggerAxis() >= 0.1) ? 2 : 1);
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

    public static BooleanEvent autoLineUpEvent() {
        return primaryController.a(eventLoop);
    }

    public static BooleanEvent driveTrainLock() {
        return primaryController.b(eventLoop);
    }

    public static BooleanEvent printOdometryEvent() {
        return rightButtonBoard.button(12, eventLoop);
    }

    private static Joystick leftButtonBoard = new Joystick(Constants.OI.SECONDARY_LEFT_CONTROLLER_PORT);
    private static Joystick rightButtonBoard = new Joystick(Constants.OI.SECONDARY_RIGHT_CONTROLLER_PORT);
    // private static Joystick rightButtonBoard = new
    // Joystick(Constants.OI.SECONDARY_RIGHT_CONTROLLER_PORT);

    public static BooleanEvent armCancelToggle() {
        return leftButtonBoard.button(7, eventLoop);
    }

    public static BooleanEvent armBase1Cube1() {
        return leftButtonBoard.button(1, eventLoop);
    }

    public static BooleanEvent armBase4Cube1() {
        return leftButtonBoard.button(2, eventLoop);
    }

    public static BooleanEvent armBase4Cube2() {
        return leftButtonBoard.button(3, eventLoop);
    }

    public static BooleanEvent armBase2Cone1() {
        return leftButtonBoard.button(5, eventLoop);
    }

    public static BooleanEvent armBase4Cone2() {
        return leftButtonBoard.button(6, eventLoop);
    }

    public static BooleanEvent armStore() {
        return leftButtonBoard.button(4, eventLoop);
    }

    public static BooleanEvent armHybrid() {
        return rightButtonBoard.button(8, eventLoop);
    }

    public static BooleanEvent armHopperGrab() {
        return leftButtonBoard.button(8, eventLoop);
    }

    public static BooleanEvent armGroundIntake() {
        return leftButtonBoard.button(9, eventLoop);
    }

    public static BooleanEvent armDoubleSubstation() {
        return leftButtonBoard.button(10, eventLoop);
    }

    public static BooleanEvent clawOpenEvent() {
        // return leftButtonBoard.button(18, eventLoop);
        return leftButtonBoard.axisGreaterThan(0, 0.1, eventLoop);
    }

    public static BooleanEvent clawCloseEvent() {
        // return leftButtonBoard.button(19, eventLoop);
        return leftButtonBoard.axisLessThan(0, -0.1, eventLoop);
    }

    public static BooleanEvent forearmFineAdjustPositiveEvent() {
        // return leftButtonBoard.button(21, eventLoop);
        return leftButtonBoard.axisGreaterThan(1, 0.1, eventLoop);
    }

    public static BooleanEvent forearmFineAdjustNegativeEvent() {
        // return leftButtonBoard.button(20, eventLoop);
        return leftButtonBoard.axisLessThan(1, -0.1, eventLoop);
    }

    public static BooleanEvent shoulderBase1() {
        // return leftButtonBoard.button(21, eventLoop);
        return leftButtonBoard.button(1, eventLoop);
    }

    public static BooleanEvent shoulderBase2() {
        // return leftButtonBoard.button(20, eventLoop);
        return leftButtonBoard.button(2, eventLoop);
    }

    public static BooleanEvent shoulderBase4() {
        // return leftButtonBoard.button(20, eventLoop);
        return leftButtonBoard.button(3, eventLoop);
    }

    public static BooleanEvent hopperExtend() {
        // return leftButtonBoard.button(20, eventLoop);
        return rightButtonBoard.button(1, eventLoop);
    }

    public static BooleanEvent hopperRetract() {
        // return leftButtonBoard.button(20, eventLoop);
        return rightButtonBoard.button(2, eventLoop);
    }

    public static BooleanEvent zeroArm() {
        // return leftButtonBoard.button(20, eventLoop);
        return rightButtonBoard.button(10, eventLoop);
    }

    public static BooleanEvent ledsIndicateCone() {
        // return leftButtonBoard.button(20, eventLoop);
        return rightButtonBoard.button(6, eventLoop);
    }

    public static BooleanEvent ledsIndicateCube() {
        // return leftButtonBoard.button(20, eventLoop);
        return rightButtonBoard.button(7, eventLoop);
    }
}
