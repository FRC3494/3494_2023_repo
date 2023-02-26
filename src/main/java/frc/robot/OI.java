package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class OI {
    private static EventLoop eventLoop = new EventLoop();

	private static XboxController primaryController = new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);

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
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value * value, value);

        return value;
    }
    
    public static void update() {
        eventLoop.poll();
    }

    public static double getTeleopXVelocity() {
        return modifyAxis(primaryController.getLeftY()) * Constants.OI.MAX_DRIVE_SPEED;
    }

    public static double getTeleopYVelocity() {
        return modifyAxis(primaryController.getLeftX()) * Constants.OI.MAX_DRIVE_SPEED;
    }

    public static double getTeleopTurnVelocity() {
        return modifyAxis(primaryController.getRightX()) * Constants.OI.MAX_TURN_SPEED;
    }

    public static double getArmDirectDrivePower() {
        return deadband(primaryController.getRightY() * 0.3, 0.05);
    }

    public static BooleanEvent getResetHeadingEvent() {
        return primaryController.x(eventLoop);
    }

    public static BooleanEvent getAutoBalanceEvent() {
        return primaryController.y(eventLoop);
    }

    public static BooleanEvent getAutoLineUpEvent() {
        return primaryController.a(eventLoop);
    }

    public static BooleanEvent getPrintOdometryEvent() {
        return primaryController.b(eventLoop);
    }


    private static Joystick leftButtonBoard = new Joystick(Constants.OI.SECONDARY_LEFT_CONTROLLER_PORT);
    private static Joystick rightButtonBoard = new Joystick(Constants.OI.SECONDARY_RIGHT_CONTROLLER_PORT);

    public static BooleanEvent armHybrid() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armHopperIntake() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armGroundIntake() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armDoubleSubstation() {
        return leftButtonBoard.button(0, eventLoop);
    }


    public static BooleanEvent armN2() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armN1B2() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armB1Base4() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armBase2N1() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armBase1B1() {
        return leftButtonBoard.button(0, eventLoop);
    }
    public static BooleanEvent armStore() {
        return leftButtonBoard.button(0, eventLoop);
    }

    public static BooleanEvent getClawToggleEvent() {
        return rightButtonBoard.button(0, eventLoop);
    }
}
