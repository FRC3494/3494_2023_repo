package frc.robot;

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
    
	public static void configureButtonBindings() {

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

    public static BooleanEvent getResetHeadingEvent() {
        return primaryController.x(eventLoop);
    }

    public static BooleanEvent getAutoBalanceEvent() {
        return primaryController.y(eventLoop);
    }
}
