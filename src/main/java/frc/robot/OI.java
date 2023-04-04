package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.NavX;

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
        value = deadband(value, 0.0001);

        // Square the axis
        value = Math.copySign(Math.pow(value, 3), value);

        return value;
    }
    
	public static void configureButtonBindings() {

	}

    public static void update() {
        eventLoop.poll();
    }

    private static double offset = 0;

    public static void zeroControls() {
        offset = -NavX.getYaw() - 90;
    }

    public static double teleopXVelocity() {
        double driveSpeed = slowMode() ? Constants.OI.SLOW_DRIVE_SPEED : Constants.OI.DRIVE_SPEED;

        double forward = primaryController.getLeftY();
        double left = primaryController.getLeftX();
        double dPadPower = ((primaryController.getPOV() == 180) ? Constants.OI.DPAD_SPEED : 0)
                + ((primaryController.getPOV() == 0) ? -Constants.OI.DPAD_SPEED : 0);

        double angle = (Math.atan2(forward, left) + Math.toRadians(offset)) % (2 * Math.PI);
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2)), driveSpeed)
                + dPadPower;

        return modifyAxis(-velocity) * Math.cos(angle) * driveSpeed;
    }

    public static double teleopYVelocity() {
        double driveSpeed = slowMode() ? Constants.OI.SLOW_DRIVE_SPEED : Constants.OI.DRIVE_SPEED;

        double forward = primaryController.getLeftY();
        double left = primaryController.getLeftX();
        double dPadPower = ((primaryController.getPOV() == 90) ? Constants.OI.DPAD_SPEED : 0)
                + ((primaryController.getPOV() == 270) ? -Constants.OI.DPAD_SPEED : 0);

        double angle = (Math.atan2(forward, left) + Math.toRadians(offset)) % (2 * Math.PI);
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2)), driveSpeed)
                + dPadPower;

        return modifyAxis(velocity) * Math.sin(angle) * driveSpeed;
    }

    public static double teleopTurnVelocity() {
        double turnSpeed = slowMode() ? Constants.OI.SLOW_TURN_SPEED : Constants.OI.TURN_SPEED;

        return modifyAxis(primaryController.getRightX()) * turnSpeed;
    }

    public static boolean slowMode() {
        return (primaryController.getRightTriggerAxis() >= 0.1) || (primaryController.getLeftTriggerAxis() >= 0.1);
    }

    public static BooleanEvent getResetHeadingEvent() {
        return primaryController.x(eventLoop);
    }

    public static BooleanEvent getAutoBalanceEvent() {
        return primaryController.y(eventLoop);
    }
    public static BooleanEvent getAutoLineUpEvent() {
        return primaryController.b(eventLoop);
    }
    public static Boolean getHopperIntake(){
        return primaryController.getAButtonPressed();
    }
    
    public static double getIntakeLeftVelocity(){
        return primaryController.getLeftTriggerAxis() * (primaryController.getLeftBumper() ? -0.5 : 0.5);
    }

    public static double getIntakeRightVelocity(){
        return primaryController.getRightTriggerAxis() * (primaryController.getRightBumper() ? 0.5 : -0.5);
    }
}
