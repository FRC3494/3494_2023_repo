package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class OI {
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

        return value * .5;
    }
    
	public static void configureButtonBindings() {
	}

    public static double getTeleopXVelocity() {
        return modifyAxis(primaryController.getLeftY());
    }

    public static double getTeleopYVelocity() {
        return modifyAxis(primaryController.getLeftX());
    }

    public static double getTeleopTurnVelocity() {
        return modifyAxis(primaryController.getRightX());
    }
}
