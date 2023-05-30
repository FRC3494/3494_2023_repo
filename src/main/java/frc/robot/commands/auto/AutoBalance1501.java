package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutoBalance1501 extends CommandBase {

    final private Drivetrain drivetrain;
    private double tol = 2;
    private double oldTilt;
    private boolean finished;

    public AutoBalance1501(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        finished = false;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        oldTilt = NavX.getPitch();
        finished = false;
        // System.out.println("Balance Begin");
    }

    @Override
    public void execute() {
        // meters / second
        // System.out.println("GOING old-2: " + (oldTilt-tol)+"
        // current:"+SWERVE.getTilt());
        if (oldTilt - tol < NavX.getPitch()) {
            // System.out.println("Actually going!");
            this.drivetrain.drive(-0.6, 0, 0.0, true);
            // this.drivetrain.drive(new Translation2d(.6, 0), 0, true, true);
            return;
        }
        // System.out.println("NO GO");
        finished = true;

    }

    @Override
    public void end(boolean inturrupted) {
        // System.out.println("Balance End");
        drivetrain.drive(0, 0, 0, false);
        // SWERVE.drive(new Translation2d(0,0), 0, true, false);
    }

    @Override
    public boolean isFinished() {

        return finished;
    }

}
