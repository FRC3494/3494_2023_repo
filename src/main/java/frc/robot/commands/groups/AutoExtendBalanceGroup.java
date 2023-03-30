package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.auto.AutoSetArm;
import frc.robot.commands.auto.AutoSetShoulder;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.shoulder.ShoulderState;

public class AutoExtendBalanceGroup {
    public static Command get(Drivetrain drivetrain, Shoulder shoulder) {
        // return new AutoDrive(drivetrain, -Constants.Commands.AutoBalance.FAST_POWER,
        // 0, 0, false)
        // .until(() -> Math.abs(NavX.getPitch()) >=
        // Constants.Commands.AutoBalance.PEAK_ANGLE)
        // .andThen(new AutoBalance(drivetrain));

        return new AutoDrive(drivetrain, -Constants.Commands.AutoExtendBalance.SLOW_POWER, 0, 0, false)
                .until(() -> Math.abs(NavX.getPitch()) >= Constants.Commands.AutoExtendBalance.TRIGGER_ANGLE)
                .andThen(
                        new ParallelDeadlineGroup(
                                new WaitCommand(2.75),
                                new AutoSetShoulder(shoulder, ShoulderState.Base4),
                                new AutoDrive(drivetrain, -Constants.Commands.AutoExtendBalance.SLOW_POWER, 0, 0,
                                        false)),
                        new AutoSetShoulder(shoulder, ShoulderState.Base2),
                        new AutoBalance(drivetrain));
    }
}
