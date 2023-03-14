package frc.robot.subsystems.arm;

import frc.robot.subsystems.forearm.Forearm;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.util.statemachine.StateMachine;

public class Arm extends StateMachine<ArmState> {
    public Arm(Shoulder shoulder, Forearm forearm, Hopper hopper) {
        registerControllables(shoulder, forearm, hopper);
    }
}