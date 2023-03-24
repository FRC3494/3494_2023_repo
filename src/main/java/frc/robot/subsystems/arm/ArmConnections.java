package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.util.statemachine.StateConnection;

// WARNING:
// This file was autogenerated by FizzyApple12's StateGenerator
// !!!Edit at your own risk!!!
public class ArmConnections {
    public static List<StateConnection<ArmState>> connections = new ArrayList<>() {{
        add(new StateConnection<ArmState>(
                new ArmState(ShoulderState.Base2, ForearmState.GroundIntake, WristState.GroundIntake),
                new ArmState(ShoulderState.Base4, ForearmState.DoubleSubstation, WristState.DoubleSubstation)
            ));
add(new StateConnection<ArmState>(
                new ArmState(ShoulderState.Base4, ForearmState.DoubleSubstation, WristState.DoubleSubstation),
                new ArmState(ShoulderState.Base2, ForearmState.GroundIntake, WristState.GroundIntake)
            ));
add(new StateConnection<ArmState>(
                new ArmState(ShoulderState.Base2, ForearmState.GroundIntake, WristState.GroundIntake),
                new ArmState(ShoulderState.Base2, ForearmState.Intermediate, WristState.Store)
            ));
add(new StateConnection<ArmState>(
                new ArmState(ShoulderState.Base2, ForearmState.Intermediate, WristState.Store),
                new ArmState(ShoulderState.Base2, ForearmState.GroundIntake, WristState.GroundIntake)
            ));
add(new StateConnection<ArmState>(
                new ArmState(ShoulderState.Base2, ForearmState.Intermediate, WristState.Store),
                new ArmState(ShoulderState.Base4, ForearmState.DoubleSubstation, WristState.DoubleSubstation)
            ));
add(new StateConnection<ArmState>(
                new ArmState(ShoulderState.Base4, ForearmState.DoubleSubstation, WristState.DoubleSubstation),
                new ArmState(ShoulderState.Base2, ForearmState.Intermediate, WristState.Store)
            ));
    }};
}
