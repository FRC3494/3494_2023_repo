package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.forearm.ForearmState;
import frc.robot.subsystems.hopper.HopperState;
import frc.robot.subsystems.shoulder.ShoulderState;
import frc.robot.util.statemachine.StateConnection;

// WARNING:
// This file was autogenerated by FizzyApple12's StateGenerator
// !!!Edit at your own risk!!!
public class ArmConnections {
    public static List<StateConnection<ArmState>> connections = new ArrayList<>() {
        {
            /*
             * add(new StateConnection<ArmState>(
             * new ArmState(ShoulderState.Base1, ForearmState.Base1Cube1,
             * HopperState.Retracted),
             * new ArmState(ShoulderState.Base2, ForearmState.Base1Cube1,
             * HopperState.Retracted)
             * ));
             */
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base2, ForearmState.Store, HopperState.Retracted),
                    new ArmState(ShoulderState.Base2, ForearmState.Intermediate, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base2, ForearmState.Intermediate, HopperState.Retracted),
                    new ArmState(ShoulderState.Base2, ForearmState.Store, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base2, ForearmState.Intermediate, HopperState.Retracted),
                    new ArmState(ShoulderState.Base1, ForearmState.GroundIntake, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base1, ForearmState.GroundIntake, HopperState.Retracted),
                    new ArmState(ShoulderState.Base2, ForearmState.Intermediate, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base2, ForearmState.Intermediate, HopperState.Retracted),
                    new ArmState(ShoulderState.Base1, ForearmState.Base1Cube1, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base1, ForearmState.Base1Cube1, HopperState.Retracted),
                    new ArmState(ShoulderState.Base2, ForearmState.Intermediate, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base1, ForearmState.GroundIntake, HopperState.Retracted),
                    new ArmState(ShoulderState.Base1, ForearmState.Base1Cube1, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base1, ForearmState.Base1Cube1, HopperState.Retracted),
                    new ArmState(ShoulderState.Base1, ForearmState.GroundIntake, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base4, ForearmState.DoubleSubstation, HopperState.Extended),
                    new ArmState(ShoulderState.Base2, ForearmState.Store, HopperState.Retracted)));
            add(new StateConnection<ArmState>(
                    new ArmState(ShoulderState.Base2, ForearmState.Store, HopperState.Retracted),
                    new ArmState(ShoulderState.Base4, ForearmState.DoubleSubstation, HopperState.Extended)));
        }
    };
}
