package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.ArmState;
import frc.robot.Util.ForearmState;

public class Arm extends SubsystemBase  {
    public ArmState currentArmState;
    public ArmState targetArmState;
    public ForearmState currentForearmState;
    public ForearmState targetForearmState;
    //2 pneumatic pistons
    //1 Motor
    //Function to go to a certin position, and angle
    private DoubleSolenoid basePiston1;
    private DoubleSolenoid basePiston2;
    private CANSparkMax forearmMotor;

    
    public Arm(){
        basePiston1 = new DoubleSolenoid(Constants.Subsystems.Arm.BASE_PCM, 
            PneumaticsModuleType.CTREPCM, Constants.Subsystems.Arm.PISTON1_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.PISTON1_SOLENOID_CHANNEL + 1);
        basePiston2 = new DoubleSolenoid(Constants.Subsystems.Arm.BASE_PCM, 
            PneumaticsModuleType.CTREPCM, Constants.Subsystems.Arm.PISTON2_SOLENOID_CHANNEL, 
            Constants.Subsystems.Arm.PISTON2_SOLENOID_CHANNEL + 1);
        forearmMotor = new CANSparkMax(Constants.Subsystems.Arm.ARM_MOTOR_CHANNEL, MotorType.kBrushless);
        currentArmState = ArmState.Base2;
        targetArmState = ArmState.Base2;
    }
    public void setArmState(ArmState newState){
        targetArmState = newState;
    }
    public void setForearmState(ForearmState newState){
        currentForearmState = newState;
    }
    @Override
    public void periodic(){
        //FIXE Me: Change current Position based on encoder Values right now you assume pnematics move instantly
        if(targetArmState == ArmState.Base1){
            basePiston1.set(Value.kReverse);
            basePiston2.set(Value.kReverse);
            currentArmState = ArmState.Base1;

        }
        else if(targetArmState == ArmState.Base2){
            basePiston1.set(Value.kForward);
            basePiston2.set(Value.kReverse);
            currentArmState = ArmState.Base2;
        }
        else if(targetArmState == ArmState.Base3){
            basePiston1.set(Value.kReverse);
            basePiston2.set(Value.kForward);
            currentArmState = ArmState.Base3;
        }
        else if(targetArmState == ArmState.Base4){
            basePiston1.set(Value.kForward);
            basePiston2.set(Value.kForward);
            currentArmState = ArmState.Base4;
        }
    }

}
