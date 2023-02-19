package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase  {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    public Intake(){
        leftMotor = new CANSparkMax(Constants.Subsystems.Intake.LEFT_MOTOR_CHANNEL, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.Subsystems.Intake.RIGHT_MOTOR_CHANNEL, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(Constants.Subsystems.Intake.AMP_LIMIT);
        rightMotor.setSmartCurrentLimit(Constants.Subsystems.Intake.AMP_LIMIT);
    }

    public void setRightMotorSpeed(double percent){
        rightMotor.set(percent);
    }

    public void setLeftMotorSpeed(double percent){
        leftMotor.set(percent);
    }

    public void setSpeed(double percent){
        rightMotor.set(percent);
        leftMotor.set(-1.0 * percent);
    }
}