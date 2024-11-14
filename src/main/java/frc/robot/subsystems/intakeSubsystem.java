package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.State.iState;

import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;;


public class intakeSubsystem extends SubsystemBase {
    public CANSparkFlex m_wheelMotor;
    public CANSparkFlex m_floorMotor;
    public PWMSparkMax m_frontMotor;
    public iState Istate;

    public intakeSubsystem(){
        m_wheelMotor = new CANSparkFlex(Constants.IntakeSystem.IntakeWheel.wheelMotorID, MotorType.kBrushless);
        m_wheelMotor.setIdleMode(IdleMode.kBrake);

        m_floorMotor = new CANSparkFlex(Constants.IntakeSystem.IntakeWheel.floorMotorID, MotorType.kBrushless); //tennisgrip

        m_frontMotor = new PWMSparkMax(Constants.IntakeSystem.IntakeWheel.frontMotorID); //tennisgrip

        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_wheelMotor, Usage.kVelocityOnly);
        Istate = frc.robot.State.iState.STOP;

        m_wheelMotor.setInverted(false);
        m_frontMotor.setInverted(false); //was true
        m_floorMotor.setInverted(false); //was true

        goIntakeWheelState(iState.STOP);
    }

    //INTAKE SPIN
    public void goIntakeWheelState(iState state){
        switch (state) {
            case IN:
                setMotorValues(-0.55, -0.75, 0.75);
                break;
            case OUT:
                setMotorValues(0.55, -0.5, 0.5);
                break;
            case STOP:
                setMotorValues(0, 0, 0);
                break;
            case AMP_IN:
                setMotorValues(0.25, 0.75, 0.75);
        }
        Istate = state;
    }

    private void setMotorValues(double wheelVal, double frontVal, double floorVal) {
        m_floorMotor.set(floorVal);
        m_frontMotor.set(frontVal);
        m_wheelMotor.set(wheelVal);
    }

}
