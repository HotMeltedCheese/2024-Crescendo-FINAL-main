package frc.robot.subsystems;

import org.ejml.data.ZMatrix;

import com.ctre.phoenix6.spns.SpnValue;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.State.eState;
import frc.robot.State.tState;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class TrapAmpSubsystem extends SubsystemBase {
    public PWMSparkMax m_trapMotor;
    public CANSparkMax m_RightArmMotor; //swapped PWM to CAN
    public CANSparkMax m_LeftArmMotor; //swapped PWM to CAN
    
    public tState tState; //Spinner
    public eState eState; //Arm

    private double spinSpeed = 0;
    private double spinCurrentLimit;

    private DutyCycleEncoder t_Encoder;
    private SparkPIDController tPID_L;
    private SparkPIDController tPID_R;


    private ArmFeedforward tFeedforward;

    private double tPV; //curr position
    private double tSetPoint; //destination we want to go to
    private PIDController armPid;
    double ArmOutput;

    //POSE PARAMETERS
    double MIN;
    double toHome;
    double toTrap;
    double toAmp;
    double toAim; //Arbitrary value based on distance, shoots
    double M_UP;
    double M_DOWN;
    double MAX;

    double IDLE;

    boolean OVERRIDE = false;



    
    // private SparkPIDController setPID(CANSparkMax m_motor) {
    //     // SparkPIDController m_pidController = m_motor.getPIDController();

    //     m_pidController.setP(0.1);
    //     m_pidController.setI(1e-5);
    //     m_pidController.setD(0.1);

    //     m_pidController.setOutputRange(0.1, 0.1);

    //     return m_pidController;
    // }


    public TrapAmpSubsystem(){
        m_trapMotor = new PWMSparkMax(frc.robot.Constants.AmpSystem.trapScorerID);
        m_RightArmMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.RightAmArmID, MotorType.kBrushless);
        m_LeftArmMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.LeftAmpArmID, MotorType.kBrushless);        //m_trapMotor.setIdleMode(IdleMode.kBrake);
        m_RightArmMotor.setIdleMode(IdleMode.kCoast);
        m_LeftArmMotor.setIdleMode(IdleMode.kCoast);

        tState = frc.robot.State.tState.STOP;

        t_Encoder = new DutyCycleEncoder(frc.robot.Constants.AmpSystem.ampEncoderID); //PWM Channel
        
        double ffP = 0.008; //was 0.01 //TODO: Tune PID
        double ffI = 0.0;
        double ffD = 0.0;

        tFeedforward = new ArmFeedforward(0, 0.01, 0.01); //-0.15
        armPid = new PIDController(ffP, ffI, ffD);
        
        //ARM SETPOINTS
        MIN = 10;
        toHome = 174; //TODO: calibrate Trap ARM Setpoints
        toTrap = 0; 
        toAim = 23.1; 
        //toAmp = 23.1;
        M_UP = tPos() + 1; //increment by 1 
        M_DOWN = tPos() - 1;
        IDLE = tPos();
        MAX = 180; //178

        m_RightArmMotor.setInverted(false); //was true
        m_LeftArmMotor.setInverted(true); //was false

        goTrapWheelState(frc.robot.State.tState.STOP);
        armPid.setSetpoint(toHome);
    }

    private double tPos() {
        return t_Encoder.getAbsolutePosition() * 360;
    }

     @Override
    public void periodic(){
        m_trapMotor.set(spinSpeed);

        //ARM
        tPV = tPos();

        ArmOutput = armPid.calculate(tPV);
        if(OVERRIDE == true){
           switch (eState) {
            case M_DOWN:
                setMotorValues(-0.1);
                break;
            case M_UP:
                setMotorValues(0.1);
            default:
                setMotorValues(0.01);
                break;
           }
        } else if(tPV > MIN && tPV < MAX) {
            setMotorValues(ArmOutput);
        } else {
            setMotorValues(0);
        }

        SmartDashboard.putNumber("Trap Encoder DIO#", t_Encoder.getSourceChannel());
        SmartDashboard.putNumber("T Setpoint", tSetPoint);
        SmartDashboard.putNumber("T encoder", tPos());
        SmartDashboard.putNumber("motor value", ArmOutput);
        SmartDashboard.putBoolean("Trap isOverride", OVERRIDE);
    }


    //TRAP AMP Spinner
    public void goTrapWheelState(tState state){
        switch(state) {
            case IN:
                spinSpeed = 1;
                break;
            case OUT:
                spinSpeed = -1;
                break;
            case STOP:
                spinSpeed = 0;
                break;
        }
        this.tState = state;
    }

    //ARM GET SETPOINT
    public double getTSetPoint(){
        return tSetPoint;
    }

    //Arm
    public void goTrapArmState(eState state){
        OVERRIDE = false;
        switch(state) {
            case HOME: 
                tSetPoint = toHome;
                break;
            case TRAP_POS:
                tSetPoint = toTrap;
                break;
            case AIM_POS:
                tSetPoint = toAim;
                break;
            case M_UP: 
                OVERRIDE = true;
                break;
            case M_DOWN:
                OVERRIDE = true;
                break;
            case M_IDLE:
                OVERRIDE = true;
                break;
            case IDLE:
                break;
        }
        eState = state;
    }

    private void setMotorValues(double motorValue){
        m_LeftArmMotor.set(motorValue);
        m_RightArmMotor.set(motorValue);
    }

    public eState getEstate() {
        return eState;
    }

    public tState getTState() {
        return tState;
    }
}
