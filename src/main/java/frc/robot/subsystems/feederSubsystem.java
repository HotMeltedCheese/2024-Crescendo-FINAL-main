package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import static edu.wpi.first.units.Units.Micro;

import java.lang.annotation.Target;

import org.ejml.data.ZMatrix;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.aState;
import frc.robot.State.fState;
import frc.robot.State.sState;
import pabeles.concurrency.IntOperatorTask.Max;
import frc.robot.Constants;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

import frc.lib.util.CANSparkFlexUtil;




public class feederSubsystem extends SubsystemBase {
    
    //FEEDER MOVEMENT
    public CANSparkFlex m_bottomShootMotor;
    public CANSparkFlex m_topShootMotor;
    public fState fstate;

    private double FeederSpinSpeed;


    //FLYWHEEL MOVEMENT
    public PWMSparkMax m_leftIndexMotor;
    public PWMSparkMax m_rightIndexMotor;

    private double FlywheelSpinSpeed;

    //ARM MOVEMENT
    public CANSparkFlex m_RightAimingMotor;
    public CANSparkFlex m_LeftAimingMotor;
    public sState sState; 
    public aState aState;

    public DutyCycleEncoder a_Encoder;

    private double armSpeed;

    private PIDController aPID;
    private ArmFeedforward aFeedforward;

    private double aPV; //curr position
    private double aSetPoint; //destination we want to go to

    //POSE PARAMETERS
    double MIN;
    double toHome;
    double toFloat;
    double toIntake;
    double toTrap;
    double toFar;//Arbitrary value based on distance, shoots
    double toNear;//Arbitrary value based on distance, shoots
    double MAX;
    double toClimb;
    double TARGET; //target angle

    double M_UP;
    double M_DOWN;

    boolean OVERRIDE = false;


    public feederSubsystem(){

        //FEEDER SPINNER
        m_bottomShootMotor = new CANSparkFlex(Constants.feederSubsystem.leftMotorID, MotorType.kBrushless); //Fixed, Had to Reconfigure Motor 21
        m_topShootMotor = new CANSparkFlex(Constants.feederSubsystem.rightMotorID, MotorType.kBrushless);
        m_bottomShootMotor.setIdleMode(IdleMode.kCoast);
        m_topShootMotor.setIdleMode(IdleMode.kCoast);

        //FLYWHEEL SPINNER
        m_leftIndexMotor = new PWMSparkMax(Constants.shooterSystem.LeftFlyWheelID);
        m_rightIndexMotor = new PWMSparkMax(Constants.shooterSystem.RightFlyWheelID);
        m_leftIndexMotor.setInverted(true);
        m_rightIndexMotor.setInverted(true);


        //ARM MOVEMENT
        m_RightAimingMotor = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.RightAimID, MotorType.kBrushless);
        m_LeftAimingMotor = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.LeftAimID, MotorType.kBrushless);

        m_RightAimingMotor.setIdleMode(IdleMode.kBrake);
        m_LeftAimingMotor.setIdleMode(IdleMode.kBrake);

        a_Encoder = new DutyCycleEncoder(frc.robot.Constants.feederSubsystem.feederEncoderID); //PWM Channel
        
        double ffP = 0.0125; //was 0.05, 0.01
        double ffI = 0;
        double ffD = 0;
        aPID = new PIDController(ffP, ffI, ffD);

        aFeedforward = new ArmFeedforward(0, 0.05, 0); //TODO: Tune Feeder Feedforward

        


        //ARM SETPOINTS
        MIN = 56; //20
        toHome = 56;
        toFloat = 60;
        toIntake = MIN+20; //TODO: calibrate Feeder ARM Setpoints
        toTrap = 0; 
        toFar = 70;
        toNear = 54;
        toClimb = 169.7;
        MAX = 169.7; //was 105.35
        //CANBUS USAGE CONSTRAINTS
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_LeftAimingMotor, CANSparkFlexUtil.Usage.kPositionOnly);
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_RightAimingMotor, CANSparkFlexUtil.Usage.kPositionOnly);

        // CANSparkMaxUtil.setCANSparkMaxBusUsage(m_leftFlyMotor, Usage.kVelocityOnly);
        // CANSparkMaxUtil.setCANSparkMaxBusUsage(m_rightFlyMotor, Usage.kVelocityOnly);


        //Wheels
        m_bottomShootMotor.setInverted(false);
        m_topShootMotor.setInverted(false);

        //Aim Motor
        m_leftIndexMotor.setInverted(false);

        //Arms
        m_LeftAimingMotor.setInverted(false);
        m_RightAimingMotor.setInverted(true);

        setASetPoint(toHome); //init position

        //Reset Wheels
        goAimWheelState(fState.STOP);
        goIndexWheelState(frc.robot.State.sState.STOP, 0);
        
        fstate = frc.robot.State.fState.STOP;
    }

    private double aPos() {
        return a_Encoder.getAbsolutePosition() * 360;
    }

     @Override
    public void periodic(){
        aPV = aPos();
        var isDisabled = false;
        double aOutput = -aPID.calculate(aPV, aSetPoint);

        //If desired setpoint is within MIN/MAX
         if(aSetPoint >= MIN && aSetPoint <= MAX){
            setAimMotors(aOutput);;
            isDisabled = false;
        }else{
             m_LeftAimingMotor.set(0);
             m_RightAimingMotor.set(0);
             isDisabled = true;
        }

        SmartDashboard.putNumber("Feeder Arm Pos", aPV); //Measured in Degrees
        SmartDashboard.putNumber("Feeder Encoder DIO#", a_Encoder.getSourceChannel());

        SmartDashboard.putNumber("Feeder Fly Speed", FeederSpinSpeed);
        SmartDashboard.putString("Flywheel State", sState.name());

        SmartDashboard.putNumber("A Output", aOutput);

        SmartDashboard.putNumber("A Setpoint", getASetPoint());
        SmartDashboard.putBoolean("Is Disabled", isDisabled);

        SmartDashboard.putNumber("Target Angle:", TARGET);
        SmartDashboard.putBoolean("Feeder isOverride", OVERRIDE);

    }


    //FLYWHEEL SPIN STATE
    public void goIndexWheelState(sState state, double speed){
        switch(state) {
            case OUT:
                setIndexMotorSpeed(speed);
                break;
            case IN:
                setIndexMotorSpeed(-speed);
                break;
            case STOP:
                setIndexMotorSpeed(0);
                break;
        }

        this.sState = state;
    }

    private void setIndexMotorSpeed(double motorSpeed) {
        m_leftIndexMotor.set(motorSpeed);
        m_rightIndexMotor.set(motorSpeed);
    }
    

    private void setFeederMotors(double bottom, double top) {
        m_bottomShootMotor.set(bottom);
        m_topShootMotor.set(top);
    }

    private void setAimMotors(double speed) {
        m_LeftAimingMotor.set(speed);
        m_rightIndexMotor.set(speed);
    }

    //AIM SPIN STATE
    public void goAimWheelState(fState state){ //shooter state
        switch (state) {
            case OUT:
                setFeederMotors(0.75, 0.75);
                break;
            case IN:
                setFeederMotors(-0.2, 0);
                break;
            case STOP:
                setFeederMotors(0, 0);
                break;
        }
        this.fstate = state;
    }


    //ARM SET SETPOINT
    public void setASetPoint(double setpoint){
        aSetPoint = setpoint;
    }

    //ARM SET SETPOINT
    public double getASetPoint(){
        return aSetPoint;
    }

    //ARM MOVEMENT STATE
     public void goFeederArmState(aState state){ 
        OVERRIDE = false;
        switch (state) {
            case INTAKE_POS:
                aSetPoint = toIntake;
                break;
            case TRAP_POS:
                aSetPoint = toTrap;
                break;
            case AIM_FAR:
                aSetPoint = toFar;
                break;
            case AIM_NEAR:
                aSetPoint = toNear;
                break;
            case HOME:
                aSetPoint = MIN;
                break;
            case CLIMB:
                aSetPoint = toClimb;
                break;
            case FLOAT:
                aSetPoint = toFloat;
                break;
            default:
                OVERRIDE = true;
        }
        this.aState = state;
    }

    public void stopWheels(){
        goAimWheelState(frc.robot.State.fState.STOP);
    }
   }
