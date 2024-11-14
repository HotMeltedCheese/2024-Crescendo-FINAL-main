package frc.robot.subsystems;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

import java.util.function.Consumer;
import java.util.logging.Logger;
import java.util.ArrayList;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;


public class SysIDTest extends SubsystemBase {    
    private SysIdRoutine routine;  
    private Consumer<Measure<Voltage>> voltageConsumer;
    // private Consumer<SysIdRoutineLog> logConsumer;
    private Consumer<SysIdRoutineLog> mechaConsumer; 
    private Swerve m_Swerve; 

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));


    // gather all swerve modules together. 
    private ArrayList<TalonFX> driveMotors; 
        // new TalonFX(Constants.Swerve.Mod0.driveMotorID),
        // new TalonFX(Constants.Swerve.Mod1.driveMotorID),
        // new TalonFX(Constants.Swerve.Mod2.driveMotorID),
        // new TalonFX(Constants.Swerve.Mod3.driveMotorID)


    private ArrayList<CANcoder> encoders;
        // new CANcoder(Constants.Swerve.Mod0.canCoderID),
        // new CANcoder(Constants.Swerve.Mod1.canCoderID),
        // new CANcoder(Constants.Swerve.Mod2.canCoderID),
        // new CANcoder(Constants.Swerve.Mod3.canCoderID)
    

    private String[] motorNames = {"m0", "m1", "m2", "m3"};

    public SysIDTest(Swerve m_Swerve) { 
        this.m_Swerve = m_Swerve;

        driveMotors = m_Swerve.getDriveMotors();
        encoders = m_Swerve.getEncoders();

        // create a lambda function to set motor voltages 
        voltageConsumer = voltage -> {
            for(TalonFX motor : driveMotors){
                motor.setVoltage(voltage.baseUnitMagnitude());
            }
        };

        // init object that logges sys data 
        mechaConsumer = logger -> {
            for(int i = 0; i < driveMotors.size(); i++)
            {
                logger.motor(motorNames[i]).voltage(
                    m_appliedVoltage.mut_replace(
                        driveMotors.get(i).get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(encoders.get(i).getPosition().getValueAsDouble(), Meters))
                        .linearVelocity(m_velocity.mut_replace(encoders.get(i).getVelocity().getValueAsDouble() , MetersPerSecond));

                // double voltageAmount = driveMotors[i].getMotorVoltage().getValueAsDouble();
                // Measure<Voltage> checker = Units.Volts;

                // driveMotors[i].getMotorVoltage().getValueAsDouble();
                // checker = driveMotors[i].getMotorVoltage().getValueAsDouble()>;
                // logger.motor(motorNames[i]).voltage(q);
            }
        };

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(voltageConsumer, mechaConsumer, this.m_Swerve,"Main Swerve")
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }
      
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}