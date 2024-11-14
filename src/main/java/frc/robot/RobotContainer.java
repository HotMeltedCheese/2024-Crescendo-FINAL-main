package frc.robot;

import java.beans.FeatureDescriptor;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.State.aState;
import frc.robot.State.eState;
import frc.robot.State.fState;
import frc.robot.State.iState;
import frc.robot.State.sState;
import frc.robot.State.tState;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.PhotonVisionCmds.RotateMove;
import frc.robot.subsystems.*;
//import frc.robot.subsystems.SysIDTest;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final CommandXboxController driver2 = new CommandXboxController(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final Timer timer = new Timer();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final SysIDTest testingSwerve = new SysIDTest(s_Swerve);
    private final PhotonVision camera1 = new PhotonVision(null);
    private final TrapAmpSubsystem s_TrapAmpSubsystem = new TrapAmpSubsystem();
    private final feederSubsystem s_FeederSubsystem = new feederSubsystem();
    private final intakeSubsystem s_IntakeSubsystem = new intakeSubsystem();
   // private final SysIDTest testingSwerve = new SysIDTest(s_Swerve);

   //Commands
    private final RotateMove c_DriveToTag = new RotateMove(null,null,null,null,null);

    private final TrapAmpCommand c_trapOut = new TrapAmpCommand(s_TrapAmpSubsystem, tState.OUT, null);
    private final TrapAmpCommand c_trapIn = new TrapAmpCommand(s_TrapAmpSubsystem,  tState.IN, null);
    private final TrapAmpCommand c_scoreAmp = new TrapAmpCommand(s_TrapAmpSubsystem, null, eState.AIM_POS);
    private final TrapAmpCommand c_home = new TrapAmpCommand(s_TrapAmpSubsystem, null, eState.HOME);

    private final feederCommand c_feederToTrap = new feederCommand(s_FeederSubsystem, aState.TRAP_POS,null);
    private final feederCommand c_feederToHome = new feederCommand(s_FeederSubsystem, aState.HOME, null);
    private final feederCommand c_feedIn = new feederCommand(s_FeederSubsystem, null, fState.IN);
    private final feederCommand c_feedOut = new feederCommand(s_FeederSubsystem, null, fState.OUT);
    private final feederCommand c_feedStop = new feederCommand(s_FeederSubsystem, null, fState.STOP);
    private final feederCommand c_flyIn = new feederCommand(s_FeederSubsystem, null, null, sState.IN, 0.2);
    private final feederCommand c_flyOut = new feederCommand(s_FeederSubsystem, null, null, sState.OUT, 0.2);
    private final feederCommand c_flyOutFast = new feederCommand(s_FeederSubsystem, null, null, sState.OUT, 0.4);
    private final feederCommand c_flyStop = new feederCommand(s_FeederSubsystem, null, null, sState.STOP, 0);
    private final feederCommand c_shootFar = new feederCommand(s_FeederSubsystem, aState.AIM_FAR, null);
    private final feederCommand c_shootHome = new feederCommand(s_FeederSubsystem, aState.HOME, null);
    private final feederCommand c_shootNear = new feederCommand(s_FeederSubsystem, aState.AIM_NEAR, null);
    private final feederCommand c_toClimb = new feederCommand(s_FeederSubsystem, aState.CLIMB, null);
    private final feederCommand c_feedWithRamp = new feederCommand(s_FeederSubsystem, null, fState.IN, sState.IN, 0.8);

    private final intakeCommand c_intakeIn = new intakeCommand(s_IntakeSubsystem, iState.IN);
    private final intakeCommand c_intakeOut = new intakeCommand(s_IntakeSubsystem, iState.OUT);
    private final intakeCommand c_intakeStop = new intakeCommand(s_IntakeSubsystem, iState.STOP);

    private final ParallelCommandGroup c_feederToIntake = new ParallelCommandGroup(
            new feederCommand(s_FeederSubsystem, aState.INTAKE_POS, fState.IN),
            c_intakeIn
        );
    
    private final ParallelCommandGroup c_intakeWithFeed = new ParallelCommandGroup(
            new feederCommand(s_FeederSubsystem, null, null, sState.OUT, 0.8),
            new intakeCommand(s_IntakeSubsystem, iState.OUT)
        );

    private final ParallelCommandGroup reallyNeedStop = new ParallelCommandGroup(
            new TrapAmpCommand(s_TrapAmpSubsystem, null, eState.M_DOWN),
            new feederCommand(s_FeederSubsystem, aState.M_DOWN, null)
        );

    private final ParallelCommandGroup kindaNeedaStop = new ParallelCommandGroup(
            new TrapAmpCommand(s_TrapAmpSubsystem, null, eState.M_IDLE),
            new feederCommand(s_FeederSubsystem, aState.M_IDLE, null)
        );

    private final ParallelCommandGroup accumulateIn = new ParallelCommandGroup(
            c_feederToHome, c_intakeIn, c_flyIn
        );

    // enable testing mode
    private final boolean testConfiguration = false;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        if(testConfiguration) {
            configureTestBindings(); 
            return;
        }
        
        configureDriver2Buttons();
    }

    private void configureTestBindings() {
        driver2.a().onTrue(testingSwerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver2.b().onTrue(testingSwerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driver2.x().onTrue(testingSwerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driver2.y().onTrue(testingSwerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    }

    private void configureDriver2Buttons() {
        initFeederBindings();
    }

    private void initFeederBindings() {
        driver2.leftBumper().onTrue(c_intakeWithFeed);
        driver2.leftBumper().onFalse(c_flyStop);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
