package frc.robot.commands;

import org.opencv.video.TrackerMIL;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.State.eState;
import frc.robot.State.tState;
import frc.robot.subsystems.TrapAmpSubsystem;

public class TrapAmpCommand extends Command {
    private TrapAmpSubsystem s_TrapAmp;
    private tState trapState = null;
    private eState eState = null;

    public TrapAmpCommand(TrapAmpSubsystem trapAmp, tState trapState, 
        eState eState) {
        this.s_TrapAmp = trapAmp;
        this.trapState = trapState;
        this.eState = eState;
    }

    @Override
    public void initialize() {
        if(trapState != null) {
            s_TrapAmp.goTrapWheelState(trapState);
        }

        if(eState != null) {
            s_TrapAmp.goTrapArmState(eState);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
