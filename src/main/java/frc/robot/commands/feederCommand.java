package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.aState;
import frc.robot.State.fState;
import frc.robot.State.sState;

public class feederCommand extends Command {
    private feederSubsystem feeder;
    private fState feederState;
    private aState aimState;
    private double speed;
    private sState indexWheel;

    public feederCommand(feederSubsystem feeder, aState aimPosition, fState feederState, 
        sState indexWheel, double speed) {
        this.feeder = feeder;
        this.feederState = feederState;
        this.aimState = aimPosition;
        this.indexWheel = indexWheel;
        this.speed = speed;
    }

    public feederCommand(feederSubsystem feeder, aState aimPosition, fState feederState) {
        this.feeder = feeder;
        this.feederState = feederState;
        this.aimState = aimPosition;
    }

    @Override
    public void initialize() {
        if(aimState != null) {
            feeder.goFeederArmState(aimState);
        }
        
        if(feederState != null) {
            feeder.goAimWheelState(feederState);
        }
        
        if(indexWheel != null) {
            feeder.goIndexWheelState(indexWheel, speed);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
 
}
