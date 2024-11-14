package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.iState;

public class intakeCommand extends Command {
    private intakeSubsystem intake;
    private iState state;

    public intakeCommand(intakeSubsystem intake, iState desiredPosition) {
        this.intake = intake;
        this.state = desiredPosition;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.goIntakeWheelState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
