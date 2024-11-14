package frc.robot.commands.PhotonVisionCmds;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PhotonVision;

public class RotateMove extends Command {
    private Swerve m_Swerve;
    private PhotonVision camera;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    
    private double strafeVal; 

    private double desiredAngle;
    private double desiredDistance;

    public RotateMove(Swerve m_Swerve, PhotonVision camera,
            DoubleSupplier translationSup, DoubleSupplier strafeSup, 
            DoubleSupplier rotationSup) {
       this.m_Swerve = m_Swerve;
       this.camera = camera;
       addRequirements(m_Swerve, camera);

       this.translationSup = translationSup;
       this.strafeSup = strafeSup;
       this.rotationSup = rotationSup;

       this.desiredAngle = 0;
       this.desiredDistance = 0.5;

       // fix later
       //addRequirements(m_Swerve, camera);
    }

    @Override
    public void execute() {
        strafeVal = strafeSup.getAsDouble();

        m_Swerve.drive(
            new Translation2d((desiredDistance - camera.getDistance()), strafeVal),
            (desiredAngle - camera.getYaw()),
            false,
            false
        );
    }

}
