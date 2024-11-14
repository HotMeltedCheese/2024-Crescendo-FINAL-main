
package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import frc.robot.PhotonConfig;

public class PhotonVision extends SubsystemBase{
    PhotonCamera camera;
    PhotonPipelineResult lastestDetection;
    PhotonTrackedTarget target;

    Transform3d targetLocation; 
    PhotonConfig configuration;

    double targetRange;
    double targetYaw;
    boolean targetSeen;

    //test commit big balls

    public PhotonVision(PhotonConfig configuration) {
        this.configuration = configuration;
        camera = new PhotonCamera(configuration.cameraName);
    }

    @Override 
    public void periodic(){
        lastestDetection = camera.getLatestResult();
        if(lastestDetection != null) {
            // look through all targets and check their target detection
            for(PhotonTrackedTarget x: lastestDetection.getTargets()) {
                if(x.getFiducialId() == configuration.targetID) {
                    targetYaw = x.getYaw();
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters( 
                        configuration.cameraHeightOffGround, 
                        configuration.targetHeightOffGround, 
                        Units.degreesToRadians(configuration.cameraPitch), 
                        Units.degreesToRadians(x.getPitch()));
                        targetSeen = true;
                        break;
                }
                targetSeen = false;
            }
        }
        else {
            targetSeen = false;
        }
    }

    public double getYaw() {
        return targetYaw;
    }

    public double getDistance() {
        return targetRange;
    }

}
