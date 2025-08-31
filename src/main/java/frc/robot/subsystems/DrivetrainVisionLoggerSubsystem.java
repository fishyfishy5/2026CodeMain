package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class DrivetrainVisionLoggerSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    // Example drivetrain values (replace with actual swerve modules)
    private double frontLeftSpeed = 0;
    private double frontRightSpeed = 0;
    private double backLeftSpeed = 0;
    private double backRightSpeed = 0;

    public DrivetrainVisionLoggerSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    @Override
    public void periodic() {
        // ----- PhotonVision Logging -----
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        //Logger.getInstance().recordOutput("PhotonVision/HasTarget", hasTargets);

        if (hasTargets) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            //Logger.getInstance().recordOutput("PhotonVision/TargetYaw", bestTarget.getYaw());
            //Logger.getInstance().recordOutput("PhotonVision/TargetPitch", bestTarget.getPitch());
            //Logger.getInstance().recordOutput("PhotonVision/TargetArea", bestTarget.getArea());
        }

        // ----- Drivetrain Logging -----
        //Logger.getInstance().recordOutput("Drivetrain/FrontLeftSpeed", frontLeftSpeed);
        //Logger.getInstance().recordOutput("Drivetrain/FrontRightSpeed", frontRightSpeed);
        //Logger.getInstance().recordOutput("Drivetrain/BackLeftSpeed", backLeftSpeed);
        //Logger.getInstance().recordOutput("Drivetrain/BackRightSpeed", backRightSpeed);
    }

    // Example setter for drivetrain speeds (call from your drive code)
    public void setModuleSpeeds(double fl, double fr, double bl, double br) {
        frontLeftSpeed = fl;
        frontRightSpeed = fr;
        backLeftSpeed = bl;
        backRightSpeed = br;
    }

    /** Optional helper to get target yaw for auto-alignment */
    public Optional<Double> getTargetYaw() {
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            return Optional.of(result.getBestTarget().getYaw());
        }
        return Optional.empty();
    }
}
