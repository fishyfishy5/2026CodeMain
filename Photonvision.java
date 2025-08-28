package frc.robot;
  
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
//import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
//import org.photonvision.common.dataflow.structures.Packet;
//import org.photonvision.estimation.TargetModel;
//import org.photonvision.simulation.VisionSystemSim;
//import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;  // <-- This is the key import
import edu.wpi.first.apriltag.AprilTagFields;      // For loading predefined fields
//import edu.wpi.first.math.geometry.Pose3d;

    
public class Photonvision {
    //Constants and Fields
    
    private Matrix<N3, N1> curStdDevs;
    //private EstimateConsumer estConsumer;
       
        //}
    
        //Simulation
        private PhotonCameraSim cameraSim;
        private VisionSystemSim visionSim;
    
        //public void Vision(EstimateConsumer estConsumer){
            //this.estConsumer = estConsumer;
        //PhotonCamera camera = new PhotonCamera("maincam");

    //}

    public static final Transform3d kRobotToCam =
    new Transform3d(new Translation3d(0, 0.3556, 0), new Rotation3d(0, 0, 0));
    private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);

    public Optional<EstimatedRobotPose> getEstimatedRobotPose() {return photonEstimator.update(result);}


    
    // Change this to match the name of your camera
     PhotonCamera camera = new PhotonCamera("maincam");

    // Query the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult();
    
    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    // Get a list of currently tracked targets.
    List<PhotonTrackedTarget> targets = result.getTargets();

    // Get the current best target.
    PhotonTrackedTarget target = result.getBestTarget();    

    // Get information from target.
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    Transform3d pose = target.getBestCameraToTarget();
    List<TargetCorner> corners = target.getDetectedCorners();

    // Get information from target.
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    // Capture pre-process camera stream image
    //camera.takeInputSnapshot();

    // Capture post-process camera stream image
    //camera.takeOutputSnapshot();

    Transform3d robotToCameraTransform = new Transform3d(
        new Translation3d(0, 0.3556, 0),
        new Rotation3d(0, 0, 0) );
    

        private static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();{
  Optional<Pose3d> tagPose = kTagLayout.getTagPose(target.getFiducialId());
    //Get target pose correctly
  if (tagPose.isPresent()) {
    //Calculate camera-to-target transform
    Transform3d cameratoTarget = target.getBestCameraToTarget();

  

    //Get distance in meters
    double distance = cameratoTarget.getTranslation().getNorm();
    
    //Calculate field-relative robot pose
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
        cameratoTarget,
        tagPose.get(),
        robotToCameraTransform
        );}

        
  }}

