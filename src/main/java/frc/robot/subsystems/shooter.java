package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
    StructPublisher<Pose3d> shooterPublisher = NetworkTableInstance.getDefault().getStructTopic("Shooter Position", Pose3d.struct).publish();
    Pose3d pose = new Pose3d(0.1,0,0.28, Rotation3d.kZero);

    public static boolean Loaded = false;
    
    public shooter() {
        shooterPublisher.set(pose);
    }
}
