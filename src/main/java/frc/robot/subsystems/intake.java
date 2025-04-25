package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intake extends SubsystemBase {
    StructPublisher<Pose3d> intakePublisher = NetworkTableInstance.getDefault().getStructTopic("Intake Position", Pose3d.struct).publish();
    Pose3d pose = new Pose3d(0,0,0, Rotation3d.kZero);
    int intakeRotation = 0;

    public intake() {
        
    }

    public void moveIntake(int degrees) {
        intakeRotation += degrees;
    }

    public Command moveIntakeCommand(int degrees) {
        return run(() -> moveIntake(degrees));
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            if (Constants.OperatorConstants.driverController.a().getAsBoolean() && intakeRotation > -87) {
                intakeRotation -= 3;
            } else if (!Constants.OperatorConstants.driverController.a().getAsBoolean() && intakeRotation < 0) {
                intakeRotation += 3;
            }
        }

        pose = new Pose3d(-0.2,0,0.064, new Rotation3d(0, Units.degreesToRadians(intakeRotation), 0));
        intakePublisher.set(pose);
    }
}
