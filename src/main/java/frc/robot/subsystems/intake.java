package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
    StructPublisher<Pose3d> LoadedNotePublisher = NetworkTableInstance.getDefault().getStructTopic("LoadedNote", Pose3d.struct).publish();
    StructArrayPublisher<Pose3d> NotePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("NotePositions", Pose3d.struct).publish();
    StructPublisher<Pose3d> intakePublisher = NetworkTableInstance.getDefault().getStructTopic("Intake Position", Pose3d.struct).publish();
    Pose3d pose = new Pose3d(0,0,0, Rotation3d.kZero);
    int intakeRotation = 0;
    boolean IntakeDown = false;
    CommandSwerveDrivetrain drivetrain;
    Pose3d drivePose;
    Pose2d intakePose;
    Pose2d nearestNote;
    List<Pose2d> notePositions = new ArrayList<>();

    public void notePositions() {
        notePositions.add(new Pose2d(2.89, 6.99, Rotation2d.kZero));
        notePositions.add(new Pose2d(2.89, 5.54, Rotation2d.kZero));
        notePositions.add(new Pose2d(2.89, 4.1, Rotation2d.kZero));
        notePositions.add(new Pose2d(8.28, 7.44, Rotation2d.kZero));
        notePositions.add(new Pose2d(8.28, 5.78, Rotation2d.kZero));
        notePositions.add(new Pose2d(8.28, 4.1, Rotation2d.kZero));
        notePositions.add(new Pose2d(8.28, 2.44, Rotation2d.kZero));
        notePositions.add(new Pose2d(8.28, .77, Rotation2d.kZero));
        notePositions.add(new Pose2d(13.67, 6.99, Rotation2d.kZero));
        notePositions.add(new Pose2d(13.67, 5.54, Rotation2d.kZero));
        notePositions.add(new Pose2d(13.67, 4.1, Rotation2d.kZero));
    };

    public void refreshNotes() {
        Pose3d[] refreshNotes = new Pose3d[notePositions.size()];
        for (int i = 0; i < notePositions.size(); i++) {
            refreshNotes[i] = new Pose3d(notePositions.get(i)).transformBy(new Transform3d(0, 0, .025, Rotation3d.kZero));
        }
        notePose3D = refreshNotes;
    }

    Pose3d[] notePose3D = {
        (new Pose3d(2.89, 6.99, .025, Rotation3d.kZero)),
        (new Pose3d(2.89, 5.54, .025, Rotation3d.kZero)),
        (new Pose3d(2.89, 4.1, .025, Rotation3d.kZero)),
        (new Pose3d(8.28, 7.44, .025, Rotation3d.kZero)),
        (new Pose3d(8.28, 5.78, .025, Rotation3d.kZero)),
        (new Pose3d(8.28, 4.1, .025, Rotation3d.kZero)),
        (new Pose3d(8.28, 2.44, .025, Rotation3d.kZero)),
        (new Pose3d(8.28, .77, .025, Rotation3d.kZero)),
        (new Pose3d(13.67, 6.99, .025, Rotation3d.kZero)),
        (new Pose3d(13.67, 5.54, .025, Rotation3d.kZero)),
        (new Pose3d(13.67, 4.1, .025, Rotation3d.kZero))
    };

    public intake(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        NotePublisher.set(notePose3D);
        notePositions();
    }

    public void moveIntake(int degrees) {
        intakeRotation += degrees;
    }

    public Command moveIntakeCommand(int degrees) {
        return run(() -> moveIntake(degrees));
    }

    public Command MoveIntakeDown() {
        return runOnce(() -> IntakeDown = true);
    }

    public Command MoveIntakeUp() {
        return runOnce(() -> IntakeDown = false);
    }

    public Command Intake() {
        return runOnce(() -> {if(!shooter.Loaded && intakeRotation == -87 && Math.hypot(intakePose.getX()-nearestNote.getX(), intakePose.getY()-nearestNote.getY()) < .25) {
            shooter.Loaded = true;
            System.out.println(nearestNote);
            System.out.println(notePositions);
            if (notePositions.indexOf(nearestNote) > -1) {
            notePositions.remove(notePositions.indexOf(nearestNote));
            refreshNotes();
            }
        }});
    }

    @Override
    public void periodic() {
        if(DriverStation.isEnabled()) {
            drivePose = new Pose3d(drivetrain.getState().Pose);
            LoadedNotePublisher.set(drivePose);
            intakePose = drivetrain.getState().Pose.transformBy(new Transform2d(-0.75, 0, Rotation2d.kZero));
            nearestNote = new Pose2d(intakePose.nearest(notePositions).getX(), intakePose.nearest(notePositions).getY(), Rotation2d.kZero);

            if(IntakeDown && intakeRotation > -87) {
                intakeRotation -= 3;
            } else if (!IntakeDown && intakeRotation < 0) {
                intakeRotation += 3;
            }
    
            pose = new Pose3d(-0.2,0,0.064, new Rotation3d(0, Units.degreesToRadians(intakeRotation), 0));
            intakePublisher.set(pose);
            NotePublisher.set(notePose3D);
        }
    }
}
