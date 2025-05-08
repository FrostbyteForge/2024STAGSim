package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
    StructPublisher<Pose3d> loadedNotePublisher = NetworkTableInstance.getDefault().getStructTopic("Loaded Note", Pose3d.struct).publish();
    StructPublisher<Pose3d> shotNotePublisher = NetworkTableInstance.getDefault().getStructTopic("Shot Note", Pose3d.struct).publish();
    
    private static final Pose3d blueSpeaker = new Pose3d(0.225, 5.55, 2.1, Rotation3d.kZero);
    private static final Pose3d redSpeaker = new Pose3d(16.317, 5.55, 2.1, Rotation3d.kZero);

    public Pose3d shotNote = new Pose3d();
    boolean isShooting = false;

    List<Pose3d> notePositions = new ArrayList<Pose3d>();
    Pose3d[] shotNotes = new Pose3d[]{};
    Pose3d loadedNote = new Pose3d(0, 0, -5, Rotation3d.kZero);

    double time = 0;

    public void refreshNotes() {
        Pose3d[] refreshNotes = new Pose3d[notePositions.size()];
        for (int i = 0; i < notePositions.size(); i++) {
            refreshNotes[i] = notePositions.get(i).transformBy(new Transform3d(0, 0, .025, Rotation3d.kZero));
        }
        shotNotes = refreshNotes;
    }


    intake intake;

    StructPublisher<Pose3d> shooterPublisher = NetworkTableInstance.getDefault().getStructTopic("Shooter Position", Pose3d.struct).publish();
    Pose3d pose = new Pose3d(0.1,0,0.28, Rotation3d.kZero);

    public static boolean loaded = true;
    
    public shooter(intake intake) {
        this.intake = intake;
        shooterPublisher.set(pose);
    }

   /* public void shootNote() {
        Pose3d noteStartPose = new Pose3d(intake.drivetrain.getState().Pose);
        notePositions.add(noteStartPose);
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance() == Optional.of(Alliance.Blue)) {
                time = Math.hypot(noteStartPose.getX() - blueSpeaker.getX(), noteStartPose.getY() - blueSpeaker.getY()) * .25;
                notePositions.get(notePositions.indexOf(noteStartPose)).interpolate(blueSpeaker, time);
            } else {
                time = Math.hypot(noteStartPose.getX() - redSpeaker.getX(), noteStartPose.getY() - redSpeaker.getY()) * .25;
                notePositions.get(notePositions.indexOf(noteStartPose)).interpolate(redSpeaker, time);
            }
        }
    } */

    public void shootNote() {
        shotNote = loadedNote;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                time = 0;
                isShooting = true;
            } else {
                time = 0;
                isShooting = true;
            }
        }
    }

    public Command shoot() {
        return runOnce(() -> {
            shootNote();
            loaded = false;
        }).unless(() -> !loaded);
    }


    public void scoreNote(Pose3d note) {
        notePositions.remove(notePositions.indexOf(note));
        refreshNotes();
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Loaded", loaded);
        if (time == 1 && isShooting) {
            shotNote = new Pose3d();
            isShooting = false;
            time = 0;
        }
        if (DriverStation.getAlliance().isPresent() && isShooting) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                
            } else {
                
            }
        }
        shotNotePublisher.set(shotNote);
        if (loaded) {
            loadedNote = new Pose3d(intake.drivetrain.getState().Pose).transformBy(new Transform3d(0.1, 0, 0.28, new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)));
        } else {
            loadedNote = new Pose3d(0, 0, -5, Rotation3d.kZero);
        }
        loadedNotePublisher.set(loadedNote);
        if (!notePositions.isEmpty()) {
            refreshNotes();
        }
    }
}
