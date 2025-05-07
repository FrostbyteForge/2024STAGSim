// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooter;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final intake intake = new intake(drivetrain);
    public final shooter shooter = new shooter(intake);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Move Intake Down", intake.MoveIntakeDown());
        NamedCommands.registerCommand("Move Intake Up", intake.MoveIntakeUp());
        NamedCommands.registerCommand("Intake", intake.Intake());

        autoChooser = AutoBuilder.buildAutoChooser("Default");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Constants.OperatorConstants.driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-Constants.OperatorConstants.driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-Constants.OperatorConstants.driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        Constants.OperatorConstants.driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        Constants.OperatorConstants.driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        Constants.OperatorConstants.driverController.back().and(Constants.OperatorConstants.driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        Constants.OperatorConstants.driverController.back().and(Constants.OperatorConstants.driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        Constants.OperatorConstants.driverController.start().and(Constants.OperatorConstants.driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        Constants.OperatorConstants.driverController.start().and(Constants.OperatorConstants.driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        Constants.OperatorConstants.driverController.a().whileTrue(intake.MoveIntakeDown()).whileFalse(intake.MoveIntakeUp());
        Constants.OperatorConstants.driverController.rightTrigger().whileTrue(intake.Intake());
        Constants.OperatorConstants.driverController.b().onTrue(shooter.Discard());

        // reset the field-centric heading on left bumper press
        Constants.OperatorConstants.driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
