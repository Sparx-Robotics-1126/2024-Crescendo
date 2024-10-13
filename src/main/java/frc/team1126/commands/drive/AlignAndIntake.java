package frc.team1126.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team1126.RobotContainer;
import frc.team1126.commands.Shooter.FinishWhenBeamBroken;
import frc.team1126.commands.Storage.SpinStorage;
import frc.team1126.commands.arm.MoveArmToPosition;
import frc.team1126.subsystems.Arm;
import frc.team1126.subsystems.Shooter;
import frc.team1126.subsystems.Storage;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.sensors.Limelight;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class AlignAndIntake extends SequentialCommandGroup {

    public AlignAndIntake(Shooter shooterSubsystem,
            Arm armSubsystem,
            Storage intakeSubsystem,
            SwerveSubsystem driveSubsystem) {
        addRequirements(RobotContainer.m_shooter, RobotContainer.m_arm, RobotContainer.m_storage, RobotContainer.m_swerve);

        Limelight m_limelight = Limelight.getInstance();
        addCommands(
                new InstantCommand(() -> driveSubsystem.resetNoteHeight()),
                new ParallelRaceGroup(
                        new FinishWhenBeamBroken(shooterSubsystem),
                        new RunCommand(() -> driveSubsystem.driveRobotRelativeToObject(),
                                driveSubsystem)
                                .until(() -> driveSubsystem.isTargetLost()),
                        new SequentialCommandGroup(
                               new SpinStorage(intakeSubsystem, 5))).andThen( new SpinStorage(intakeSubsystem, 5)));

    }
}