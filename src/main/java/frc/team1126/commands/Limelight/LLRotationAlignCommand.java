package frc.team1126.commands.Limelight;
// // Copyright (c) FIRST and other WPILib contributors.

// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

//package frc.team1126.commands.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Toolbox.DoubleSmoother;
import frc.team1126.RobotContainer;
import frc.team1126.Constants.LimelightConstants;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.sensors.Limelight;
import frc.team1126.subsystems.sensors.LimelightHelpers;

public class LLRotationAlignCommand extends Command {

    private static SwerveSubsystem driveSubsystem;

    PIDController RotationPIDAmount;

    DoubleSmoother driveOutputSmoother;
    DoubleSmoother strafeOutputSmoother;

    Translation2d translation = new Translation2d();

    /** Creates a new LLTargetCommand. */
    public LLRotationAlignCommand(SwerveSubsystem swerve) {
        addRequirements(RobotContainer.m_swerve);
        driveSubsystem = swerve;
        // driveSubsystem = RobotContainer.m_swerve;

        RotationPIDAmount = new PIDController(
                LimelightConstants.K_LL_ALIGN_DRIVE_GAINS.kP,
                LimelightConstants.K_LL_ALIGN_DRIVE_GAINS.kI,
                LimelightConstants.K_LL_ALIGN_DRIVE_GAINS.kD);
        RotationPIDAmount.setTolerance(0.5);

        driveOutputSmoother = new DoubleSmoother(LimelightConstants.K_ALIGN_DRIVE_MOTION_SMOOTHING);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // LimelightHelpers.setLEDMode_ForceOn("");
        Limelight.getInstance().setForTargeting(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Limelight.getInstance().hasSpeakerTarget()) {
            if (LimelightHelpers.getTV("")) {

                // SmartDashboard.putNumber("LL TX", LimelightHelpers.getTX(""));

                double rotationAmount = RotationPIDAmount.calculate(LimelightHelpers.getTX(""), 0);

                // in last year's code, we had the x and y values as seperate parameters,
                // replaced by translation2D parameter
                driveSubsystem.drive(new Translation2d(0, 0), rotationAmount, true);

            } else {
                driveSubsystem.drive(new Translation2d(0, 0), 0.0, true);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_PipelineControl("");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RotationPIDAmount.atSetpoint();
    }
}
