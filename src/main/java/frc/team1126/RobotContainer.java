// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team1126.Constants.GeneralConstants;
import frc.team1126.Constants.SwerveConstants;
import frc.team1126.commands.Limelight.LLRotationAlignCommand;
import frc.team1126.commands.Shooter.CalculateShooter;
import frc.team1126.commands.Shooter.ShootNote;
import frc.team1126.commands.Shooter.SpinShooter;
import frc.team1126.commands.Storage.EjectNote;
import frc.team1126.commands.Storage.SpinStorage;
import frc.team1126.commands.arm.ArmWithCalculation;
import frc.team1126.commands.arm.MoveArm;
import frc.team1126.commands.arm.MoveArmForClimb;
import frc.team1126.commands.arm.MoveArmToAmp;
import frc.team1126.commands.arm.MoveArmToPosition;
import frc.team1126.commands.arm.MoveArmToPositionNoPID;
import frc.team1126.commands.arm.MoverArmPIDAngle;
import frc.team1126.commands.arm.ZeroPigeon;
import frc.team1126.commands.climber.MoveClimber;
import frc.team1126.commands.climber.MoveHome;
import frc.team1126.commands.climber.MoveMax;
import frc.team1126.subsystems.CANdleSubsystem;
import frc.team1126.subsystems.Climber;
import frc.team1126.subsystems.Shooter;
import frc.team1126.subsystems.Storage;
import frc.team1126.subsystems.Arm;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.CANdleSubsystem.LEDState;
import frc.team1126.subsystems.sensors.Limelight;

public class RobotContainer {

    private final int m_rotationAxis = XboxController.Axis.kRightX.value;

    public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

    final static SendableChooser<Command> m_chooser = new SendableChooser<>();

    CommandXboxController m_driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
    CommandXboxController m_operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

    public static SwerveSubsystem m_swerve;

    public final static Limelight m_limeLight = new Limelight();

    public final static Climber m_climber = new Climber();

    public final static Arm m_arm = new Arm();

    public final static Storage m_storage = new Storage();

    public final static Shooter m_shooter = new Shooter();

    public RobotContainer() {
      
        /* REGISTER PATHPLANNER COMMANDS HERE */
        // ARM COMMANDS
        NamedCommands.registerCommand("moveArmTo23", new MoverArmPIDAngle(m_arm, 25.5).withTimeout(1.5));
        NamedCommands.registerCommand("moveArmTo30", new MoverArmPIDAngle(m_arm, 30.8).withTimeout(1.5));
        NamedCommands.registerCommand("moveArmTo53", new MoverArmPIDAngle(m_arm, 53).withTimeout(1.5));
        NamedCommands.registerCommand("moveArmToHome", new MoverArmPIDAngle(m_arm, .02).withTimeout(2));
        NamedCommands.registerCommand("moveDown", new MoveArmToPositionNoPID(m_arm, m_limeLight, .02).withTimeout(1.5));
        NamedCommands.registerCommand("holdClose",
                new MoverArmPIDAngle(m_arm, GeneralConstants.CLOSE_SPEAKER_ANGLE).withTimeout(1.5));
                NamedCommands.registerCommand("holdMid",
                new MoverArmPIDAngle(m_arm, GeneralConstants.MID_SPEAKER_ANGLE).withTimeout(1.5));
        NamedCommands.registerCommand("calculateArm", new ArmWithCalculation(m_arm).withTimeout(1.5));
        // SHOOTER COMMANDS
        NamedCommands.registerCommand("shootNote",
                new ShootNote(m_shooter, m_storage, m_shooter.calculateShooter()).withTimeout(1));
        NamedCommands.registerCommand("spinShooter",
                new SpinShooter(m_shooter, GeneralConstants.CLOSE_SPEAKER_POWER).withTimeout(1.5));
        NamedCommands.registerCommand("spinShooterMid",
                new SpinShooter(m_shooter, GeneralConstants.MID_SPEAKER_POWER).withTimeout(1.5));
        NamedCommands.registerCommand("calculateShooter", new CalculateShooter(m_shooter).withTimeout(2));
        // STORAGE COMMANDS
        NamedCommands.registerCommand("spinStorage",
                new SpinStorage(m_storage, GeneralConstants.STORAGE_POWER));
        NamedCommands.registerCommand("spinStorageLong",
                new SpinStorage(m_storage, GeneralConstants.STORAGE_POWER).withTimeout(3));
        NamedCommands.registerCommand("lowPowerStorage",
                new SpinStorage(m_storage, GeneralConstants.LOW_STORAGE_POWER).withTimeout(3));
        NamedCommands.registerCommand("ejectNote", new EjectNote(m_storage).withTimeout(2));

        //OTHER COMMANDS
        //NamedCommands.registerCommand("limelightTarget", new LLRotationAlignCommand(m_swerve).withTimeout(1.5));

        m_swerve = new SwerveSubsystem(
                new File(Filesystem.getDeployDirectory(), "swerve"));

        Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
                () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftY(), .1), -1,
                        1),
                () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftX(), .1), -1,
                        1),
                () -> -m_driver.getRightX());

        m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
       
        configureChooser();

        // use only if we want manual control of climber;
        m_climber
                .setDefaultCommand(
                        new MoveClimber(m_climber, () -> -m_operator.getRawAxis(XboxController.Axis.kLeftY.value)));

        m_arm.setDefaultCommand(new MoveArm(m_arm, () -> -m_operator.getRawAxis(XboxController.Axis.kRightY.value)));
        configureDriverBindings();
        configureOperatorBindings();

    }

    private void configureDriverBindings() {
        
        m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
        m_driver.a().whileTrue(new LLRotationAlignCommand(m_swerve));
    }

    private void configureOperatorBindings() {

        /* Operator Buttons */
       
        // Fire note
        m_operator.rightTrigger().and(m_operator.x())
                .whileTrue(new ShootNote(m_shooter, m_storage, m_shooter.calculateShooter())
                        .alongWith(new ArmWithCalculation(m_arm))); 
        m_operator.rightTrigger().and(m_operator.b())
                .whileTrue(new ShootNote(m_shooter, m_storage, 3000)
                        .alongWith(new MoverArmPIDAngle(m_arm, 15.5)));
        m_operator.leftTrigger().whileTrue(new EjectNote(m_storage));

        // pickup and move to driving/amp angle. no need for shooter yet
        m_operator.a().whileTrue(new SpinStorage(m_storage, GeneralConstants.STORAGE_POWER)
                .andThen(new MoverArmPIDAngle(m_arm, GeneralConstants.DRIVE_ANGLE)));
        
        //pickup without moving to angle
        m_operator.povRight().whileTrue(new SpinStorage(m_storage, GeneralConstants.STORAGE_POWER));

        // move are to position. Limelight will return the angle base on specific
        // distances (close, mid, far)
        m_operator.x().whileTrue((new ArmWithCalculation(m_arm))
                .alongWith(new CalculateShooter(m_shooter))); //
        // m_operator.b().whileTrue(new MoverArmPIDAngle(m_arm, GeneralConstants.MID_SPEAKER_ANGLE)
        //         .alongWith(new SpinShooter(m_shooter, GeneralConstants.MID_SPEAKER_POWER)));

        m_operator.b().whileTrue(new SpinShooter(m_shooter, 3000).alongWith(new MoverArmPIDAngle(m_arm, 15.5)));
        m_operator.y().whileTrue(new MoveArmToAmp(m_arm));
       
        //move arm to drive / acquire position
        m_operator.povUp().onTrue(new MoverArmPIDAngle(m_arm, GeneralConstants.DRIVE_ANGLE));
        m_operator.povDown().onTrue(new MoveArmToPositionNoPID(m_arm, m_limeLight, m_rotationAxis));

        // climber
        m_operator.leftBumper().whileTrue(new MoveMax(m_climber, 0.5));
        m_operator.rightBumper().whileTrue(new MoveHome(m_climber, 0.5));
        m_operator.back().whileTrue(new MoveArmForClimb(m_arm));

       //reset arm pigeon
        m_operator.rightStick().onTrue(new ZeroPigeon(m_arm));

        // close 25
        // mid 34.5
        // far
        // amp no shooter 53

        // m_operator.leftTrigger().whileTrue(new EjectNote(m_storage));
        // operator.start().onTrue(new InstantCommand(()
        // ->m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.CONE)));
        // operator.back().onTrue(new InstantCommand(() ->
        // m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.CUBE)));
        // operator.x().onTrue(new InstantCommand(() ->
        // operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)));

        // operator.leftBumper().whileTrue(new MoveMax(climber, .05));
        // operator.rightBumper().whileTrue(new MoveHome(climber, .05));

        // operator.leftStick().whileTrue(
        // climber.moveLeftClimber(operator.getLeftY()));

    }

    double getXSpeed() {
        int pov = m_driver.getHID().getPOV();
        double finalX;

        if (pov == 0)
            finalX = -0.05;
        else if (pov == 180)
            finalX = 0.05;
        else if (Math.abs(m_driver.getLeftY()) <= 0.1)
            finalX = 0.0;
        else
            finalX = m_driver.getLeftY() * 0.75 * (1.0 + m_driver.getLeftTriggerAxis());

        return finalX;
    }

    public double getYSpeed() {
        int pov = m_driver.getHID().getPOV();

        double finalY;
        if (pov == 270 || pov == 315 || pov == 225)
            finalY = -0.05;
        else if (pov == 90 || pov == 45 || pov == 135)
            finalY = 0.05;
        else if (Math.abs(m_driver.getLeftX()) <= 0.1)
            finalY = 0.0;
        else
            finalY = m_driver.getLeftX() * 0.75 * (1.0 + m_driver.getLeftTriggerAxis());

        // if (SwerveType.isStandard())
        // finalY = -finalY;
        return finalY;
    }

    public void configureChooser() {

        // autos using pathplanner
        m_chooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        m_chooser.addOption("Leave Start Area", new PathPlannerAuto("LeaveStartingZone"));
        // m_chooser.addOption("Leave Angled Start",new
        // PathPlannerAuto("angledLeaveAuto"));
        // m_chooser.addOption("Move Arm Pathplanner", new
        // PathPlannerAuto("moveArmTest"));
        // m_chooser.addOption("Leave from subwoofer", new
        // PathPlannerAuto("angledLeaveAuto"));
        // m_chooser.addOption("fromangleshoot(PATH ONLY)", new
        // PathPlannerAuto("movefromangleshoot"));
        m_chooser.addOption("ShootAndMoveFromFront", new PathPlannerAuto("ShootMoveShoot"));

        m_chooser.addOption("shootfromAmpSide", new PathPlannerAuto("MoveFromAngleShoot"));
        // m_chooser.addOption("test",new PathPlannerAuto("test"));
        // m_chooser.addOption("shootmoveshootpaths", new
        // PathPlannerAuto("shootmoveshootpaths"));
        // am_chooser.addOption("moveFromSourceSide", new
        // PathPlannerAuto("MoveFromRight"));
        m_chooser.addOption("shootFromSourceSide", new PathPlannerAuto("ShootFromRight"));
        m_chooser.addOption("3 NOTE AUTO", new PathPlannerAuto("3NoteAuto"));
        m_chooser.addOption("x tuning", new PathPlannerAuto("xTuningTest"));
        m_chooser.addOption("y tuning", new PathPlannerAuto("test"));
        m_chooser.addOption("angleTuning",new PathPlannerAuto("AngleTuning"));

        m_chooser.addOption("shootStage", new PathPlannerAuto("playoffs 1"));

        //m_chooser.addOption("quals 34",new PathPlannerAuto("quals 34"));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *\
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // swerve.zeroGyro();
        //
        // // m_driveSubsystem.setHeading(180);
        // Timer.delay(0.05);
        // // the command to be run in autonomous

        // return _chooser.getSelected();
        return m_chooser.getSelected();
        // return swerve.getAutonomousCommand(_chooser.getSelected().getName(), true);

    }

    public void setCANdle() {
        var ll = Limelight.getInstance();
        // ☝️🤓 <- me fr
        if (RobotContainer.m_storage.hasNote() && !ll.hasSpeakerTarget()) {
            m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.YELLOW);
        } else if (ll.hasSpeakerTarget() && RobotContainer.m_storage.getHasNote()) {
            // double targetDistance = ll.calculateTargetDistanceInInches();
            m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.GREEN);// close angle
        } else {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.RED);
            } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
                m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.BLU);
            }
        }
    }

    public void EndGameRumble() {

        if(DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 18) {
            m_candleSubsystem.setLEDState(LEDState.PURPLE);
        }

        if (DriverStation.getMatchTime() < SwerveConstants.ENDGAME_SECONDS 
                && DriverStation.getMatchTime() > SwerveConstants.STOP_RUMBLE_SECONDS) {
            
            m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);
            m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1);

        } else {
            m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
            m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

        }
    }

    //method for operator to know whether or not the shooter is up to speed for any given distance
    public void upToSpeedRumble() {
        if(m_shooter.isMotorUpToSpeed()) {
            m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,1);
        } 
        m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);
    }

}
