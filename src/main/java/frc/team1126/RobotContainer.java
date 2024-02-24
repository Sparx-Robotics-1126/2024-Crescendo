// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1126;

import java.io.File;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team1126.Constants.SwerveConstants;
import frc.team1126.commands.Shooter.ShootNote;
import frc.team1126.commands.Shooter.SpinShooter;
import frc.team1126.commands.Shooter.SpinShooterForAmp;
import frc.team1126.commands.Storage.EjectNote;
import frc.team1126.commands.Storage.SpinStorage;
import frc.team1126.commands.arm.ArmToGround;
import frc.team1126.commands.arm.DoNothingArm;
import frc.team1126.commands.arm.HoldArmAtPosition;
import frc.team1126.commands.arm.MoveArm;
import frc.team1126.commands.arm.MoveArmForClimb;
import frc.team1126.commands.arm.MoveArmToAmp;
import frc.team1126.commands.arm.MoveArmToPosition;
import frc.team1126.commands.arm.MoveArmWithLimelight;
import frc.team1126.commands.arm.HoldArmAtPosition;
import frc.team1126.commands.climber.MoveClimber;
import frc.team1126.commands.climber.MoveHome;
import frc.team1126.commands.climber.MoveMax;
import frc.team1126.subsystems.CANdleSubsystem;
import frc.team1126.subsystems.Climber;
import frc.team1126.subsystems.Shooter;
import frc.team1126.subsystems.Storage;
import frc.team1126.subsystems.Arm;
import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.CANdleSubsystem.AnimationTypes;
//import frc.team1126.subsystems.SwerveSubsystem;
import frc.team1126.subsystems.sensors.Limelight;

public class RobotContainer {

  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;

  public static final CANdleSubsystem m_candleSubsystem = new CANdleSubsystem();

  private static HashMap<String, Command> m_pathMap = new HashMap<>();

  final static SendableChooser<Command> m_chooser = new SendableChooser<>();

  CommandXboxController m_driver = new CommandXboxController(Constants.GeneralConstants.DRIVER_CONTROLLER_ID);
  CommandXboxController m_operator = new CommandXboxController(Constants.GeneralConstants.OPERATOR_CONTROLLER_ID);

  /* Operator Buttons */
  // private final JoystickButton cubeMode = new JoystickButton(operator.getHID(),
  // XboxController.Button.kStart.value);
  // private final JoystickButton coneMode = new JoystickButton(operator.getHID(),
  // XboxController.Button.kBack.value);

  public final static SwerveSubsystem m_swerve = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"));

  public final static Limelight m_limeLight = new Limelight();

  public final static Climber m_climber = new Climber();

  public final static Arm m_arm = new Arm();

  public final static Storage m_storage = new Storage();

  public final static Shooter m_shooter = new Shooter();

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();

NamedCommands.registerCommand("moveArmTo32", new MoveArmToPosition(m_arm,m_limeLight,32));

    Command driveFieldOrientedAnglularVelocity = m_swerve.driveCommand(
        () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftY(), .1), -1,
            1),
        () -> MathUtil.clamp(MathUtil.applyDeadband(-m_driver.getLeftX(), .1), -1,
            1),
        () -> -m_driver.getRightX());

    m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    // swerve.setDefaultCommand(new DriveFieldRelative(swerve,
    // () -> driver.getRawAxis(translationAxis)*-1,
    // () -> driver.getRawAxis(strafeAxis) *-1,
    // () -> driver.getRawAxis(rotationAxis)*-1));

    configureChooser();

    //use only if we want manual control of climber;
    m_climber
        .setDefaultCommand(new MoveClimber(m_climber, () -> -m_operator.getRawAxis(XboxController.Axis.kLeftY.value)));

    m_arm.setDefaultCommand(new MoveArm(m_arm, () -> -m_operator.getRawAxis(XboxController.Axis.kRightY.value)));

    // m_climber.setDefaultCommand(m_climber.moveClimber(
    // MathUtil.applyDeadband(m_operator.getRawAxis(XboxController.Axis.kLeftY.value),
    // .1)));
    // climber.setDefaultCommand(climber.moveLeftClimber(
    // MathUtil.applyDeadband(operator.getRawAxis(XboxController.Axis.kLeftY.value),
    // .1)));
  }

  private void configureDriverBindings() {
    m_driver.leftTrigger().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
  }

  private void configureOperatorBindings() {
   
    var angle = 32;

    var closeAngle = 21;
    var midAngle = 34.5;
    var ampAngle = 53;
    var driveAngle = 35;

    // System.out.println("getting calcresult in robotContainer: " + angle1);

    // m_operator.rightTrigger().whileTrue(new ShootNote(m_shooter,m_storage)
    // .alongWith( new MoveArmToPosition(m_arm, angle)));

    // Fire note
    m_operator.rightTrigger().whileTrue(new ShootNote(m_shooter, m_storage)
        .alongWith(new HoldArmAtPosition(m_arm)));
    // m_operator.b().whileTrue(new EjectNote(m_storage));

    // m_operator.a().whileTrue(new SpinStorage(m_storage).andThen(new
    // MoveArmToPosition(m_arm,m_limeLight.calculateTargetAngle()).alongWith(new
    // SpinShooter(m_shooter))));

    // pickup and move to driveing/amp angle. no need for shooter yet
    m_operator.a().onTrue(new SpinStorage(m_storage)
        .andThen(new MoveArmToPosition(m_arm, m_limeLight, driveAngle)));

    //move are to position.  Limelight will return the angle base on specific distances (close, mid, far)
    m_operator.x().whileTrue(new MoveArmToPosition(m_arm, m_limeLight, closeAngle).alongWith(new SpinShooter(m_shooter)));
    m_operator.b().whileTrue(new MoveArmToPosition(m_arm, m_limeLight, midAngle).alongWith(new SpinShooter(m_shooter)));
    m_operator.y().whileTrue(new MoveArmToAmp(m_arm));
    // .alongWith(new SpinShooter(m_shooter))

    // m_operator.y().whileTrue(new MoveArmToPosition(m_arm, m_limeLight));

    //climber
    m_operator.leftBumper().whileTrue(new MoveMax(m_climber, 0.5));
    m_operator.rightBumper().whileTrue(new MoveHome(m_climber, 0.5));

    // m_operator.b().whileTrue(new MoveArmWithLimelight(m_arm, m_limeLight)
    //     .alongWith(new SpinShooter(m_shooter)));

    m_operator.back().whileTrue(new MoveArmForClimb(m_arm));
    m_operator.povUp().onTrue(new MoveArmToPosition(m_arm, m_limeLight, driveAngle));

 

    m_operator.povDown().onTrue(new MoveArm(m_arm, -.2));

    // close 25
    // mid 34.5
    // far
    // amp no shooter 53

    m_operator.leftTrigger().whileTrue(new EjectNote(m_storage));
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
    
    m_chooser.setDefaultOption("Leave Community", new PathPlannerAuto("Leave Community"));
    m_chooser.addOption("Mover arm test",new MoveArm(m_arm, -.2).withTimeout(1)
                .andThen(new MoveArmToPosition(m_arm, m_limeLight, 21).alongWith(new SpinShooter(m_shooter))
                .withTimeout(2))
                .andThen(new ShootNote(m_shooter, m_storage)
                .alongWith(new HoldArmAtPosition(m_arm)).withTimeout(1).andThen(new WaitCommand(1))
                .andThen(new ArmToGround(m_arm)).withTimeout(1).alongWith(new PathPlannerAuto("Leave Community")))
                );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
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

  public void EndGameRumble() {

    if (DriverStation.getMatchTime() < SwerveConstants.ENDGAME_SECONDS
        && DriverStation.getMatchTime() > SwerveConstants.STOP_RUMBLE_SECONDS) {
      m_candleSubsystem.changeAnimation(AnimationTypes.purpleStrobe);
      m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
      m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.5);

    } else {
      if (DriverStation.getAlliance().get() == Alliance.Red && !m_storage.getHasNote()) {
        m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.RED);
      } else if (DriverStation.getAlliance().get() == Alliance.Blue && !m_storage.getHasNote()) {
        m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.BLU);
      } else {
        m_candleSubsystem.setLEDState(CANdleSubsystem.LEDState.YELLOW);
      }
      m_driver.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);
      m_operator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0);

    }
  }

}
