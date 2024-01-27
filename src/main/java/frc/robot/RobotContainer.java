// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoCommand;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.AutoSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystem
  private final SwerveSubsystem m_swervesubsystem = new SwerveSubsystem();
  private final AutoSubsystem m_autosubsystem = new AutoSubsystem();
  // Command
  public ManualDrive manualDrive = new ManualDrive(m_swervesubsystem);
  public AutoCommand autocommand = new AutoCommand(m_swervesubsystem);
  // Joystick
  public static final Joystick baseJoystick = new Joystick(0);
  // Chooser
  private final SendableChooser<Command> autoChooser;
  // private SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    // Registed NoteIn
    NamedCommands.registerCommand("NoteIn", Commands.run(()->{
      m_autosubsystem.inTrue();
    }, m_autosubsystem).withTimeout(1));
    // Registed NoteShoot
    NamedCommands.registerCommand("NoteShoot", Commands.run(()->{
      m_autosubsystem.shootTrue();
    }, m_autosubsystem).withTimeout(0.5));
    // Registed Stop
    NamedCommands.registerCommand("Stop", Commands.run(()->{
      m_swervesubsystem.drive_auto(new ChassisSpeeds(0, 0, 0));
    }).withTimeout(0.5));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(baseJoystick, 1).onTrue(Commands.runOnce(()->{
      m_swervesubsystem.resetGyro();
      m_swervesubsystem.setPose(new Pose2d(0, 0, new Rotation2d()));})
    );
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    m_autosubsystem.inFalse();
    m_autosubsystem.shootFalse();
    m_swervesubsystem.resetGyro();
    // autoSubsystem.falsevery();
    return autoChooser.getSelected();
  }
}
