// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.*;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem swerveSubsystem;
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(4);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(4);
  private double deadBand = 0.1;
  public ManualDrive(SwerveSubsystem _swerveSubsystem) {
    this.swerveSubsystem = _swerveSubsystem;
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(baseJoystick.getRawButton(6) == true){
      xSpeed = xLimiter.calculate(-Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(1), deadBand))*0.6;
      ySpeed = yLimiter.calculate(-Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(0), deadBand))*0.6;
      zSpeed = -Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(2), deadBand)*0.8;
    }
    else{
      xSpeed = xLimiter.calculate(-Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(1), deadBand))*0.4;
      ySpeed = yLimiter.calculate(-Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(0), deadBand))*0.4;
      zSpeed = -Constants.SwerveConstants.joysickValue(baseJoystick.getRawAxis(2), deadBand)*0.8;
    }
    swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
