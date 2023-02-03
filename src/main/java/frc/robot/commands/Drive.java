// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.lib.RebelUtil;
import frc.lib.input.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Sims.SimpleDifferentialDriveSim;


/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_driveSubsystem;
  //private final SimpleDifferentialDrivetrainSim m_driveSubsystem; /*UNCOMMENT THIS */
  private final XboxController xboxDriver;
  private final double MAX_FORWARD_SPEED = 5;
  private final double MAX_TURN_SPEED = 5;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Drive(Drivetrain driveSubsystem, XboxController controller) {
    xboxDriver = controller;
    m_driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }
  /* UNCOMMENT WHEN NEEDED
  public Drive(SimpleDifferentialDriveSim drive, XboxController controller){
    xboxDriver = controller;
    m_driveSubsystem = m_driveSubsystem;
    addRequirements(m_driveSubsystem);
  }
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = RebelUtil.linearDeadband(xboxDriver.getLeftY(), 0.1) * MAX_FORWARD_SPEED;
    double turnSpeed = RebelUtil.linearDeadband(-xboxDriver.getRightX(), 0.1) * MAX_TURN_SPEED;
    m_driveSubsystem.drive(forwardSpeed, turnSpeed);
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
