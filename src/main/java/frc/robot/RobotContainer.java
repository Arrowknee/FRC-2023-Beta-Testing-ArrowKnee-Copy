// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.input.XboxController;
import frc.robot.Sims.SimpleDifferentialDriveSim;
import frc.robot.commands.ArmController;
import frc.robot.commands.ClawController;
import frc.robot.commands.Drive;
import frc.robot.commands.ElevatorController;
import frc.robot.commands.ElevatorPIDController;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorPID;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // ---------- Robot Subsystems ---------- \\
  private final Drivetrain drive = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  // private final ElevatorPID elevatorPID = new ElevatorPID();
  private final Claw claw = new Claw();
 // private final SimpleDifferentialDriveSim driveSim = new SimpleDifferentialDriveSim(); 
  
  // The robot's controllers
  private final XboxController xboxDriver;
  private final XboxController xboxOperator;

  // Create a Sendable Chooser, which allows us to select between Commands (in
  // this case, auto commands)
  private final SendableChooser<Command> chooser = new SendableChooser<Command>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Instantiate our controllers with proper ports.
    this.xboxDriver = new XboxController(3);
    this.xboxOperator = new XboxController(2);

    // Controler Throttle Mappings
    this.drive.setDefaultCommand(
        new Drive(drive, xboxDriver));
    
    this.elevator.setDefaultCommand(
        new ElevatorController(elevator, xboxOperator)); // added, works
  
    this.arm.setDefaultCommand(
      new ArmController(arm, xboxOperator));
      
    //this.driveSim.setDefaultCommand(new Drive(driveSim, xboxDriver));


    //  this.elevatorPID.setDefaultCommand(
        // new ElevatorPIDController(elevatorPID,xboxOperator)); // added, untested

    this.claw.setDefaultCommand(
      new ClawController(claw, xboxOperator)
    );

    this.xboxOperator.getAButton().onTrue(
      new InstantCommand(() -> this.claw.toggle())
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command chosen = chooser.getSelected();
    return chosen;
  }
}
