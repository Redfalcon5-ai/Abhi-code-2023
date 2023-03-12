// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.GoToIntake;
import frc.robot.commands.GoToScoring;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.setElevator;
import frc.robot.commands.setGantry;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gantry;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.auto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Elevator elevator = new Elevator();
  private final Gantry gantry = new Gantry();
  private final Wrist wrist = new Wrist();
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController opp = new CommandXboxController(1);

  private final GoToIntake goToIntake = new GoToIntake(elevator, gantry, wrist);
  private final GoToScoring goToScoring = new GoToScoring(elevator, gantry, wrist);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    elevator.setDefaultCommand(new setElevator(elevator, () -> -opp.getRawAxis(1)*1.27, () -> elevator.getTarget()));
    gantry.setDefaultCommand(new setGantry(gantry, () -> opp.getRawAxis(0), () -> gantry.getTarget()));
    intake.setDefaultCommand(new setIntake(intake, () -> opp.getRawAxis(2)*1.27, () -> opp.getRawAxis(3)*1.27));

    swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(1), 
                () -> -driver.getRawAxis(0), 
                () -> -driver.getRawAxis(4), 
                () -> driver.a().getAsBoolean()
            )
        );


    configureBindings();
  }

  private void configureBindings() {
    opp.a().whileTrue(goToIntake);
    opp.y().whileTrue(goToScoring);

    driver.y().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new auto(swerve);
  }
}
