// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gantry;
import frc.robot.subsystems.Wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GoToScoring extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Elevator m_Elevator;
  private final Gantry m_Gantry;
  private final Wrist m_Wrist;

  private double elePos = 25;
  private double ganPos = 20;
  private boolean wristUp = false;

  public GoToScoring(Elevator elevator, Gantry gantry, Wrist wrist) {
    m_Elevator = elevator;
    m_Gantry = gantry;
    m_Wrist = wrist;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(power.getAsDouble() < 0.1 && power.getAsDouble() > -0.1) power = () -> 0;
    m_Elevator.setTarget(elePos);
    m_Gantry.setTarget(ganPos);

    if(wristUp) m_Wrist.Up();
    else m_Wrist.Down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
