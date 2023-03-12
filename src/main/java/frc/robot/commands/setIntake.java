// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Gantry;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class setIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_Intake;
  private DoubleSupplier left;
  private DoubleSupplier right;

  public setIntake(Intake intake, DoubleSupplier left, DoubleSupplier right) {
    m_Intake = intake;
    this.left = left;
    this.right = right;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(power.getAsDouble() < 0.1 && power.getAsDouble() > -0.1) power = () -> 0;
    double newLeft = MathUtil.applyDeadband(left.getAsDouble(), 0.1);
    double newRight = MathUtil.applyDeadband(right.getAsDouble(), 0.1);

    m_Intake.setTriggers(newLeft.getAsDouble(), newRight.getAsDouble());
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
