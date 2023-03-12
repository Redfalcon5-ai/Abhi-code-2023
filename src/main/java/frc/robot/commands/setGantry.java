// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Gantry;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class setGantry extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Gantry m_Gantry;
  private DoubleSupplier power;
  private DoubleSupplier pos;

  public setGantry(Gantry gantry, DoubleSupplier power, DoubleSupplier pos) {
    m_Gantry = gantry;
    this.power = power;
    this.pos = pos;

    addRequirements(gantry);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if(power.getAsDouble() < 0.1 && power.getAsDouble() > -0.1) power = () -> 0;
    double newPower = MathUtil.applyDeadband(power.getAsDouble(), 0.2);
    m_Gantry.setPower(newPower);
    m_Gantry.setTarget(this.pos.getAsDouble());
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
