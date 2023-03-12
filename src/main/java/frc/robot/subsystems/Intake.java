// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GantryConstants;

public class Intake extends SubsystemBase {
  private final CANSparkMax left;
  private final CANSparkMax right;

  private double leftTrigger;
  private double rightTrigger

  public Intake() {
    left = new CANSparkMax(4, MotorType.kBrushless);
    right = new CANSparkMax(5, MotorType.kBrushless);

    motorSetup(left);
    motorSetup(right);

    left.setInverted(true);
  }

  public void motorSetup(CANSparkMax c){
    c.setSmartCurrentLimit(30);
    c.restoreFactoryDefaults();
    c.setIdleMode(IdleMode.kBrake);
  }

  public void power(double power){
    left.setPower(power);
    right.setPower(power);
  }

  public void setTriggers(double left, double right){
    this.leftTrigger = left;
    this.rightTrigger = right;
  }

  
  @Override
  public void periodic() {
    if(leftTrigger > 0){
        power(leftTrigger);
    }
    else if(rightTrigger > 0){
        power(rightTrigger);
    }
    else power(0);
  }
}
