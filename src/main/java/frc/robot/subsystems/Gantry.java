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

public class Gantry extends SubsystemBase {
  private final CANSparkMax gantry;

  private double target = 0;
  private double power = 0;

  PIDController PID = new PIDController(0.1, 0, 0);

  private enum Mode {
    POWER,
    POSITION
  }

  private Mode mode = Mode.POWER;

  public Gantry() {
    gantry = new CANSparkMax(GantryConstants.ID, MotorType.kBrushless);

    motorSetup(gantry);

    zeroEncoders();
    gantry.getEncoder().setPositionConversionFactor(GantryConstants.revsToInches);

    gantry.setInverted(false);
  }

  public void motorSetup(CANSparkMax c){
    c.setSmartCurrentLimit(30);
    c.restoreFactoryDefaults();
    c.setIdleMode(IdleMode.kBrake);
  }

  public void zeroEncoders(){
      gantry.getEncoder().setPosition(0);
  }

  public void power(double power){
    gantry.set(power);
  }

  public void setTarget(double t){
    t = Math.max(0,Math.min(t,GantryConstants.max));

    target = t;
  }

  public void setPower(double p){
    power = p;
  }

  public void incrementTarget(double t){
    setTarget(target+t);
  }

  public void incrementTarget(DoubleSupplier t){
    setTarget(target+t.getAsDouble()*0.01);
  }

  public double getTarget(){
    return target;
  }

  public void moveToTarget(double t){
    setTarget(t);
    moveToTarget();
  }

  public void moveToTarget(){
    gantry.set(PID.calculate(gantry.getEncoder().getPosition(), target));
  }

  public double getPos(){
    return gantry.getEncoder().getPosition();
  }

  public void setMode(Mode mode){
    this.mode = mode;
  }

  @Override
  public void periodic() {
    if(power != 0) setMode(Mode.POWER);
    else setMode(Mode.POSITION);

    if(mode == Mode.POWER){
      power(power);
      target = getPos();
    }
    else if(mode == Mode.POSITION){
      //power = 0;
      moveToTarget();
    }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("gantrPos", gantry.getEncoder().getPosition());
    SmartDashboard.putNumber("gantryTarget", target);
    SmartDashboard.putNumber("gantryPower", power);
    SmartDashboard.putString("gantryMode", mode.name());
  }
}
