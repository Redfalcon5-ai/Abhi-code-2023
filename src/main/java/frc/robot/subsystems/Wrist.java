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
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  private final DoubleSolenoid wrist = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 8, 9);

  public enum Mode{
    UP,
    DOWN
  }

  Mode mode = Mode.UP;

  public Wrist() {
    //Up();
  }

  public Value getPos(){
    return wrist.get();
  }

  public void Up(){
      wrist.set(Value.kForward);
  }

  public void Down(){
      wrist.set(Value.kReverse);
  }

  public void setMode(Mode mode){
    this.mode= mode;
  }

  public Mode getMode(){
    return mode;
  }

  @Override
  public void periodic() {
    if(mode == Mode.UP) Up();
    else if(mode == Mode.DOWN) Down();

    SmartDashboard.putString("wristMode", mode.name());
  }
}
