/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  AHRS navx = new AHRS(Port.kMXP);
   
   TalonSRX left;
   TalonSRX right;
   //TalonSRX leftFollow;
   //TalonSRX rightFollow;

  public DriveTrain() {
    left = new TalonSRX(2);
    right = new TalonSRX(3);
    //leftFollow = new TalonSRX(4);

    right.setInverted(true);
    right.setSensorPhase(true);
    left.setSensorPhase(true);
    //leftFollow.setInverted(true);
  }

  public void tankDrive(double lPow, double rPow) {
    left.set(ControlMode.Velocity, lPow);
    right.set(ControlMode.Velocity, rPow);
    //leftFollow.set(ControlMode.Follower, 2);
    //rightFollow.set(ControlMode.Follower, 3);
  }
  public double getAngle() {
    if(!navx.isCalibrating()) {return navx.getAngle();}
    return 0;
    }
  public void resetAngle() {
    navx.zeroYaw();
  }
  public void resetDistance(){
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }
  public double getDistance(){
    double leftD = left.getSelectedSensorPosition();
    double rightD = right.getSelectedSensorPosition();
    System.out.println("Left: " + leftD + ", Right: " + rightD);
    return (leftD + rightD)/2;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tankDrive(Robot.m_robotContainer.getLeftJoy().getY(), Robot.m_robotContainer.getRightJoy().getY());
  }
}
