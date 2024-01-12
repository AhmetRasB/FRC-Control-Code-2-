// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.print.attribute.standard.Compression;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(3);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(1);
  private WPI_VictorSPX leftSlave = new WPI_VictorSPX(1);
  private WPI_VictorSPX rightSlave = new WPI_VictorSPX(2);
  private WPI_TalonSRX armMotor = new WPI_TalonSRX(5);
  private WPI_VictorSPX armSlave = new WPI_VictorSPX(3);
  private WPI_TalonSRX rollerMotor = new WPI_TalonSRX(4);
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid hatchIntake = new DoubleSolenoid(null, 0, 1);
  private DifferentialDrive drive = new DifferentialDrive(leftMaster,rightMaster);
  private Joystick driverJoystick = new Joystick(0);
  private Joystick operatorJoystick = new Joystick(1);

  private final double kDriveTick2Feet = 1.0/4096*6* Math.PI /12;
  private final double kArmTick2Deg = 360.0 / 512 *26 / 42*18 / 60*18 / 84;

  @Override
  public void robotInit() {
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    armMotor.setInverted(false);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    armSlave.setInverted(InvertType.FollowMaster);

    //encoders
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,10);

    leftMaster.setSensorPhase(false);
    rightMaster.setSensorPhase(true);
    armMotor.setSensorPhase(true);


    //reset encoders to 0
    leftMaster.setSelectedSensorPosition(0,0,10);
    rightMaster.setSelectedSensorPosition(0,0,10);
    armMotor.setSelectedSensorPosition(0,0,10);

    //set encoder boundary limits to stop motors
    armMotor.configReverseSoftLimitThreshold((int)(0/kArmTick2Deg), 10);
    armMotor.configForwardSoftLimitThreshold((int) (175/kArmTick2Deg), 10);
    
    compressor.start();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition()*kArmTick2Deg);
    SmartDashboard.putNumber("Left Drive Encoder Value", leftMaster.getSelectedSensorPosition()*kDriveTick2Feet );
    SmartDashboard.putNumber("Right Drive Encoder Value", rightMaster.getSelectedSensorPosition()* kDriveTick2Feet);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double power = -driverJoystick.getRawAxis(1);
    double turn = driverJoystick.getRawAxis(4);
  //   if(Math.abs(power)< 0.05){
  //     power = 0;
  //   }
  //   if(Math.abs(turn)< 0.05){
  //     power = 0;
  //   }
     drive.arcadeDrive((power*0.6), turn*0.3);
     //arm control 
     double armPower = - operatorJoystick.getRawAxis(1);
     if(Math.abs(armPower)<0.05){
      armPower= 0;
     }
     armPower *=0.5;
     armMotor.set(ControlMode.PercentOutput, armPower);
   }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
