// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Drivetrain extends SubsystemBase {
    private static TalonFX m_leftMotor = new TalonFX(1);
    private static TalonFX m_leftFollower = new TalonFX(2);
    private static TalonFX m_rightMotor = new TalonFX(3);
    private static TalonFX m_rightFollower = new TalonFX(4);
    private static TalonFX[] m_motors = { m_leftMotor, m_leftFollower, m_rightMotor, m_rightFollower };

    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0.086571, 1.9726, 0.18671);

    private final double p = 0.23126, i = 0, d = 0;
    private final PIDController m_leftPIDController = new PIDController(p, i, d);
    private final PIDController m_rightPIDController = new PIDController(p, i, d);
  
    private double m_defaultSpeed = 1;

    public Drivetrain() {
        m_rightMotor.setInverted(false);
        m_leftMotor.setInverted(true);
        m_leftFollower.setControl(new Follower(m_leftMotor.getDeviceID(), false));
        m_rightFollower.setControl(new Follower(m_rightMotor.getDeviceID(), false));

        SmartDashboard.putData("Drivebase/Motors/RightLeader", m_rightMotor);
        SmartDashboard.putData("Drivebase/Motors/RightFollower", m_rightFollower);
        SmartDashboard.putData("Drivebase/Motors/LeftLeader", m_leftMotor);
        SmartDashboard.putData("Drivebase/Motors/LeftFollower", m_leftFollower);

        for (var motor : m_motors) {
            motor.getConfigurator().apply(new OpenLoopRampsConfigs());
            motor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    public void reset() {
        resetMaxSpeed();
        for (var motor : m_motors) {
            motor.setPosition(0);
        }
    }

    public void arcadeDrive(double speed, double rotation) {
        m_robotDrive.arcadeDrive(speed, -rotation);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_robotDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void setMaxSpeed(double maxOutput) {
        m_robotDrive.setMaxOutput(maxOutput);
    }

    public void resetMaxSpeed() {
        m_robotDrive.setMaxOutput(m_defaultSpeed);
    }
    
    public SimpleMotorFeedforward getFeedforward() {
        return m_feedForward;
    }

    public PIDController getLeftPIDController() {
        return m_leftPIDController;
    }

    public PIDController getRightPIDController() {
        return m_rightPIDController;
    }

    public TalonFX getLeftMotors() {
      return m_leftMotor;
    }

    public TalonFX getRightMotors() {
      return m_rightMotor;
    }

    public void setOutputVolts(double left, double right) {
        var rightSetpoint = right / 12;
        var leftSetpoint = left / 12;

        SmartDashboard.putNumber("Left Motor Setpoint (in)", leftSetpoint);
        SmartDashboard.putNumber("Right Motor Setpoint (in)", rightSetpoint);

        m_leftMotor.set(leftSetpoint);
        m_rightMotor.set(rightSetpoint);
    }
}
