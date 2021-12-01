// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class DriveTrain extends SubsystemBase {
    private final double gearRatio = 1 / 2.5;

    private final double kS = 0.75;
    private final double kV = 2.2;
    private final double kA = 0.48;

    public double kP = 2;
    public double kI = 0;
    public double kD = 0;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
    DifferentialDriveOdometry odometry;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    PIDController leftPIDController = new PIDController(kP, kI, kD);
    PIDController rightPIDController = new PIDController(kP, kI, kD);

    private final TalonFX[] driveMotors = {
            new TalonFX(Constants.leftFrontDriveMotor),
            new TalonFX(Constants.leftRearDriveMotor),
            new TalonFX(Constants.rightFrontDriveMotor),
            new TalonFX(Constants.rightRearDriveMotor)
    };


    public DriveTrain() {
        driveMotors[0].setInverted(true);
        driveMotors[1].setInverted(true);
        driveMotors[2].setInverted(false);
        driveMotors[3].setInverted(false);

        driveMotors[0].setSensorPhase(false);
        driveMotors[2].setSensorPhase(false);

        driveMotors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
        driveMotors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());
        driveMotors[1].setNeutralMode(NeutralMode.Brake);
        driveMotors[3].setNeutralMode(NeutralMode.Brake);

        odometry = new DifferentialDriveOdometry(new Rotation2d());
    }

    public double getMotorInputCurrent(int motorIndex) {
        return driveMotors[motorIndex].getSupplyCurrent();
    }

    public void setMotorArcadeDrive(double throttle, double turn) {
        double leftPWM = throttle + turn;
        double rightPWM = throttle - turn;

        // Normalization
        double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
        if (magnitude > 1.0) {
            leftPWM *= 1.0 / magnitude;
            rightPWM *= 1.0 / magnitude;
        }

        setMotorPercentOutput(leftPWM, rightPWM);
    }

    public void setMotorTankDrive(double leftOutput, double rightOutput) {
        setMotorPercentOutput(leftOutput, rightOutput);
    }

    private void setMotorPercentOutput(double leftOutput, double rightOutput) {
        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
    }

    public void setVoltageOutput(double leftVoltage, double rightVoltage) {
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage)) > batteryVoltage) {
            leftVoltage *= batteryVoltage / 12.0;
            rightVoltage *= batteryVoltage / 12.0;
        }
        setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);
    }

    public void setDriveTrainNeutralMode(int mode) {
        switch(mode) {
            case 2:
                for(var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Coast);
                break;
            case 1:
                for(var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Brake);
                break;
            case 0:
            default:
                driveMotors[0].setNeutralMode(NeutralMode.Brake);
                driveMotors[1].setNeutralMode(NeutralMode.Coast);
                driveMotors[2].setNeutralMode(NeutralMode.Brake);
                driveMotors[3].setNeutralMode(NeutralMode.Coast);
                break;
        }
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        double leftMetersPerSecond = 0, rightMetersPerSecond = 0;
        
        // getSelectedSensorVelocity() returns values in units per 100ms. Need to convert value to RPS
        leftMetersPerSecond = (driveMotors[0].getSelectedSensorVelocity() * 10.0 / 2048.0) * gearRatio * Math.PI * Units.feetToMeters(0.5);
        rightMetersPerSecond = (driveMotors[2].getSelectedSensorVelocity() * 10.0 / 2048.0) * gearRatio * Math.PI * Units.feetToMeters(0.5);
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    public void resetEncoderCounts() {
        driveMotors[0].setSelectedSensorPosition(0);
        driveMotors[2].setSelectedSensorPosition(0);
    }

    public double getWheelDistanceMeters(int sensorIndex) {
        
        return (driveMotors[sensorIndex].getSelectedSensorPosition() / 4096.0) * gearRatio * Math.PI * Units.feetToMeters(0.5);
    }

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public DifferentialDriveKinematics getDriveTrainKinematics() {
        return kinematics;
    }

    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        odometry.resetPosition(pose, rotation);
        resetEncoderCounts();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(Rotation2d.fromDegrees(0), getWheelDistanceMeters(0), getWheelDistanceMeters(2));
        SmartDashboard.putNumber("Left Wheels", getWheelDistanceMeters(0));
        SmartDashboard.putNumber("Right Wheels", getWheelDistanceMeters(2));
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
