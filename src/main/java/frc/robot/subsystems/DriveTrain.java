// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class DriveTrain extends SubsystemBase {

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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
