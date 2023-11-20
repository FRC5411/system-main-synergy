package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import frc.robot.Utils.FrostConfigs;

import static frc.robot.subsystems.drive.DriveVars.DriveConstants.*;

public class ForReference {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private WPI_CANCoder angleEncoder;
    private PIDController angleController;
    private double kF;
    private SwerveModuleState desiredState = new SwerveModuleState();
    private String key;

    public ForReference(
        int driveMotorID, int angleMotorID, int angleEncoderID, double offset,
        PIDController angleController, double kF, String key) {
        this(
            new WPI_TalonFX(driveMotorID), 
            new WPI_TalonFX(angleMotorID), 
            new WPI_CANCoder(angleEncoderID), 
            angleController, kF, key );
        FrostConfigs.configPID(angleController);
        FrostConfigs.configPosition(this.angleEncoder, offset);
        FrostConfigs.configAzimuth(
            this.angleMotor, this.angleEncoder, 
            angleController.getP(), angleController.getD(), kF);
        FrostConfigs.configDrive(this.driveMotor);
    }

    public ForReference(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, WPI_CANCoder angleEncoder, 
                        PIDController angleController, double kF, String key) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.angleController = angleController;
        this.kF = kF;
        this.key = key;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
        setAngle(desiredState);
        setVelocity(desiredState);
    }

    public void setLockedState(SwerveModuleState lockedState) {
        this.desiredState = lockedState;
        lockedState = SwerveModuleState.optimize(lockedState, Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
        lockAngle(lockedState.angle.getDegrees());
        setVelocity(lockedState);
    }

    public void setAngle(SwerveModuleState desiredState) {
        double angleDegrees = 
            Math.abs(desiredState.speedMetersPerSecond) <= (MAX_LINEAR_SPEED * 0.01) 
            ? angleEncoder.getAbsolutePosition() : desiredState.angle.getDegrees();
        lockAngle(angleDegrees);
    }

    public void setVelocity(SwerveModuleState desiredState) {
        driveMotor.set(
            ControlMode.Velocity, 
            (desiredState.speedMetersPerSecond * DRIVE_GEAR_RATIO
            / (Math.PI * WHEEL_DIAMETER_METERS) * 2048) / 10);
    }

    public void lockAngle(double angleDegrees) {
        angleMotor.setVoltage(
            12 * MathUtil.clamp(
                angleController.calculate(angleEncoder.getAbsolutePosition(), angleDegrees) 
                + kF * Math.signum( angleController.getPositionError() ), -1, 1) );
    }

    public void stopMotors() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        angleMotor.set(ControlMode.PercentOutput, 0);
    }

    public SwerveModuleState geDesiredState() {
        return desiredState;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            (driveMotor.getSelectedSensorVelocity() * 10 * WHEEL_PERIMETER_METERS)
            / (2048 * DRIVE_GEAR_RATIO), 
            Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            0.94 * ((driveMotor.getSelectedSensorPosition() * WHEEL_PERIMETER_METERS)
            / (2048 * DRIVE_GEAR_RATIO)), 
            Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    public static double degreesToFalcon(double degrees) {
        return degrees / (360.0 / ( AZIMUTH_GEAR_RATIO * 2048.0));
    }
}