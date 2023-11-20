package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;

import static frc.robot.subsystems.drive.DriveVars.DriveConstants.*;

public class ModuleTalonFX implements ModuleIO{
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX azimuthMotor;
    private WPI_CANCoder angleEncoder;
    private PIDController angleController;
    private double kF;
    private SwerveModuleState desiredState = new SwerveModuleState();

    public ModuleTalonFX(
        int driveMotorID, int angleMotorID, int angleEncoderID, PIDController angleController, double kF) {
        this(
            new WPI_TalonFX(driveMotorID), 
            new WPI_TalonFX(angleMotorID), 
            new WPI_CANCoder(angleEncoderID),
            angleController, kF);
    }

    public ModuleTalonFX(
        WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, WPI_CANCoder angleEncoder, 
        PIDController angleController, double kF) {
        this.driveMotor = driveMotor;
        this.azimuthMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.angleController = angleController;
        this.kF = kF;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionM = getPosition().distanceMeters;
        inputs.driveVelocityMPS = getState().speedMetersPerSecond;
        inputs.driveAppliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.driveStatorCurrentAMP = driveMotor.getStatorCurrent();
        
        inputs.azimuthAbsolutePositionRAD = getPosition().angle.getRadians();
        inputs.azimuthPositionRAD = 
            2.0 * Math.PI * azimuthMotor.getSelectedSensorPosition() / (2048.0 * AZIMUTH_GEAR_RATIO);
        inputs.azimuthVelocityRPS = 
            10.0 * 2.0 * Math.PI * azimuthMotor.getSelectedSensorVelocity() / (2048.0 * AZIMUTH_GEAR_RATIO);
        inputs.azimuthAppliedVolts = azimuthMotor.getMotorOutputVoltage();
        inputs.azimuthStatorCurrentAMP = azimuthMotor.getStatorCurrent();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
        setDriveVelocity(desiredState);
        setAzimuthAngle(desiredState);
    }

    @Override
    public void setDriveVelocity(SwerveModuleState desiredState) {
        driveMotor.set(
            ControlMode.Velocity, 
            (desiredState.speedMetersPerSecond * DRIVE_GEAR_RATIO
            / (Math.PI * WHEEL_DIAMETER_METERS) * 2048) / 10 );
    }

    @Override
    public void setAzimuthAngle(SwerveModuleState desiredState) {
        double angleDegrees = 
            Math.abs(desiredState.speedMetersPerSecond) <= (MAX_LINEAR_SPEED * 0.01) 
            ? angleEncoder.getAbsolutePosition() : desiredState.angle.getDegrees();

        setAzimuthVolts(
            12 * MathUtil.clamp(
                angleController.calculate(angleEncoder.getAbsolutePosition(), angleDegrees) 
                + kF * Math.signum(angleController.getPositionError()), -1, 1) );
    }

    @Override
    public void setDriveVolts(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setAzimuthVolts(double volts) {
        azimuthMotor.setVoltage(volts);
    }

    @Override
    public void stopMotors() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        azimuthMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            (driveMotor.getSelectedSensorVelocity() * 10 * WHEEL_PERIMETER_METERS)
            / (2048 * DRIVE_GEAR_RATIO), 
            Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            0.94 * ((driveMotor.getSelectedSensorPosition() * WHEEL_PERIMETER_METERS)
            / (2048 * DRIVE_GEAR_RATIO)), 
            Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
    }
}