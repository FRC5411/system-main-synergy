package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;

import static frc.robot.subsystems.drive.DriveVars.DriveConstants.*;

public class ModuleSim implements ModuleIO{
    private FlywheelSim driveMotor;
    private FlywheelSim azimuthMotor;
    private DCMotor driveMotorModel;
    private DCMotor azimuthMotorModel;
    private PIDController angleController;
    private double kF;
    private SwerveModuleState desiredState = new SwerveModuleState();

    public ModuleSim(
        FlywheelSim driveMotor, FlywheelSim azimuthMotor, DCMotor driveMotorModel, DCMotor azimuthMotorModel,
        PIDController driveController, PIDController angleController, double kF) {
        this.driveMotor = driveMotor;
        this.azimuthMotor = azimuthMotor;
        this.driveMotorModel = driveMotorModel;
        this.azimuthMotorModel = azimuthMotorModel;
        this.angleController = angleController;
        this.kF = kF;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVelocityMPS = getState().speedMetersPerSecond;
        inputs.drivePositionM += inputs.driveVelocityMPS * 0.02;
        inputs.driveStatorCurrentAMP = driveMotor.getCurrentDrawAmps();
        inputs.driveAppliedVolts = driveMotorModel.getVoltage(
            driveMotorModel.getTorque(inputs.driveStatorCurrentAMP), inputs.driveVelocityMPS);
        
        inputs.azimuthVelocityRPS = azimuthMotor.getAngularVelocityRadPerSec();
        inputs.azimuthAbsolutePositionRAD += inputs.azimuthVelocityRPS * 0.02;
        inputs.azimuthPositionRAD += inputs.azimuthVelocityRPS * 0.02;
        inputs.azimuthStatorCurrentAMP = azimuthMotor.getCurrentDrawAmps();
        inputs.azimuthAppliedVolts = driveMotorModel.getVoltage(
            azimuthMotorModel.getTorque(inputs.azimuthStatorCurrentAMP), inputs.azimuthVelocityRPS);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
        setDriveVelocity(desiredState);
        setAzimuthAngle(desiredState);
    }

    @Override
    public void setDriveVelocity(SwerveModuleState desiredState) {}

    @Override
    public void setAzimuthAngle(SwerveModuleState desiredState) {
        double angleDegrees = 
            Math.abs(desiredState.speedMetersPerSecond) <= (MAX_LINEAR_SPEED * 0.01) 
                ? angleEncoder.getAbsolutePosition() 
                : desiredState.angle.getDegrees();

        setAzimuthVolts(
            12 * MathUtil.clamp(
                angleController.calculate(angleEncoder.getAbsolutePosition(), angleDegrees) 
                + kF * Math.signum(angleController.getPositionError()), -1, 1) );
    }

    @Override
    public void setDriveVolts(double volts) {
        driveMotor.setInputVoltage(volts);
        driveMotor.update(0.02);
    }

    @Override
    public void setAzimuthVolts(double volts) {
        azimuthMotor.setInputVoltage(volts);
        azimuthMotor.update(0.02);
    }
}