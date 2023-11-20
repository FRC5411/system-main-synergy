package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface ModuleIO {
    
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionM = 0.0;
        public double driveVelocityMPS = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveStatorCurrentAMP = 0.0;

        public double azimuthAbsolutePositionRAD = 0.0;
        public double azimuthPositionRAD = 0.0;
        public double azimuthVelocityRPS = 0.0;
        public double azimuthAppliedVolts = 0.0;
        public double azimuthStatorCurrentAMP = 0.0;
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVolts(double volts) {}

    public default void setDriveVelocity(SwerveModuleState desiredState) {}

    public default void setAzimuthVolts(double volts) {}

    public default void setAzimuthAngle(SwerveModuleState desiredState) {}

    public default void setDesiredStates(SwerveModuleState desiredState) {}

    public default void stopMotors() {
        setDriveVolts(0);
        setAzimuthVolts(0);
    }

    public default SwerveModuleState getState() {
        return new SwerveModuleState(
            ModuleIOInputs.driveVelocityMPS, 
            new Rotation2d(ModuleIOInputs.azimuthPositionRAD)
        );
    }

    public default SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public default SwerveModuleState getDesiredState() {
        return new SwerveModuleState();
    }
}
