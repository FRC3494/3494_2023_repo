package com.swervedrivespecialties.swervelib;

import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.rev.NeoDriveControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.NeoSteerConfiguration;
import com.swervedrivespecialties.swervelib.rev.NeoSteerControllerFactoryBuilder;
import com.swervedrivespecialties.swervelib.rev.SparkMAXEncoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.rev.SparkMAXEncoderFactoryBuilder;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class Mk4SwerveModuleHelper {
    private Mk4SwerveModuleHelper() {
    }
    
    private static DriveControllerFactory<?, Integer> getNeoDriveFactory(Mk4ModuleConfiguration configuration) {
        return new NeoDriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }

    private static SteerControllerFactory<?, NeoSteerConfiguration<SparkMAXEncoderAbsoluteConfiguration>> getNeoSteerFactory(Mk4ModuleConfiguration configuration) {
        return new NeoSteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(1.0, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build((CANSparkMax parentMotorController) -> new SparkMAXEncoderFactoryBuilder().build(parentMotorController));
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new NeoSteerConfiguration<>(
                        steerMotorPort,
                        new SparkMAXEncoderAbsoluteConfiguration(steerEncoderPort, gearRatio.getConfiguration().isSteerEncoderInverted())
                )
        );
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort
    ) {
        return createNeo(container, new Mk4ModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort);
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort
    ) {
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                driveMotorPort,
                new NeoSteerConfiguration<>(
                        steerMotorPort,
                        new SparkMAXEncoderAbsoluteConfiguration(steerEncoderPort, gearRatio.getConfiguration().isSteerEncoderInverted())
                )
        );
    }

    /**
     * Creates a Mk4 swerve module that uses NEOs for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        return createNeo(new Mk4ModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort);
    }

    public enum GearRatio {
        L1(SdsModuleConfigurations.MK4_L1),
        L2(SdsModuleConfigurations.MK4_L2),
        L3(SdsModuleConfigurations.MK4_L3),
        L4(SdsModuleConfigurations.MK4_L4);

        private final ModuleConfiguration configuration;

        GearRatio(ModuleConfiguration configuration) {
            this.configuration = configuration;
        }

        public ModuleConfiguration getConfiguration() {
            return configuration;
        }
    }
}
