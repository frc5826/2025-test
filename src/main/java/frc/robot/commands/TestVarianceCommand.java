package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.localization.Localization;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Function;

public class TestVarianceCommand extends LoggedCommand {

    private double wheelVel;

    private final SwerveSubsystem swerveSubsystem;

    private List<Double> speeds;

    private int counter;

    private Function<SwerveSubsystem, Double> speedSupplier;

    private Function<Double, ChassisSpeeds> speedProvider;

    public TestVarianceCommand(double wheelVel, Function<SwerveSubsystem,
            Double> speedSupplier, Function<Double, ChassisSpeeds> speedProvider,
                               SwerveSubsystem swerveSubsystem) {

        this.speedProvider = speedProvider;
        this.speedSupplier = speedSupplier;

        this.wheelVel = wheelVel;

        this.swerveSubsystem = swerveSubsystem;

        speeds = new ArrayList<>();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        speeds.clear();
    }

    @Override
    public void execute() {
        swerveSubsystem.driveFieldOriented(speedProvider.apply(wheelVel));

        speeds.add(speedSupplier.apply(swerveSubsystem));

        if (counter++ % 50 == 0) {
            final double mean = speeds.stream().mapToDouble(d -> d).average().getAsDouble();
            double squared = speeds.stream().mapToDouble(d -> Math.pow(d - mean, 2)).sum();
            double variance = squared / speeds.size();

            //System.out.println(variance);

            if (speeds.size() > 10000) {
                speeds.clear();
            }
        }
    }
}
