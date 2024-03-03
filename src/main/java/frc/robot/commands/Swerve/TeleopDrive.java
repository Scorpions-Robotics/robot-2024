package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class TeleopDrive extends Command {

    private SwerveBase s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier speedCutoffSup;
    private Boolean speedCutoffVal = false;

    public TeleopDrive(
            SwerveBase s_Swerve,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier speedCutoffSup,
            BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.speedCutoffSup = speedCutoffSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.OIConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.OIConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.OIConstants.stickDeadband);
        speedCutoffVal = speedCutoffSup.getAsBoolean() ? !speedCutoffVal : speedCutoffVal;
        SmartDashboard.putNumber("translationVal", translationVal);
        SmartDashboard.putNumber("strafeVal", strafeVal);
        SmartDashboard.putNumber("rotationVal", rotationVal);
        SmartDashboard.putBoolean("Speed Cut Off", speedCutoffVal);

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times( speedCutoffVal ? 0.5 : 1),
                rotationVal * Constants.Swerve.maxAngularVelocity * (speedCutoffVal ? 0.5 : 1),
                !robotCentricSup.getAsBoolean(),
                true
        );
    }
}
