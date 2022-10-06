package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.ROBOT;
import frc.swervelib.Gyroscope;
import frc.swervelib.GyroscopeHelper;
import frc.swervelib.Mk4SwerveModuleHelper;
import frc.swervelib.SwerveConstants;
import frc.swervelib.SwerveModule;
import frc.swervelib.SwerveSubsystem;
import frc.swervelib.SwerveDrivetrainModel;
import frc.wpiClasses.QuadSwerveSim;

public class BearSwerveHelper {
    public static SwerveDrivetrainModel createBearSwerve() {
        passConstants();
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);

        // There are 4 methods you can call to create your swerve modules.
        // The method you use depends on what motors you are using.
        //
        // Mk3SwerveModuleHelper.createFalcon500(...)
        //   Your module has two Falcon 500s on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createNeo(...)
        //   Your module has two NEOs on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createFalcon500Neo(...)
        //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
        //
        // Mk3SwerveModuleHelper.createNeoFalcon500(...)
        //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
        //
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.
    
        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        // FIXME Setup motor configuration
        SwerveModule m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4SwerveModuleHelper.GearRatio.L1,
                // This is the ID of the drive motor
                DRIVE.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                DRIVE.FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                DRIVE.FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                DRIVE.FRONT_LEFT_MODULE_STEER_OFFSET, "FL"
        );
    
        // We will do the same for the other modules
        SwerveModule m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                DRIVE.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                DRIVE.FRONT_RIGHT_MODULE_STEER_MOTOR,
                DRIVE.FRONT_RIGHT_MODULE_STEER_ENCODER,
                DRIVE.FRONT_RIGHT_MODULE_STEER_OFFSET, "FR"
        );
    
        SwerveModule m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                DRIVE.BACK_LEFT_MODULE_DRIVE_MOTOR,
                DRIVE.BACK_LEFT_MODULE_STEER_MOTOR,
                DRIVE.BACK_LEFT_MODULE_STEER_ENCODER,
                DRIVE.BACK_LEFT_MODULE_STEER_OFFSET, "BL"
        );
    
        SwerveModule m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4SwerveModuleHelper.GearRatio.L1,
                DRIVE.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                DRIVE.BACK_RIGHT_MODULE_STEER_MOTOR,
                DRIVE.BACK_RIGHT_MODULE_STEER_ENCODER,
                DRIVE.BACK_RIGHT_MODULE_STEER_OFFSET, "BR"
        );
    
        Gyroscope gyro = GyroscopeHelper.createPigeonCAN(DRIVE.PIGEON_ID);
    
        modules.add(m_frontLeftModule);
        modules.add(m_frontRightModule);
        modules.add(m_backLeftModule);
        modules.add(m_backRightModule);
        return new SwerveDrivetrainModel(modules, gyro);
    }

    public static SwerveSubsystem createSwerveSubsystem(SwerveDrivetrainModel dt) {
        return new SwerveSubsystem(dt);        
    }

    private static void passConstants() {
        SwerveConstants.MAX_FWD_REV_SPEED_MPS = DRIVE.MAX_FWD_REV_SPEED_MPS;
        SwerveConstants.MAX_VOLTAGE = DRIVE.MAX_VOLTAGE;
        SwerveConstants.DFLT_START_POSE = ROBOT.DFLT_START_POSE;

        SwerveConstants.THETACONTROLLERkP = AUTO.THETACONTROLLERkP;
        SwerveConstants.TRAJECTORYXkP = AUTO.TRAJECTORYXkP;
        SwerveConstants.TRAJECTORYYkP = AUTO.TRAJECTORYYkP;
        SwerveConstants.THETACONTROLLERCONSTRAINTS = AUTO.THETACONTROLLERCONSTRAINTS;

        SwerveConstants.TRACKWIDTH_METERS = DRIVE.TRACKWIDTH_METERS;
        SwerveConstants.TRACKLENGTH_METERS = DRIVE.WHEELBASE_METERS;
        SwerveConstants.MASS_kg = ROBOT.MASS_kg;
        SwerveConstants.MOI_KGM2 = ROBOT.MOI_KGM2;
        SwerveConstants.KINEMATICS = DRIVE.KINEMATICS;
    }
}
