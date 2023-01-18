package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;

public class ShuffleboardManager {

    private static ShuffleboardTab m_autoConfigTab;

    private static SendableChooser<Constants.PlacePosition> m_preloadedPieceLevelChooser = new SendableChooser<>();
    private static SendableChooser<Constants.PlacePosition> m_stagedPieceLevelChooser = new SendableChooser<>();
    private static SendableChooser<Constants.ConeOffsetPosition> m_preloadedPieceConeOffsetChooser = new SendableChooser<>();
    private static SendableChooser<Constants.ConeOffsetPosition> m_stagedPieceConeOffsetChooser = new SendableChooser<>();

    public ShuffleboardManager(ShuffleboardTab autoConfigTab) {

        m_autoConfigTab = autoConfigTab;
    }

    public void configureShuffleboard() {

        m_preloadedPieceLevelChooser.setDefaultOption("High Cone", Constants.PlacePosition.HighCone);
        m_preloadedPieceLevelChooser.addOption("Middle Cone", Constants.PlacePosition.MiddleCone);
        m_preloadedPieceLevelChooser.addOption("Low Cone", Constants.PlacePosition.LowCone);
        m_preloadedPieceLevelChooser.addOption("High Cube", Constants.PlacePosition.HighCube);
        m_preloadedPieceLevelChooser.addOption("Middle Cube", Constants.PlacePosition.MiddleCube);
        m_preloadedPieceLevelChooser.addOption("Low Cube", Constants.PlacePosition.LowCube);
        m_autoConfigTab.add("Preloaded Piece Level", m_preloadedPieceLevelChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(5, 1)
                .withPosition(0, 0);

        m_preloadedPieceConeOffsetChooser.setDefaultOption("Left", Constants.ConeOffsetPosition.Left);
        m_preloadedPieceConeOffsetChooser.addOption("Right", Constants.ConeOffsetPosition.Right);
        m_autoConfigTab.add("Preloaded Cone Offset", m_preloadedPieceConeOffsetChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(2, 1)
                .withPosition(5, 0);

        m_stagedPieceLevelChooser.setDefaultOption("High Cone", Constants.PlacePosition.HighCone);
        m_stagedPieceLevelChooser.addOption("Middle Cone", Constants.PlacePosition.MiddleCone);
        m_stagedPieceLevelChooser.addOption("Low Cone", Constants.PlacePosition.LowCone);
        m_stagedPieceLevelChooser.addOption("High Cube", Constants.PlacePosition.HighCube);
        m_stagedPieceLevelChooser.addOption("Middle Cube", Constants.PlacePosition.MiddleCube);
        m_stagedPieceLevelChooser.addOption("Low Cube", Constants.PlacePosition.LowCube);
        m_autoConfigTab.add("Staged Piece Level", m_stagedPieceLevelChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(5, 1)
                .withPosition(0, 1);

        m_stagedPieceConeOffsetChooser.setDefaultOption("Left", Constants.ConeOffsetPosition.Left);
        m_stagedPieceConeOffsetChooser.addOption("Right", Constants.ConeOffsetPosition.Right);
        m_autoConfigTab.add("Staged Cone Offset", m_stagedPieceConeOffsetChooser)
                .withWidget(BuiltInWidgets.kSplitButtonChooser)
                .withSize(2, 1)
                .withPosition(5, 1);
    }

    public Constants.PlacePosition getPreloadedPieceLevel() {
        return m_preloadedPieceLevelChooser.getSelected();
    }
    
    public Constants.ConeOffsetPosition getPreloadedConeOffsetPosition() {
        return m_preloadedPieceConeOffsetChooser.getSelected();
    }

    public Constants.PlacePosition getStagedPieceLevel() {
        return m_stagedPieceLevelChooser.getSelected();
    }
    
    public Constants.ConeOffsetPosition getStagedConeOffsetPosition() {
        return m_stagedPieceConeOffsetChooser.getSelected();
    }
}