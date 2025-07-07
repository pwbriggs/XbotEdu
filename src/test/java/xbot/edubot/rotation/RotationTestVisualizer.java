package xbot.edubot.rotation;

import java.awt.EventQueue;
import java.awt.FlowLayout;
import java.awt.event.ItemEvent;

import javax.swing.BoxLayout;
import javax.swing.DefaultComboBoxModel;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JSplitPane;

import xbot.edubot.rotation.BaseOrientationEngineTest.RotationEnvironmentState;

public class RotationTestVisualizer {
    private JFrame frmOrientationTestVisualizer;
    private RotationVisualizationPanel vizPanel;
    BaseOrientationEngineTest currentTestEnvironment;
    RotationEnvironmentState envState = new RotationEnvironmentState();
    private JPanel controlPanel;
    private JComboBox<OrientationTest> testSelectionBox;
    private JSlider speedSlider;

    /**
     * Launch the application.
     */
    public static void main(String[] args) {
        EventQueue.invokeLater(new Runnable() {
            public void run() {
                try {
                    RotationTestVisualizer window = new RotationTestVisualizer();
                    window.frmOrientationTestVisualizer.setVisible(true);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });
    }

    /**
     * Create the application.
     */
    public RotationTestVisualizer() {
        initialize();
    }
    
    /**
     * Initialize the contents of the frame.
     */
    private void initialize() {
        frmOrientationTestVisualizer = new JFrame();
        frmOrientationTestVisualizer.setTitle("Orientation test visualizer");
        frmOrientationTestVisualizer.setBounds(100, 100, 600, 400);
        frmOrientationTestVisualizer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frmOrientationTestVisualizer.getContentPane().setLayout(new BoxLayout(frmOrientationTestVisualizer.getContentPane(), BoxLayout.Y_AXIS));
        
        JSplitPane splitPane = new JSplitPane();
        frmOrientationTestVisualizer.getContentPane().add(splitPane);
        
        vizPanel = new RotationVisualizationPanel(400, 300);
        splitPane.setLeftComponent(vizPanel);
        
        controlPanel = new JPanel();
        splitPane.setRightComponent(controlPanel);
        
        testSelectionBox = new JComboBox<OrientationTest>();
        testSelectionBox.addItemListener((e) -> {
            if(e.getStateChange() == ItemEvent.SELECTED) {
                setOrientationTest((OrientationTest)e.getItem());
            }
        });
        
        controlPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 5));
        testSelectionBox.setModel(new DefaultComboBoxModel<OrientationTest>(OrientationTest.values()));
        testSelectionBox.setSelectedIndex(0);
        controlPanel.add(testSelectionBox);
        
        speedSlider = new JSlider();
        speedSlider.setMinimum(1);
        speedSlider.setValue(10);
        controlPanel.add(speedSlider);
        
        vizPanel.updateState(envState);
        
        setOrientationTest((OrientationTest)testSelectionBox.getSelectedItem());
    }
    
    private void setOrientationTest(OrientationTest test) {
        if(currentTestEnvironment != null) {
            currentTestEnvironment.stopTestEnv();
        }
        
        switch(test) {
            case GO_LEFT_90_FROM_0:
            case GO_LEFT_90_FROM_150:
            case GO_LEFT_90_FROM_NEG_150:
            case GO_LEFT_90_FROM_NEG_90:
                currentTestEnvironment = new TurnLeft90DegreesCommandTest();
                break;
            case ROTATE_TO_ORIENTATION_0_150:
            case ROTATE_TO_ORIENTATION_0_NEG_150:
            case ROTATE_TO_ORIENTATION_NEG_90_150:
                currentTestEnvironment = new DriveToOrientationTest();
                break;
            default:
                break;
        }
        
        currentTestEnvironment.setUp();
        
        currentTestEnvironment.setAsAsync((BaseOrientationEngineTest.RotationEnvironmentState envState) -> {
            this.envState = envState;
            vizPanel.updateState(envState);
            vizPanel.repaint();
            
            if(envState.isCommandFinished) {
                currentTestEnvironment.stopTestEnv();
            }
            
            // The slider makes the engine do more physics
            currentTestEnvironment.setAsyncPeriodMultiplier(10d / speedSlider.getValue());
        });
        
        ((SelectableOrientationTest) currentTestEnvironment).invokeOrientationTest(test);
        
    }

    public static enum OrientationTest {
        GO_LEFT_90_FROM_0,
        GO_LEFT_90_FROM_NEG_90,
        GO_LEFT_90_FROM_NEG_150,
        GO_LEFT_90_FROM_150,
        ROTATE_TO_ORIENTATION_0_150,
        ROTATE_TO_ORIENTATION_0_NEG_150,
        ROTATE_TO_ORIENTATION_NEG_90_150,
    }
    
    public static interface SelectableOrientationTest {
        void invokeOrientationTest(OrientationTest test);
    }
}
