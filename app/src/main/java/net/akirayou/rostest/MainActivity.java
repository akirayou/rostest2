package net.akirayou.rostest;

import android.os.Bundle;
import android.view.View;

import org.ros.android.AppCompatRosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

//import for ROS
//import for button



public class MainActivity extends AppCompatRosActivity {
    private TestTalker talker;
    public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("rostest2", "rostest2");


    }
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        findViewById(R.id.bt_kick).setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                talker.kick();
            }
        });
    }
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());

        talker=new TestTalker();
        nodeMainExecutor.execute(talker, nodeConfiguration);
    }
}
