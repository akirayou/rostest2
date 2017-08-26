package net.akirayou.rostest;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.UInt16;

/**
 * Created by akira on 17/08/27.
 * Danger flag via point cloud
 */

class PubDanger extends AbstractNodeMain {
    private String topic_name;
    private Publisher<UInt16> publisher;
    private int count=0;
    private boolean inited=false;

    public PubDanger(){
        topic_name="/danger_level";
    }
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rostest2/danderDetect");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(topic_name, std_msgs.UInt16._TYPE);
        publisher.setLatchMode(true);
        inited=true;
    }

    public void kick(short dangerLevel){
        if(!inited)return;//Don't run before onStart
        count++;
        std_msgs.UInt16 data = publisher.newMessage();
        data.setData(dangerLevel);
        publisher.publish(data);
    }

}
