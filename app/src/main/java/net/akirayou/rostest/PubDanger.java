package net.akirayou.rostest;

import org.ros.concurrent.CancellableLoop;
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
    private boolean inited=false;
    private short dangerLevel=0x7fff;


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
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void setup() {
            }
            @Override
            protected void loop() throws InterruptedException {
                std_msgs.UInt16 data = publisher.newMessage();
                data.setData(dangerLevel);
                publisher.publish(data);
                Thread.sleep(200);
            }
        });
    }

    public void kick(short in){
        dangerLevel=in;
    }

}
