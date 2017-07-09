package net.akirayou.rostest;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;


/**
 * Created by akira on 17/07/04.
 */

public class TestTalker extends AbstractNodeMain {
    private String topic_name;
    private Publisher<std_msgs.String> publisher;
    private int count=0;
    private boolean inited=false;
    public TestTalker(){
        topic_name="/chatter";
    }
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava_tutorial_pubsub/talker");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(topic_name, std_msgs.String._TYPE);
/*In cace of sending data async to UI thread (periodic send)
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void setup() {
                //do nothing
            }
            @Override
            protected void loop() throws InterruptedException {
                //do
                count++;
                std_msgs.String str = publisher.newMessage();
                str.setData("Hello world! " + count);
                publisher.publish(str);
                Thread.sleep(1000);
            }
        });*/
        inited=true;
    }

    public void kick(){
        if(!inited)return;//Don't run before onStart

        /* send message*/
        count++;
        std_msgs.String str = publisher.newMessage();
        str.setData("Hello world! " + count);

        publisher.publish(str);
    }
}
