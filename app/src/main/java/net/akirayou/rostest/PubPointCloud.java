package net.akirayou.rostest;

import com.google.atap.tangoservice.TangoPointCloudData;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;


/**
 * Created by akira on 17/07/04.
 */

public class PubPointCloud extends AbstractNodeMain {
    private String topic_name;
    private Publisher<sensor_msgs.PointCloud2> publisher;
    private boolean inited=false;
    public PubPointCloud(){
        topic_name="/points";
    }
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rostest/pubpointcloud");
    }
    private ArrayList<PointField> pfl;

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(topic_name, PointCloud2._TYPE);
        publisher.setLatchMode(true);
        pfl=new ArrayList<>(4);
        for(int i=0;i<4;i++) {
            PointField pf=connectedNode.getTopicMessageFactory().newFromType(PointField._TYPE);
            pf.setDatatype(PointField.FLOAT32);
            pf.setCount(1);
            pfl.add(pf);
            pfl.get(i).setOffset(i*4);
        }
        pfl.get(0).setName("x");
        pfl.get(1).setName("y");
        pfl.get(2).setName("z");
        pfl.get(3).setName("c");
        inited=true;

    }

    public void publish(TangoPointCloudData pointCloud){
        if(!inited)return;//Don't run before onStart
        if(!publisher.hasSubscribers())return;

        int nofElement=pointCloud.points.capacity();
        ByteBuffer b=ByteBuffer.allocate(nofElement*4);
        b.order(ByteOrder.LITTLE_ENDIAN);
        b.asFloatBuffer().put(pointCloud.points);


        ChannelBuffer cb= ChannelBuffers.wrappedBuffer(b);
        sensor_msgs.PointCloud2 c=publisher.newMessage();
        c.getHeader().setFrameId("tango_depth_device");
        c.getHeader().setStamp(new Time(pointCloud.timestamp));
        c.setIsDense(false);
        c.setHeight(1);
        c.setWidth(pointCloud.numPoints);
        c.setPointStep(4*4);
        c.setRowStep(4*nofElement);
        c.setFields(pfl);
        c.setData(cb);

        publisher.publish(c);


    }
}
