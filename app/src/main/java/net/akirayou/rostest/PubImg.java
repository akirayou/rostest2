package net.akirayou.rostest;

import android.util.Log;

import com.google.atap.tangoservice.experimental.TangoImageBuffer;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
;import java.nio.ByteOrder;

/**
 * Created by akira on 17/07/30.
 */

public class PubImg extends AbstractNodeMain {
    private String topic_name;
    private Publisher<sensor_msgs.Image> publisher;
    private static final String TAG="pubImg";
    public ChannelBuffer  cb;
    public PubImg(String topic_name){
        this.topic_name=topic_name;
        cb= ChannelBuffers.dynamicBuffer(ByteOrder.LITTLE_ENDIAN,1920*1080*2);
        cb.writeZero(1920*1080*2);//dummy write
        cb.resetWriterIndex();
    }
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rostest2/image");
    }
    private boolean inited = false;
    private sensor_msgs.Image img;

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        publisher = connectedNode.newPublisher(topic_name, sensor_msgs.Image._TYPE);
        img = publisher.newMessage();
        img.setEncoding("mono8");
        img.getHeader().setFrameId("tango_device");
        inited=true;
    }
    public boolean hasSubscribers(){
        if(!inited)return false;
        return publisher.hasSubscribers();
    }
    public void kick(TangoImageBuffer  buffer){
        if(!inited)return;//Don't run before onStart
        final int skip=4;

        Log.e(TAG,"============capacity "+String.valueOf(buffer.data.capacity())+"width"+String.valueOf(buffer.width)+"height"+String.valueOf(buffer.height) );
        //output format is UYVY
        int inU,inV,inUVStep;
        int nofPix=buffer.width*buffer.height;
        //cb.ensureWritableBytes(nofPix*2);
        cb.ensureWritableBytes(nofPix/skip/skip);
        switch(buffer.format){
            case 17/*buffer.YCRCB_420_SP*/: //NV21
                inV = nofPix;
                inU = inV + 1;
                inUVStep = 2;
                break;
            case 842094169/*buffer.YV12*/://YV12
                inU = nofPix;
                inV = inU +  nofPix/4;
                inUVStep = 1;
                break;
            default:
                Log.e(TAG,"Unknown format ");
                return;
        }

        //copy Y
        Log.i(TAG,"===============copyY"+String.valueOf(nofPix));
        buffer.data.position(0);
        if(skip==1) {
            ChannelBuffer cb_in = ChannelBuffers.wrappedBuffer(buffer.data);
            cb.clear();
            cb.writeBytes(cb_in, nofPix);
            cb_in = null;
        }else {
            cb.clear();
            for(int y=0;y<buffer.height;y+=skip) {
                int pos=y*buffer.width;
                for (int x = 0; x < buffer.width; x += skip) {

                    cb.writeByte(buffer.data.get(pos));
                    pos+=skip;
                }
            }
        }


        /*
        int outPosU1=0;
        int outPosV1=2;
        int outPosU2=buffer.width*2;
        int outPosV2=outPosU2+2;
        Log.i(TAG,"===============copy Color");

        //copy U,V
        for(int y=0;y<buffer.height/2;y++) {
            for(int x=0;x<buffer.width/2;x++) {
                //copy to (x,y) and (x,y+1)
                byte u=cb_in.getByte(inU);
                byte v=cb_in.getByte(inV);

                cb.setByte(outPosU1,u);
                cb.setByte(outPosV1,v);
                cb.setByte(outPosU2,u);
                cb.setByte(outPosV2,v);

                outPosU1+=4;
                outPosV1+=4;
                outPosU2+=4;
                outPosV2+=4;
                inU+=inUVStep;
                inV+=inUVStep;
            }
        }
        */


        img.getHeader().setStamp(new Time(buffer.timestamp));
        img.setHeight(buffer.height/skip);
        img.setWidth(buffer.width/skip);
        //img.setEncoding("yuv422");
        //img.setStep(buffer.width*2);
        img.setStep(buffer.width/skip);
        img.setData(cb);
        Log.i(TAG,"===============publish"+String.valueOf(img.getData().readableBytes())+"/"+String.valueOf(cb.readableBytes()));
        publisher.publish(img);
    }

}
