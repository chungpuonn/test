package transforma.qb2_user_app.rosmsg;

import android.util.Log;


import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import ROS.*;

/**
 * Gson object class: A class of a PicaBot specified rosMessage with its "msg" typed as pointCloud, which is a special type of rsoMessage.
 * It contains information of geolocation of tens of thousands of raw scanning points for pointCloud viewer to use
 */


public class RosTopicMessagePointCloud extends RosTopicMessage {


    MsgPointCloud msg;

    public RosTopicMessagePointCloud(String op, String topic, String type) {
        super(op, topic, type);
    }

    public class MsgPointCloud{

        RosTopicMessage_MsgStd_Header header;

        public RosTopicMessage_MsgStd_Header getHeader() {
            return header;
        }

        RosTopicMessage_MsgGeometry_Point[] points;

    }

    /**
     * THis method converts would convert all information in the class and return a float array of all xyz points
     * @return float[] pointcloud
     */
    public float[] toArray(){

        float[] pointcloud = new float[msg.points.length * 3];
        int x = 0;
        //JsonReader reader = new JsonReader(new StringReader(msg.points));
        Gson gson = new GsonBuilder().create();
//        for (RosTopicMessage_MsgGeometry_Point location: msg.points) {
//            pointcloud[x++] = location.x;
//            pointcloud[x++] = location.y;
//            pointcloud[x++] = location.z;
//        }

        return pointcloud;

    }
}
