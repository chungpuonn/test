/**
 * R
 * @author Transforma Robotics  - Lyu Zizheng
 * @use org.java-websocket:Java-WebSocket:1.4.0
 * @use com.google.code.gson:gson:2.8.5
 */

package ROS;

import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.drafts.Draft_6455;
import org.java_websocket.handshake.ServerHandshake;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;

import transforma.qb2_user_app.rosmsg.RosTopicMessageString;

//TODO: Adding a method to automatically detect which bot we are connecting


/**
 * <p>
 *  This is a singleton class that establishes a customised websocket connection with the robot using package JAVA-WEBSOCKET..
 *  It supports the feature of auto offline reconnection, status detectiion.
 *  It has a method to notify all listners when a messgae is received.
 *</p>
 * <p>
 *  Carefully convert string messages to different Message Classes based on message topic or type.
 *  Carefully choose which listener to response based on message received.
 *</p>
 * <p>
 *  Sending RosMessgae in such sequence:.
 *  -Call initSocket.
 *  -Call 1 of 5 methods (advertise, unadvertise, subscribe, publish, call service).
 *  -For publish method, build ur own RosMessage Class object and pass into it.
 *  -Call Gson to serialise the Message Calss.
 *  -Call client to send the Message.
 * </p>
 *
 */
public class RosBridgeConnection {

    private static final String TAG = "RosBridge";
    private static final String ADVERTISE = "advertise";
    private static final String UNADVERTISE = "unadvertise";
    private static final String SUBSCRIBE = "subscribe";
    private static final String PUBLISH = "publish";
    private static final String UNSUBSCRIBE = "unsubscribe";
    private static final String CALL_SERVICE = "call_service";

    /**
     * A singleton instance
     */
    private static RosBridgeConnection rosBridgeConnection;

    private String socketAddr;
    private String imageStreamAddr;
    private URI uri;

    /**
     * This is an list that adds all pages into listeners
     */
    private List<RosBridgeListener> listeners;
    /**
     * This is the WebSocketClient, it is the main core module that in charge of communication, it is public as other class may call reconnect when client disconnects due to some reason
     */
    public WebSocketClient client;


    /**
     * This is the class constructor. Private constructor to prevent more than one class being created
     */
    private RosBridgeConnection(){
        listeners = new ArrayList<>();
    }

    /**
     * This is the core method to obtain the singleton RosBridgeConnection object
     * @return New object if not exist otherwise the only object that exists
     */

    public static RosBridgeConnection getRosBridgeConnection(){
        if (rosBridgeConnection == null){
           rosBridgeConnection = new RosBridgeConnection();
        }
        return rosBridgeConnection;

    }

    /**
     * Important method to initialise the connection
     *
     * @param socketAddr The string address of the websocket
     */

    public void initSocket(String socketAddr){
        this.socketAddr = socketAddr;
        try {
            uri = new URI(socketAddr);
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
        if (null == client){ //If we already have a client profile, we just need to reconnect
            client = new WebSocketClient(uri, new Draft_6455(), null, 1000) {
                @Override
                public void onOpen(ServerHandshake handshakedata) {
                    Log.i(TAG, "onOpen");
                    notifyListerners(true);

                }

                @Override
                public void onMessage(String message) {

                    //Log.i(TAG, "onMessage: " + message);
                    notifyListerners(message);

                }

                @Override
                public void onClose(int code, String reason, boolean remote) {
                    Log.i(TAG, "onClose: ");
                    notifyListerners(false);

                }
                //Called when any kind of error happens including the 1000ms timeout mentioned above
                @Override
                public void onError(Exception ex) {
                    Log.i(TAG, "onError: " + uri + "\n" + ex.getMessage());
                    ex.printStackTrace();
                    notifyListerners(false);

                }
            };
            client.connect();
            client.setConnectionLostTimeout(20);
        }
        else {
            client.reconnect();
            //client
        }
    }

    /**
     * Publish a new topic using msg parameter
     * @param topic Rostopic
     * @param type Rostopic type
     */

    public void advertise (String topic, String type){
        RosTopicMessage msg = new RosTopicMessage(ADVERTISE, topic, type);
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(msg));
        client.send(gson.toJson(msg));
    }



    /**
     * Claiming to publish a new topic/publish new message using RosTopicMessage object
     * @param msg A object of RosTopicMessage or subclass of RosTopicMessage
     */
    public void advertise (RosTopicMessage msg){
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(msg));
        client.send(gson.toJson(msg));
    }


    /**
     * Claiming not to publish any message to a topic any longer using msg parameter
     * @param topic Rostopic
     * @param type Rostopic type
     *
     */
    public void unadvertise (String topic, String type){
        RosTopicMessage msg = new RosTopicMessage(UNADVERTISE, topic, type);
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(msg));
        client.send(gson.toJson(msg));
    }

    /**
     * Claiming not to publish any message to a topic any longer using RosTopicMessage object
     * @param msg A object of RosTopicMessage or subclass of RosTopicMessage
     */

    public void unadvertise (RosTopicMessage msg){
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(msg));
        client.send(gson.toJson(msg));
    }

    /**
     * Subscribe to a topic using msg parameter
     * @param topic Rostopic
     * @param type Rostopic type
     */
    public void subscribe (String topic, String type){
        RosTopicMessage msg = new RosTopicMessage(SUBSCRIBE, topic, type);
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(msg));
        client.send(gson.toJson(msg));
    }

    /**
     * Subscribe to a topic using RosTopicMessage object
     * @param msg A object of RosTopicMessage or subclass of RosTopicMessage
     */

    public void subscribe (RosTopicMessage msg){
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(msg));
        client.send(gson.toJson(msg));
    }

    /**
     * Unsubscribe to a topic using msg parameter
     * @param topic Rostopic
     * @param type Rostopic type
     */
    public void unSubscribe (String topic, String type){
        RosTopicMessage msg = new RosTopicMessage(UNSUBSCRIBE, topic, type);
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(msg));
        client.send(gson.toJson(msg));
    }

    /**
     * Unsubscribe to a topic using RosTopicMessage object
     * @param topicMessage A object of RosTopicMessage or subclass of RosTopicMessage
     */

    public void ubsubscribe (RosTopicMessage topicMessage){
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(topicMessage));
        client.send(gson.toJson(topicMessage));
    }

    /**
     * Publish a message to a topic using RosTopicMessage object
     * @param topicMessage A object of RosTopicMessage or subclass of RosTopicMessage
     */
    public void publish (RosTopicMessage topicMessage){
        Gson gson = new GsonBuilder().create();
        Log.i("Sending Message",gson.toJson(topicMessage));
        client.send(gson.toJson(topicMessage));
    }




    public void callService (RosServiceMessage srvMessage){
        Gson gson = new GsonBuilder().create();
        Log.i("Service Call:", gson.toJson(srvMessage));
        client.send(gson.toJson(srvMessage));
    }

    /**
     * Add the page listerners to the class
     * @param listener
     */
    public void addListeners(RosBridgeListener listener, String tag) {
        if(!listeners.contains(listener)){
            listeners.add(listener);
            Log.i("Listners added:", tag);
        }else{
            Log.i("Listener adding failed:", "Listener already exist");
        }
    }

    /**
     * Remove the page listener from the class
     * @param listener
     */
    public void removeListeners(RosBridgeListener listener){
        try {
            this.listeners.remove(listener);
        }catch (Exception ex){
            Log.i("Listener doesn't Exist", ex.getMessage());
        }
    }

    /**
     * This method is a core method to be called when the rosBridge receive a message
     *
     * @param message message string
     */


    private synchronized void notifyListerners(final String message){
        RosMsgRaw rosMsgRaw = new Gson().fromJson(message, RosMsgRaw.class);
        for (final RosBridgeListener existingListener : listeners) {
            existingListener.onMessage(message, rosMsgRaw);
        }
    }
    private synchronized void notifyListerners(Boolean bool){
        for (RosBridgeListener existingListener : listeners) {
            existingListener.onMessage(bool);
        }
    }

    /**
     * This method gets the current set sorWebServer streaming address
     * @return the http address string
     */

    public String getImageStreamAddr() {
        return imageStreamAddr;
    }

    /**
     * This method sets the rosWebServer address
     * @param imageStreamAddr address string
     */

    public void setImageStreamAddr(String imageStreamAddr) {
        this.imageStreamAddr = imageStreamAddr;
    }

    /**
     * Check whether the connection is closed
     * @return If there is no webSocketClient, return true, otherwise return connection status
     */


    public boolean isClosed(){
        if (client == null)
            return true;
        return client.isClosed();
    }

    /**
     * Check whether the connection is open
     * @return If there is no webSocketClient, return false, otherwise return connection status
     */

    public boolean isOpen(){
        if (client == null)
            return false;
        return client.isOpen();
    }

    /**
     * Close webSocketClient connection with rosBridge
     * @return True on successfully close
     */

    public boolean close(){
        if (client == null)
            return false;
        client.close();
        return true;
    }


    /**
     * Reconnect to the same client when connection address does not change
     * @return
     */

    public boolean reconnect(){
        if (client == null)
            return false;
        client.reconnect();
        return true;
    }



}


