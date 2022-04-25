package transforma.qb2_user_app;

/**
 * This class contains all configuration parameters for the app
 */
public class ConfigurationParameters

{
    //Map view Parameters:

    public static final float MAPVIEWRANGE = 4f;  //Initial Range is 4*3 meters height and automatic width according to screen density
    public static final float INITIAL_X = 10f; //Initial Position of the map
    public static final float INITIAL_Y = 10f;
    public static final int DEFINITION_X = 400; //Size of the data[] grid transferred
    public static final int DEFINITION_Y = 400;
    public static final float PIXELRESOLUTION = 0.05f; // 5cm per pixel detected by robot

    public static final double VEL_MAXLINEAR = 0.22; //The maximum linear velocity of the robot moving system
    public static final double VEL_MAXANGULAR = 2.84; //The maximun rotation velocity of the robot moving system

    public static final String mapFileName =  "map.txt"; //The maximun rotation velocity of the robot moving system; //The maximun rotation velocity of the robot moving system


    //Scanning Point Cloud Parameters:

    /**
     * <li>This number is used to find out exactly how many points the point cloud information from the raw message string contains</li>
     * <li>This 25.8 is a rough number form about tens of sample data string.</li>
     * <li>This number cant be larger as it would lead to insufficient buffer size allocated to points</li>
     * <li>Smaller number would lead to unused buffer as points are stored in buffer for optimising running</li>
     */

    public static final float DIVIDING_FACTOR = 25.8f;  //This is the
    //To adjust initial viewing range, you can adjust in the actually java file but not recommended
    //To adjust poind cloud size and color, you need to adjust in res/raw/point_vertex_shader & ../point_fragment_shader



    //Company Facts
    public static final String tel = "tel: +6565702669";
    public static final String email = "mailto:contact@transformarobotics.com";


    //Other configuration parameters rest inside rosBridgeConnection Interface

}
