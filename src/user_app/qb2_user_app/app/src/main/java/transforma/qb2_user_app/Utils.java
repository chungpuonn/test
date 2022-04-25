package transforma.qb2_user_app;

import android.content.Context;
import android.content.Intent;
import android.net.Uri;
import android.os.Vibrator;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

import static android.content.Context.MODE_PRIVATE;

/**
 * This is a util class for storing common methods
 */

public class Utils {





    /**
     * Call this method to virbrate the device
     * @param context the activity context need for vibration
     */


    public static void deviceVibrate (Context context){
        Vibrator v = (Vibrator) context.getSystemService(Context.VIBRATOR_SERVICE);
        v.vibrate(300);
    }

    /**
     * Read a file from t device internal storage
     * @param context activity context
     * @param fileName the name of the file (eg /doc/info.txt)
     * @return
     */

    public static String readFile(Context context, String fileName){
        try {
            BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(context.openFileInput(fileName)));
             return bufferedReader.readLine();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    /**
     * Write a file to device internal storage, will be erased when app uninstall
     * @param context activity context
     * @param data The data string to be written
     * @param fileName The path to the file (eg. "/doc/info.txt")
     */

    public static void writeFIle(Context context, String data, String fileName){
        try {
            OutputStreamWriter osw = new OutputStreamWriter(context.openFileOutput(fileName, MODE_PRIVATE));
            osw.write(data);
            osw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     *  Generate a dialer dialing attempt
     * @param context activity context
     * @param phoneNumber phone number in string
     */

    public static void phoneCall(Context context, String phoneNumber){
        Intent intent = new Intent(Intent.ACTION_DIAL);
        intent.setData(Uri.parse(phoneNumber));
        context.startActivity(intent);
    }

    /**
     * Generate a sending email attempt to our company email
     * @param context activity context
     * @param emailAddress email string
     */

    public static void sendEmail(Context context, String emailAddress){
        Intent intent = new Intent(Intent.ACTION_SENDTO);
        intent.setData(Uri.parse(emailAddress));
        context.startActivity(intent);
    }




}
