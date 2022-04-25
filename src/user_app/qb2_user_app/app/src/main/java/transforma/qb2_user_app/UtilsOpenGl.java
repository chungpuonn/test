package transforma.qb2_user_app;

import javax.microedition.khronos.opengles.GL10;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.opengl.GLES32;
import android.opengl.GLSurfaceView;
import android.opengl.GLUtils;
import android.util.Log;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

/**
 * This is a util class that store static method for OpenGl functionality
 */

public class UtilsOpenGl {

    /**
     * This methos is a generic method for loading textures to a Glsurfaceview,
     * however, configs need to be adjusted when using more complex texturing. Configurations can be passed into the method parameter
     * @param activity
     * @param imgResID
     * @return
     */

    public static int LoadTexture(Activity activity, int imgResID){
        Log.d("UtilsOpenGl", "Loadtexture");
        Bitmap img = null;
        int textures[] = new int[1];
        try {
            img = BitmapFactory.decodeResource(activity.getResources(), imgResID);
            GLES32.glGenTextures(1, textures, 0);
            GLES32.glBindTexture(GLES32.GL_TEXTURE_2D, textures[0]);
            GLES32.glTexParameterf(GLES32.GL_TEXTURE_2D, GLES32.GL_TEXTURE_MAG_FILTER, GLES32.GL_LINEAR);
            GLES32.glTexParameterf(GLES32.GL_TEXTURE_2D, GLES32.GL_TEXTURE_MIN_FILTER, GLES32.GL_LINEAR);
            GLUtils.texImage2D(GLES32.GL_TEXTURE_2D, 0, img, 0);
            Log.d("LoadTexture", "Loaded texture"+":H:"+img.getHeight()+":W:"+img.getWidth());
        } catch (Exception e){
            Log.d("LoadTexture", e.toString()+ ":" + e.getMessage()+":"+e.getLocalizedMessage());
        }
        img.recycle();
        return textures[0];
    }

    /**
     * This method is a general method for loading shader to vertex or fragment
     * @param strSource the RawSourceString for the vertex or the fragment
     * @param iType vertex shader or fragment shader
     * @return success or failure indicator, 0 for fail
     */

    public static int LoadShader(String strSource, int iType)
    {
        Log.d("UtilsOpenGl", "LoadShader");
        int[] compiled = new int[1];
        int iShader = GLES32.glCreateShader(iType);
        GLES32.glShaderSource(iShader, strSource);
        GLES32.glCompileShader(iShader);
        GLES32.glGetShaderiv(iShader, GLES32.GL_COMPILE_STATUS, compiled, 0);
        if (compiled[0] == 0) {
            Log.d("Load Shader Failed", "Compilation\n"+GLES32.glGetShaderInfoLog(iShader));
            return 0;
        }
        return iShader;
    }

    /**
     * This is a general method for loading glgrograme for the entire drawing
     * @param strVSource Raw String for vertex
     * @param strFSource Raw String for fragment
     * @return success or failure indicator, 0 for fail
     */

    public static int LoadProgram(String strVSource, String strFSource)
    {
        Log.d("UtilsOpenGl", "LoadProgram");
        int iVShader;
        int iFShader;
        int iProgId;
        int[] link = new int[1];

        iVShader = LoadShader(strVSource, GLES32.GL_VERTEX_SHADER);
        if (iVShader == 0)
        {
            Log.d("Load Program", "Vertex Shader Failed");
            return 0;
        }
        iFShader = LoadShader(strFSource, GLES32.GL_FRAGMENT_SHADER);
        if(iFShader == 0)
        {
            Log.d("Load Program", "Fragment Shader Failed");
            return 0;
        }

        iProgId = GLES32.glCreateProgram();

        GLES32.glAttachShader(iProgId, iVShader);
        GLES32.glAttachShader(iProgId, iFShader);

        GLES32.glLinkProgram(iProgId);

        GLES32.glGetProgramiv(iProgId, GLES32.GL_LINK_STATUS, link, 0);
        if (link[0] <= 0) {
            Log.d("Load Program", "Linking Failed");
            return 0;
        }
        GLES32.glDeleteShader(iVShader);
        GLES32.glDeleteShader(iFShader);
        return iProgId;
    }



    public static float rnd(float min, float max) {
        float fRandNum = (float)Math.random();
        return min + (max - min) * fRandNum;
    }

    /**
     * Read raw string stored in res/raw folder so that we dont need to write raw string in Java files
     * @param context GLSurfaceView
     * @param resourceId raw string ID
     * @return raw string
     */

    public static String readTextFileFromRawResource(final Context context,
                                                     final int resourceId)
    {
        final InputStream inputStream = context.getResources().openRawResource(
                resourceId);
        final InputStreamReader inputStreamReader = new InputStreamReader(
                inputStream);
        final BufferedReader bufferedReader = new BufferedReader(
                inputStreamReader);

        String nextLine;
        final StringBuilder body = new StringBuilder();

        try
        {
            while ((nextLine = bufferedReader.readLine()) != null)
            {
                body.append(nextLine);
                body.append('\n');
            }
        }
        catch (IOException e)
        {
            return null;
        }
        Log.i("Shader:",body.toString());

        return body.toString();
    }


}
