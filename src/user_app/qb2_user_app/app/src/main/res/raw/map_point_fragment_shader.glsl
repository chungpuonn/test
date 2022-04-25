precision mediump float; // Set the default precision to medium. We don't need as high of a precision in the fragment shader.
uniform sampler2D u_texture;
varying vec4 v_Color;

void main()                    
{
	vec4 tex = texture2D(u_texture, gl_PointCoord);
	gl_FragColor = tex * v_Color;  // White Color
}                              