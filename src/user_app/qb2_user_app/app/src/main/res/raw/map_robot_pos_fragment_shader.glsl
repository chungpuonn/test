precision mediump float; // Set the default precision to medium. We don't need as high of a precision in the fragment shader.
uniform sampler2D u_texture;

void main()                    
{
	vec4 tex = texture2D(u_texture, gl_PointCoord);
	gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0);  // White Color
}                              