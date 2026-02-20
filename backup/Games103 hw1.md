impulse方法
``` c++
void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		
		Matrix4x4 R = Matrix4x4.Rotate(this.transform.rotation);
		Vector3 T = transform.position;
		
		Vector3 sum = Vector3.zero;
		int collisionNum = 0;

		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 r_i = vertices[i];
			Vector3 Rri = R.MultiplyVector(r_i);
			Vector3 x_i = T + Rri;
			float d = Vector3.Dot(x_i - P, N);
			if (d < 0.0f) // collision occur
			{
				Vector3 v_i = v + Vector3.Cross(w, Rri);
				float v_N_size = Vector3.Dot(v_i, N);
				// check velocity
				if (v_N_size < 0.0f)
				{
					sum += r_i;
					collisionNum++;
				}
			}
		}
		
		if (collisionNum == 0) return;
		Matrix4x4 I_rot = R * I_ref * R.transpose;
		Matrix4x4 I_inverse = I_rot.inverse;      
		Vector3 r_collision = sum / (float)collisionNum;                // virtual collision point（local coordination）
		Vector3 Rr_collision = R.MultiplyVector(r_collision);
		//Vector3 x_collision = T + Rr_collision;							 // virtual collision point（global coordination）
		Vector3 v_collision = v + Vector3.Cross(w, Rr_collision);
        
		// Compute the wanted v_N
		Vector3 v_N = Vector3.Dot(v_collision, N) * N;
		Vector3 v_T = v_collision - v_N;
		Vector3 v_N_new = -1.0f * restitution * v_N;
		float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0.0f);
		Vector3 v_T_new = a * v_T;
		Vector3 v_new = v_N_new + v_T_new;
        
		// Compute the impulse J
		Matrix4x4 Rri_star = Get_Cross_Matrix(Rr_collision);
		Matrix4x4 K = Matrix_subtraction(Matrix_miltiply_float(Matrix4x4.identity, 1.0f / mass),
			Rri_star * I_inverse * Rri_star);
		Vector3 J = K.inverse.MultiplyVector(v_new - v_collision);
        
		// Update v and w with impulse J
		v = v + 1.0f / mass * J;
		w = w + I_inverse.MultiplyVector(Vector3.Cross(Rr_collision, J));
	}
```

