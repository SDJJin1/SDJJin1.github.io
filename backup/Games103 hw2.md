``` c++
void Get_Gradient(Vector3[] X, Vector3[] X_hat, float t, Vector3[] G)
	{
		for (int i = 0; i < G.Length; i++)
		{
			G[i] = (mass / (t * t)) * (X[i] - X_hat[i]) - mass * new Vector3(0, -9.8f, 0);
		}

		for (int e = 0; e < E.Length / 2; e++)
		{
			int i = E[e * 2 + 0];
			int j = E[e * 2 + 1];
			
			Vector3 f = spring_k * (1-L[e] / (X[i] - X[j]).magnitude) *(X[i] - X[j]);
			G[i] += f;
			G[j] -= f;
		}
	}
```

``` c++
void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X 		= mesh.vertices;
		Vector3[] last_X 	= new Vector3[X.Length];
		Vector3[] X_hat 	= new Vector3[X.Length];
		Vector3[] G 		= new Vector3[X.Length];

		//Initial Setup.
		for (int i = 0; i < X.Length; i++)
		{
			V[i] *= damping;
			X[i] = X_hat[i]= X[i] + t * V[i];
		}

		float omega = 1.0f;
		for(int k=0; k<32; k++)
		{
			if (k == 0) omega = 1.0f;
			else if (k == 1) omega = 2.0f / (2.0f - rho * rho);
			else omega = 4.0f / (4.0f - rho * rho * omega);
			
			Get_Gradient(X, X_hat, t, G);
			
			//Update X by gradient.
			for (int i = 0; i < X.Length; i++)
			{
				if(i == 0 || i == 20) continue;
				
				Vector3 new_x = omega * (X[i] - 1/ (mass / (t * t) + spring_k*4)*G[i])
					+ (1-omega)*last_X[i];
				last_X[i] = X[i];
				X[i] = new_x;
			}
		}

		for (int i = 0; i < X.Length; i++)
		{
			V[i] += (X[i] - X_hat[i]) / t;
		}
		
		mesh.vertices = X;

		Collision_Handling ();
		mesh.RecalculateNormals ();
	}
```