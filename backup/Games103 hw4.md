``` C++
void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{		
		for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping;

                if (i != 0) new_h[i, j] += (h[i - 1, j] - h[i, j]) * rate;
                if (i != size - 1) new_h[i, j] += (h[i + 1, j] - h[i, j]) * rate;
                if (j != 0) new_h[i, j] += (h[i, j - 1] - h[i, j]) * rate;
                if (j != size - 1) new_h[i, j] += (h[i, j + 1] - h[i, j]) * rate;

                old_h[i, j] = h[i, j];
            }

        {
            GameObject Cube = GameObject.Find("Cube");
            Vector3 cube_p = Cube.transform.position;
            Mesh cube_mesh = Cube.GetComponent<MeshFilter>().mesh;

            int li = (int)((cube_p.x + 5.0f) * 10) - 3;
            int ui = (int)((cube_p.x + 5.0f) * 10) + 3;
            int lj = (int)((cube_p.z + 5.0f) * 10) - 3;
            int uj = (int)((cube_p.z + 5.0f) * 10) + 3;
            Bounds bounds = cube_mesh.bounds;

            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        Vector3 p = new Vector3(i * 0.1f - size * 0.05f, -11, j * 0.1f - size * 0.05f);
                        Vector3 q = new Vector3(i * 0.1f - size * 0.05f, -10, j * 0.1f - size * 0.05f);
                        p = Cube.transform.InverseTransformPoint(p);
                        q = Cube.transform.InverseTransformPoint(q);

                        Ray ray = new Ray(p, q - p);
                        float dist = 99999;
                        bounds.IntersectRay(ray, out dist);

                        low_h[i, j] = -11 + dist;//cube_p.y-0.5f;
                    }

            //Initialize Pressure
            for (int i = 0; i < size; i++)
                for (int j = 0; j < size; j++)
                {
                    if (low_h[i, j] > h[i, j])
                    {
                        b[i, j] = 0;
                        vh[i, j] = 0;
                        cg_mask[i, j] = false;
                    }
                    else
                    {
                        cg_mask[i, j] = true;
                        b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
                    }
                }
            Conjugate_Gradient(cg_mask, b, vh, li - 1, ui + 1, lj - 1, uj + 1);
        }

        {
            GameObject Cube = GameObject.Find("Block");
            Vector3 cube_p = Cube.transform.position;
            Mesh cube_mesh = Cube.GetComponent<MeshFilter>().mesh;

            int li = (int)((cube_p.x + 5.0f) * 10) - 3;
            int ui = (int)((cube_p.x + 5.0f) * 10) + 3;
            int lj = (int)((cube_p.z + 5.0f) * 10) - 3;
            int uj = (int)((cube_p.z + 5.0f) * 10) + 3;
            Bounds bounds = cube_mesh.bounds;

            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        Vector3 p = new Vector3(i * 0.1f - size * 0.05f, -11, j * 0.1f - size * 0.05f);
                        Vector3 q = new Vector3(i * 0.1f - size * 0.05f, -10, j * 0.1f - size * 0.05f);
                        p = Cube.transform.InverseTransformPoint(p);
                        q = Cube.transform.InverseTransformPoint(q);

                        Ray ray = new Ray(p, q - p);
                        float dist = 99999;
                        bounds.IntersectRay(ray, out dist);

                        low_h[i, j] = -11 + dist;//cube_p.y-0.5f;
                    }


            //Initialize Pressure
            for (int i = 0; i < size; i++)
                for (int j = 0; j < size; j++)
                {
                    if (low_h[i, j] > h[i, j])
                    {
                        b[i, j] = 0;
                        vh[i, j] = 0;
                        cg_mask[i, j] = false;
                    }
                    else
                    {
                        cg_mask[i, j] = true;
                        b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
                    }
                }
            Conjugate_Gradient(cg_mask, b, vh, li - 1, ui + 1, lj - 1, uj + 1);
        }


        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                if (cg_mask[i, j])
                    vh[i, j] *= gamma;
            }

        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
            {
                if (i != 0) new_h[i, j] += (vh[i - 1, j] - vh[i, j]) * rate;
                if (i != size - 1) new_h[i, j] += (vh[i + 1, j] - vh[i, j]) * rate;
                if (j != 0) new_h[i, j] += (vh[i, j - 1] - vh[i, j]) * rate;
                if (j != size - 1) new_h[i, j] += (vh[i, j + 1] - vh[i, j]) * rate;
            }

        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
                h[i, j] = new_h[i, j];

        //Update cube
        {

            GameObject Cube = GameObject.Find("Cube");
            Vector3 cube_p = Cube.transform.position;
            Mesh cube_mesh = Cube.GetComponent<MeshFilter>().mesh;

            int li = (int)((cube_p.x + 5.0f) * 10) - 3;
            int ui = (int)((cube_p.x + 5.0f) * 10) + 3;
            int lj = (int)((cube_p.z + 5.0f) * 10) - 3;
            int uj = (int)((cube_p.z + 5.0f) * 10) + 3;
            Bounds bounds = cube_mesh.bounds;

            float t = 0.004f;
            float mass = 10.0f;
            Vector3 force = new Vector3(0, -mass * 9.8f, 0);
            Vector3 torque = new Vector3(0, 0, 0);

            for (int i = li - 3; i <= ui + 3; i++)
                for (int j = lj - 3; j <= uj + 3; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size)
                    {
                        Vector3 p = new Vector3(i * 0.1f - size * 0.05f, -11, j * 0.1f - size * 0.05f);
                        Vector3 q = new Vector3(i * 0.1f - size * 0.05f, -10, j * 0.1f - size * 0.05f);

                        //Debug.Log("ok");
                        //Debug.Log(p);

                        p = Cube.transform.InverseTransformPoint(p);
                        q = Cube.transform.InverseTransformPoint(q);
                        //Debug.Log(p);

                        Ray ray = new Ray(p, q - p);
                        float dist = 99999;
                        bounds.IntersectRay(ray, out dist);

                        /*if(i==50 && j==50)	
                        {
                            Debug.Log(p);
                            Debug.Log(q);
                            Debug.Log(-11+dist);
                        }*/
                        //Debug.Log(cube_p.y-0.5f);


                        if (vh[i, j] != 0)
                        {
                            Vector3 r = p + dist * (q - p) - cube_p;
                            Vector3 f = new Vector3(0, vh[i, j], 0) * 4.0f;
                            force += f;

                            torque += Vector3.Cross(r, f);
                        }
                    }
            cube_v *= 0.99f;
            cube_w *= 0.99f;
            cube_v += force * t / mass;
            cube_p += cube_v * t;
            Cube.transform.position = cube_p;
            cube_w += torque * t / (100.0f * mass);
            Quaternion cube_q = Cube.transform.rotation;
            Quaternion wq = new Quaternion(cube_w.x, cube_w.y, cube_w.z, 0);
            Quaternion temp_q = wq * cube_q;
            cube_q.x += 0.5f * t * temp_q.x;
            cube_q.y += 0.5f * t * temp_q.y;
            cube_q.z += 0.5f * t * temp_q.z;
            cube_q.w += 0.5f * t * temp_q.w;
            Cube.transform.rotation = cube_q;
        }
	}
```

``` C++
void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				h[i, j] = X[i * size + j].y;
			}
		}
		
		if (Input.GetKeyDown ("r")) 
		{
			//TODO: Add random water.
			int i = Random.Range(0, size);
			int j = Random.Range(0, size);
			float val = Random.value;

			if (i < 1) i = 1;
			if (j < 1) j = 1;
			if (i >= size - 1) i = size - 2;
			if (j >= size - 1) j = size - 2;

			h[i, j] += val;
			h[i - 1, j] -= val / 4.0f;
			h[i + 1, j] -= val / 4.0f;
			h[i, j - 1] -= val / 4.0f;
			h[i, j + 1] -= val / 4.0f;
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}
		
		//TODO: Store h back into X.y and recalculate normal.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				X[i * size + j].y = h[i, j];
			}
		}
		mesh.vertices = X;
		mesh.RecalculateNormals();
	}
```

