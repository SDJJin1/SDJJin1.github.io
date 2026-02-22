补全Start函数中的部分
``` C++
inv_Dm = new Matrix4x4[tet_number];
		for(int i = 0; i < tet_number; i++)
		{
			Matrix4x4 Dm = Build_Edge_Matrix(i);
			inv_Dm[i] = Dm.inverse;
		}
```

Build_Edge_Matrix函数
```
Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.

	    for (int i = 0; i < 3; i++)//使用 tet 四面体的顶点坐标构建边矩阵。矩阵的前三列存储了三个边向量，这些向量是从四面体的第一个顶点到其他三个顶点的向量
	    {
		    ret[0, i] = X[Tet[tet * 4 + i + 1]].x - X[Tet[tet * 4 + 0]].x;
		    ret[1, i] = X[Tet[tet * 4 + i + 1]].y - X[Tet[tet * 4 + 0]].y;
		    ret[2, i] = X[Tet[tet * 4 + i + 1]].z - X[Tet[tet * 4 + 0]].z;
	    }
	    ret[3, 3] = 1.0f;//ret[3, 3] = 1.0f 用于将矩阵转换为齐次坐标
 
	    return ret;
    }
```

_Update函数
```
// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
		    Force[i] = mass * new Vector3(0, -9.8f, 0);
	    }

    	for(int tet=0; tet<tet_number; tet++)
    	{
		    Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
		    //TODO: Green Strain
		    Matrix4x4 G = Matrix4x4_mul_float(Matrix4x4_minus_Matrix4x4(F.transpose * F, Matrix4x4.identity), 0.5f);
		    //TODO: Second PK Stress
		    Matrix4x4 S = Matrix4x4.zero;
		    float trace = G[0, 0] + G[1, 1] + G[2, 2];
		    S = Matrix4x4_add_Matrix4x4(Matrix4x4_mul_float(G, 2.0f * stiffness_1), Matrix4x4_mul_float(Matrix4x4.identity, stiffness_0 * trace));
		    //TODO: Elastic Force
		    float volume = 1.0f / (inv_Dm[tet].determinant * 6);
		    Matrix4x4 Elastic_force = Matrix4x4_mul_float(F * S * inv_Dm[tet].transpose, -volume);
		    for (int k = 0; k < 3; k++)//将弹性力应用到顶点
		    {
			    Force[Tet[tet * 4 + k + 1]].x += Elastic_force[0, k];
			    Force[Tet[tet * 4 + k + 1]].y += Elastic_force[1, k];
			    Force[Tet[tet * 4 + k + 1]].z += Elastic_force[2, k];
		    }
 
		    // Elastic_force0 = -Elastic_force1-Elastic_force2-Elastic_force3 
		    //将弹性力添加到四面体的顶点上。前三个顶点的力加上 Elastic_force，第一个顶点的力减去所有其他顶点的力，以保证力的平衡
		    Force[Tet[tet * 4 + 0]].x -= Elastic_force[0, 0] + Elastic_force[0, 1] + Elastic_force[0, 2];
		    Force[Tet[tet * 4 + 0]].y -= Elastic_force[1, 0] + Elastic_force[1, 1] + Elastic_force[1, 2];
		    Force[Tet[tet * 4 + 0]].z -= Elastic_force[2, 0] + Elastic_force[2, 1] + Elastic_force[2, 2];
	    }
	    
	    Smooth_V(0.75f);

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
		    V[i] = (V[i] + dt * Force[i] / mass) * damp;//Force[i] / mass 计算每个顶点所受的加速度,dt * Force[i] / mass 计算时间步长 dt 内的速度变化量
		    X[i] = X[i] + V[i] * dt;//X[i] + V[i] * dt 使用更新后的速度计算新位置
 
		    //TODO: (Particle) collision with floor.处理与地面的碰撞
		    float d = Vector3.Dot(X[i] - P, N);//计算顶点与地面的距离 Vector3.Dot(X[i] - P, N) 计算顶点到地面某一点的向量在地面法线 N 上的投影，得到顶点到地面的垂直距离 d
		    if (d < 0.0f) // collision occur
		    {
			    //计算法向速度分量 v_N_size
			    float v_N_size = Vector3.Dot(V[i], N);//Vector3.Dot(V[i], N) 计算速度 V[i] 在法线方向上的分量 v_N_size
			    // check velocity检查法向速度方向
			    if (v_N_size < 0.0f)//v_N_size < 0.0f，表示顶点在向下运动，需要处理碰撞
			    {
				    Vector3 v_N = v_N_size * N;//Vector3 v_N = v_N_size * N 计算法向速度分量 v_N
				    Vector3 v_T = V[i] - v_N;//Vector3 v_T = V[i] - v_N 计算切向速度分量 v_T
				    Vector3 v_N_new = -1.0f * 0.5f * v_N;//Vector3 v_N_new = -1.0f * restitution * v_N 计算碰撞后的法向速度，应用恢复系数 restitution 进行反弹
				    float a = Math.Max(1.0f - 0.5f * (1.0f + 0.5f) * v_N.magnitude / v_T.magnitude, 0.0f);//计算切向速度的衰减系数，考虑摩擦系数 friction 和恢复系数 restitution
				    Vector3 v_T_new = a * v_T;//计算碰撞后的切向速度
				    V[i] = v_N_new + v_T_new;//更新顶点速度，包含新的法向速度和切向速度
			    }
		    }
    	}
```

