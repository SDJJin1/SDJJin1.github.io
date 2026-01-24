``` c++
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f hitColor = this->backgroundColor;
    Intersection shade_point_inter = Scene::intersect(ray);
    if (shade_point_inter.happened)
    {

        Vector3f p = shade_point_inter.coords;
        Vector3f wo = ray.direction;
        Vector3f N = shade_point_inter.normal;
        Vector3f L_dir(0), L_indir(0);

       //sampleLight(inter,pdf_light)
        Intersection light_point_inter;
        float pdf_light;
        sampleLight(light_point_inter, pdf_light);
        //Get x,ws,NN,emit from inter
        Vector3f x = light_point_inter.coords;
        Vector3f ws = normalize(x-p);
        Vector3f NN = light_point_inter.normal;
        Vector3f emit = light_point_inter.emit;
        float distance_pTox = (x - p).norm();
        //Shoot a ray from p to x
        Vector3f p_deviation = (dotProduct(ray.direction, N) < 0) ?
                p + N * EPSILON :
                p - N * EPSILON ;

        Ray ray_pTox(p_deviation, ws);
        //If the ray is not blocked in the middleff
        Intersection blocked_point_inter = Scene::intersect(ray_pTox);
        if (abs(distance_pTox - blocked_point_inter.distance < 0.01 ))
        {
            L_dir = emit * shade_point_inter.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (distance_pTox * distance_pTox * pdf_light);
        }
        //Test Russian Roulette with probability RussianRouolette
        float ksi = get_random_float();
        if (ksi < RussianRoulette)
        {
            //wi=sample(wo,N)
            Vector3f wi = normalize(shade_point_inter.m->sample(wo, N));
            //Trace a ray r(p,wi)
            Ray ray_pTowi(p_deviation, wi);
            //If ray r hit a non-emitting object at q
            Intersection bounce_point_inter = Scene::intersect(ray_pTowi);
            if (bounce_point_inter.happened && !bounce_point_inter.m->hasEmission())
            {
                float pdf = shade_point_inter.m->pdf(wo, wi, N);
                if(pdf> EPSILON)
                    L_indir = castRay(ray_pTowi, depth + 1) * shade_point_inter.m->eval(wo, wi, N) * dotProduct(wi, N) / (pdf *RussianRoulette);
            }
        }
        hitColor = shade_point_inter.m->getEmission() + L_dir + L_indir;
    }
    return hitColor;
}
```
得到结果

<img width="493" height="486" alt="Image" src="https://github.com/user-attachments/assets/674c57ad-224b-446d-8bc8-20b4d97f7983" />