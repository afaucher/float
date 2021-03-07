#ifndef PLAYER_H
#define PLAYER_H

#include "HoverCraftyCommon.h"
#include "GameArena.h"

namespace HoverCrafty {
    class Player {
    private:
        GameArena * game_arena;
        dBodyID body_id;
        static const int ray_count = 10;
        static const float max_thrust = 6.0f;
        static const float ship_prop_height = 0.0f;
        static const float jump_thrust = 3000.0f;
        dGeomID thruster_rays[ray_count][ray_count];
        float distance_found[ray_count][ray_count];
        float thrust_force[ray_count][ray_count];
        float thrust_percent;
        float x_dist;
        float y_dist;
        float throttle_percent;
        float rudder_position;
        
        std::vector<std::vector<dReal> > forces;
        void AddRelativeBodyForce(dReal fx,dReal fy,dReal fz,dReal px,dReal py,dReal pz);
        void AddRelativeBodyForceAbs(dReal fx,dReal fy,dReal fz,dReal px,dReal py,dReal pz);
        
        GameArena::dCollisionData thruster_ray_collision_data;
        
        static const float ship_length = 10;
        static const float ship_width = 5;
        static const float ship_height = 1;
        static const float thruster_max_length = 5;
        
    public:
        Player(GameArena * game_arena);
        
        const dReal * GetPosition();
        
        void Jump();
        
        void Draw();
        void Update(float step_size);
        void SetThrust(float thrust_percent);
        void SetThrottle(float throttle_percent);
        void SetRudder(float rudder_position);
        void SetRoll(float r);
        void SetPitch(float p);
        void Reset();
    private:
        static bool RayAccumulate(void * context, dGeomID o1, dGeomID o2, dContact * contacts, int n_contacts);
    };
}

#endif //PLAYER_H