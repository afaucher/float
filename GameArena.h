#ifndef GAME_ARENA_H
#define GAME_ARENA_H

#include "HoverCraftyCommon.h"

namespace HoverCrafty {
    class GameArena {
    private:
        int x_size;
        int y_size;
        int block_size;
        dWorldID world_id;
        dSpaceID world_space_id;
        //dBodyID ground_body_id;
        dJointGroupID contactGroup;
        
        std::vector<dBodyID> boxes;
        
    public:
        
        std::vector<Player*> players;
        
        typedef struct {
            //o1 is always the object for this dCollisionData
            //return true to ignore collision
            typedef bool (*dCollisionFunction)(void*context,dGeomID o1, dGeomID o2, dContact * contacts, int n_contacts);
            dCollisionFunction callback;
            void * context;
        } dCollisionData;
        
        dWorldID GetPhysics() { return world_id; }
        dSpaceID GetSpace() { return world_space_id; }
        
        GameArena();
        
        void Draw();
        
        void Update(float step_size);
        
    private:
        void callbackFunc(dGeomID o1, dGeomID o2);
        static void callbackFuncHelper(void* data,dGeomID o1, dGeomID o2);
        void CreateBox();
        void CreateGroundBox();
    };
}

#endif //GAME_ARENA_H