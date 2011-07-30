#define USE_SDL_JOYSTICK 0
#define USE_LINUX_JOYSTICK 1

#if USE_SDL_JOYSTICK
#include <SDL/SDL.h>
#include <SDL/SDL_joystick.h>
#endif //USE_SDL_JOYSTICK
#if USE_LINUX_JOYSTICK
#include "joystick.hpp"
#endif //USE_LINUX_JOYSTICK

#include "HoverCraftyCommon.h"
#include "Player.h"
#include "GameArena.h"
#include "Util.h"
#include "joystick.hpp"

namespace HoverCrafty {
    
#define TICK_MS 50

#define MAX_CONTACTS 16
#define JOYSTICK_AXIS_RANGE 32767
#define THROTTLE_INDEX 2
#define PITCH_INDEX 0
#define ROLL_INDEX 1
#define RUDDER_INDEX 3
#define FIRE_THRUSTERS_INDEX 0

    class GameArena;
    
    static void setTransform (const dReal pos[3], const dReal R[12])
    {
        GLfloat matrix[16];
        matrix[0]=R[0];
        matrix[1]=R[4];
        matrix[2]=R[8];
        matrix[3]=0;
        matrix[4]=R[1];
        matrix[5]=R[5];
        matrix[6]=R[9];
        matrix[7]=0;
        matrix[8]=R[2];
        matrix[9]=R[6];
        matrix[10]=R[10];
        matrix[11]=0;
        matrix[12]=pos[0];
        matrix[13]=pos[1];
        matrix[14]=pos[2];
        matrix[15]=1;
        glPushMatrix();
        glMultMatrixf (matrix);
    }
    
    
    void GameArena::CreateGroundBox()
    {
        //dGeomID ground_geom_id = dCreateBox(world_space_id,100,100,1);
        dGeomID ground_plane_geom_id = dCreatePlane(world_space_id,0,0,1,0);
    }
    
    static bool hit_box[50] = {};

    void GameArena::CreateBox()
    {
        //Faling Box
        dBodyID ground_body_id = dBodyCreate(world_id);
        dMass m;
        dMassSetBox(&m,1,1,1,1);
        dBodySetMass(ground_body_id,&m);
        
        //Falling box geom
        dGeomID ground_geom_id = dCreateBox(world_space_id,1,1,1);
        dGeomSetBody(ground_geom_id,ground_body_id);
        
        dBodySetPosition(ground_body_id,rand() % 20,rand() % 20, 1 + (rand() % 8));
        
        boxes.push_back(ground_body_id);
    }
    
    GameArena::GameArena() :
        x_size(100),
        y_size(100),
        block_size(6),
        world_id(NULL),
        world_space_id(NULL),
        contactGroup(NULL)
    {
        world_id = dWorldCreate();
        
        dWorldSetGravity(world_id,0,0,-9.81f);
        //dWorldSetAutoDisableFlag(world_id, 1);
        
        world_space_id = dHashSpaceCreate(NULL);
        contactGroup = dJointGroupCreate(0);

        for (int i = 0; i < 50; i++) CreateBox();
        
        //Ground Plane
        CreateGroundBox();
        
        
        players.push_back(new Player(this));
    }
    
    void GameArena::Draw()
    {
        /*glColor3f (1, 1, 1);
        glBegin (GL_QUADS);
        glVertex3d (0, 0,0);
        glVertex3f (0, y_size * block_size,0);
        glVertex3f (x_size * block_size, y_size * block_size,0);
        glVertex3f (x_size * block_size, 0,0);
        glEnd ();*/
        
        Player * first_player = players[0];
        const dReal * pos = first_player->GetPosition();
        
        gluLookAt (pos[0] + 10,
                   pos[1] + 10,
                   pos[2] + 10,
                   pos[0],
                   pos[1],
                   pos[2],
                   0,0,1);
        
        glColor3f(0.4f,0.4f,0.4f);
        glBegin(GL_LINES);
        int ground_draw_distance = 50;
        for (int x = 0; x < ground_draw_distance; x++) {
            for (int y = 0; y < ground_draw_distance; y++) {
                int x_pos = pos[0] - ground_draw_distance/2 + x;
                int y_pos = pos[1] - ground_draw_distance/2 + y;
                glVertex3f(x_pos,y_pos,0);
                glVertex3f(x_pos,y_pos+ground_draw_distance,0);
                glVertex3f(x_pos,y_pos,0);
                glVertex3f(x_pos+ground_draw_distance,y_pos,0);
            }
        }
        glEnd();
        
        std::vector<dBodyID>::iterator boxes_it = boxes.begin();
        for (int i = 0; boxes_it != boxes.end(); boxes_it++, i++) {
            if (hit_box[i]) {
                glColor3f (0, 1, 0);
            } else {
                glColor3f (0, 0, 1);
            }
            hit_box[i] = false;
            setTransform(dBodyGetPosition((*boxes_it)),dBodyGetRotation((*boxes_it)));
            glutWireCube(1);
            glPopMatrix();
        }
        
        std::vector<Player*>::iterator it = players.begin();
        for (; it != players.end(); it++) {
            (*it)->Draw();
        }
    }
    
    void GameArena::Update(float step_size)
    {
        //printf("Step: %f\n",step_size);
        
        dSpaceCollide(world_space_id,this,&callbackFuncHelper);
        dWorldQuickStep(world_id, step_size);
        dJointGroupEmpty(contactGroup);

        std::vector<Player*>::iterator it = players.begin();
        for (; it != players.end(); it++) {
            (*it)->Update(step_size);
        }
    }
    
    void GameArena::callbackFunc(dGeomID o1, dGeomID o2)
    {
        dContact contacts[MAX_CONTACTS];
        int i,numc;
        if (numc = dCollide(o1,o2,MAX_CONTACTS,&contacts[0].geom,sizeof(dContact))) {
            
            bool ignore = false;
            
            dCollisionData * cd = (dCollisionData *)dGeomGetData(o1);
            if (cd != NULL && cd->callback) {
                //printf("Calling callback o1\n");
                ignore = ignore || cd->callback(cd->context,o1,o2, contacts, numc);
            }
            cd = (dCollisionData *)dGeomGetData(o2);
            if (cd != NULL && cd->callback) {
                //printf("Calling callback o2\n");
                ignore = ignore || cd->callback(cd->context,o2,o1, contacts, numc);
            }
            
            if (ignore) {
                std::vector<dBodyID>::iterator boxes_it = boxes.begin();
                for (int i = 0; boxes_it != boxes.end(); boxes_it++,i++) {
                    if (dGeomGetBody(o1) == *boxes_it || dGeomGetBody(o2) == *boxes_it) {
                        hit_box[i] = true;
                    }
                }
                return;
            }
            
            dBodyID b1 = dGeomGetBody(o1);
            dBodyID b2 = dGeomGetBody(o2);
            
            for (i = 0;i < numc;i++)
            {
                contacts[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
                contacts[i].surface.mu = dInfinity;
                contacts[i].surface.slip1 = 0.1;
                contacts[i].surface.slip2 = 0.1;
                contacts[i].surface.soft_erp = 0.5;
                contacts[i].surface.soft_cfm = 0.0;

                dJointID c = dJointCreateContact(world_id,contactGroup,&contacts[i]);
                dJointAttach(c,b1,b2);
            }
            
            
        }
    }
    void GameArena::callbackFuncHelper(void* data,dGeomID o1, dGeomID o2)
    {
        GameArena * a = (GameArena*)data;
        a->callbackFunc(o1,o2);
    }
    
    const dReal * Player::GetPosition()
    {
        return dBodyGetPosition(body_id);
    }
    
    bool Player::RayAccumulate(void * context, dGeomID o1, dGeomID o2, dContact * contacts, int n_contacts)
    {
        Player * p = (Player*)context;
        
        if (n_contacts <= 0) {
            printf("no hits\n");
            return true;
        }
        
        //printf("ray hit\n");
        for (int t_x = 0; t_x < ray_count; ++t_x) {
            for (int t_y = 0; t_y < ray_count; ++t_y) {
                
                if (p->thruster_rays[t_x][t_y] == o1) {
                    dBodyID o2_body_id = dGeomGetBody(o2);
                    assert(o2_body_id != p->body_id);
                    p->distance_found[t_x][t_y] = contacts[0].geom.depth;
                    for (int i = 1; i < n_contacts; i++) {
                        if (p->distance_found[t_x][t_y] > contacts[i].geom.depth) {
                            printf("Out of order\n");
                        }
                        p->distance_found[t_x][t_y] = std::min((dReal)p->distance_found[t_x][t_y], contacts[i].geom.depth);
                    }
                    
                    //printf("Ray Hit: %f\n",contacts[0].geom.depth);
                    
                    return true;
                }
            }
        }
        printf("Ray Not Found!\n");
        return true;
    }
    
    void Player::Jump() {
        AddRelativeBodyForceAbs(
            0,0,jump_thrust,0,0,0);
    }
    
    void Player::Reset()
    {
        printf("Reset\n");
        throttle_percent = 0;
        rudder_position = 0.5f;
        thrust_percent = 0;
        x_dist = 0.5f;
        y_dist = 0.5f;
        dBodySetPosition(body_id,5,5,10);
        dMatrix3 R;
        dRFromAxisAndAngle(R,0,1,0,0);
        dBodySetRotation(body_id,R);
        dBodySetLinearVel(body_id,0,0,0);
    }
    
    Player::Player(GameArena * game_arena) :
        game_arena(game_arena),
        body_id(NULL),
        thruster_ray_collision_data(),
        distance_found(),
        thrust_percent(0.0f),
        x_dist(0.5f),
        y_dist(0.5f),
        throttle_percent(0),
        rudder_position(0.5f)
    {
        dWorldID world_id = game_arena->GetPhysics();
        dSpaceID space_id = game_arena->GetSpace();
        
        body_id = dBodyCreate(world_id);
        dBodySetPosition(body_id,5,5,10);
        dMass m;
        dMassSetBox(&m,1,ship_length,ship_width,ship_height);
        dBodySetMass(body_id,&m);
        //dReal damping = dBodyGetAngularDamping(body_id);
        dReal damping = 0.01f;
        //printf("Player damping: %f\n", damping);
        dBodySetAngularDamping(body_id,damping);
        dBodySetLinearDamping(body_id,damping);
        dBodySetGyroscopicMode(body_id,true);
        
        dGeomID ship_geom_id = dCreateBox(space_id,ship_length,ship_width,ship_height);
        dGeomSetBody(ship_geom_id,body_id);
        
        thruster_ray_collision_data.callback = &Player::RayAccumulate;
        thruster_ray_collision_data.context = this;
        
        for (int t_x = 0; t_x < ray_count; ++t_x) {
            for (int t_y = 0; t_y < ray_count; ++t_y) {
                dGeomID ray_geom_id = dCreateRay(space_id,thruster_max_length);
                thruster_rays[t_x][t_y] = ray_geom_id;
                
                float r_x = (t_x * ship_length / (float)(ray_count - 1)) - (ship_length / 2.0f);
                float r_y = (t_y * ship_width / (float)(ray_count - 1)) - (ship_width / 2.0f);
                
                dGeomRaySet(ray_geom_id,0,0,0,0,0,1);
                dGeomSetBody(ray_geom_id,body_id);
                float ship_height_offset = (ship_height - 0.01f) / 2.0f;
                dMatrix3 R;
                dRFromAxisAndAngle(R,0,1,0,M_PI);
                dGeomSetOffsetRotation(ray_geom_id,R);
                dGeomSetOffsetPosition(ray_geom_id,r_x,r_y,-ship_height_offset);
                
                dGeomSetData(ray_geom_id,&thruster_ray_collision_data);
                dGeomRaySetClosestHit(ray_geom_id,1);
            }
        }
    }
    
    void Player::SetThrust(float thrust_percent)
    {
        this->thrust_percent = std::max(std::min(thrust_percent,1.0f),0.0f);
    }

        
    void Player::Draw()
    {
        glColor4f (0, 0, 1, 1);
        setTransform(dBodyGetPosition(body_id),dBodyGetRotation(body_id));
            glPushMatrix();
                glScalef(ship_length,ship_width,ship_height);
                glutWireCube(1);
            glPopMatrix();
            
            std::vector<std::vector<dReal> >::iterator forces_it = forces.begin();
            
            glBegin(GL_LINES);
            for (; forces_it != forces.end(); forces_it++) {
                float length_factor = 0.1f;
                glColor3f(1,0,0);
                glVertex3f((*forces_it)[3],(*forces_it)[4],(*forces_it)[5]);
                glColor3f(1,1,1);
                glVertex3f((*forces_it)[0]*length_factor + (*forces_it)[3],
                           (*forces_it)[1]*length_factor + (*forces_it)[4],
                           (*forces_it)[2]*length_factor + (*forces_it)[5]);
            }
            glEnd();
            forces.clear();
            
            glTranslatef(ship_length/-2.0f,0,ship_prop_height);
            glutWireCube(throttle_percent);
        glPopMatrix();
        
        for (int t_y = 0; t_y < ray_count; ++t_y) {
            for (int t_x = 0; t_x < ray_count; ++t_x) {
                const dReal * pos = dGeomGetPosition(thruster_rays[t_x][t_y]);
                const dReal * rot = dGeomGetRotation(thruster_rays[t_x][t_y]);
                
                setTransform(pos,rot);
                    
                    float d = distance_found[t_x][t_y];
                    distance_found[t_x][t_y] = 0;
                    
                    glBegin(GL_LINES);
                    if (d < thruster_max_length) {
                        glColor4f (1, 0, 0, 1);
                    } else {
                        glColor4f (1, 1, 0, 1);
                    }
                    glVertex3f(0,0,d*0.9f);
                    glVertex3f(0,0,d);
                    glColor4f (1, 0, 1, 1);
                    glVertex3f(0,0,-1-thrust_force[t_x][t_y]*0.9f);
                    glVertex3f(0,0,-1-thrust_force[t_x][t_y]);
                    glEnd();
                    
                glPopMatrix();
            }
        }
    }
    
    void Player::Update(float step_size)
    {
        float force_per = max_thrust * thrust_percent;
        
        for (int t_y = 0; t_y < ray_count; ++t_y) {
            for (int t_x = 0; t_x < ray_count; ++t_x) {
                thrust_force[t_x][t_y] = 0;
                
                float r_x = (t_x * ship_length / (float)(ray_count - 1)) - (ship_length / 2.0f);
                float r_y = (t_y * ship_width / (float)(ray_count - 1)) - (ship_width / 2.0f);
                
                float float_x_pos = (t_x / (float)(ray_count-1));
                float thrust_dist_x = x_dist * float_x_pos + (1.0f - x_dist) * (1 - float_x_pos);
                
                float float_y_pos = (t_y / (float)(ray_count-1));
                float thrust_dist_y = y_dist * float_y_pos + (1.0f - y_dist) * (1 - float_y_pos);
                
                //printf("x %d,%f -> %f\n", t_x,x_dist,2.0f * thrust_dist_x);
                //printf("y %d,%f -> %f\n", t_y,y_dist,2.0f * thrust_dist_y);
                
                float backwash_ratio = 0.5f;
                float normal_thrust = force_per * (thrust_dist_x + thrust_dist_y);
                float backwash_force_at = 0;
                
                if (distance_found[t_x][t_y] < thruster_max_length
                    && distance_found[t_x][t_y] > 0)
                {
                    float backwash = ((thruster_max_length - distance_found[t_x][t_y]) /thruster_max_length);
                    backwash_force_at = backwash_ratio * normal_thrust * backwash;
                }
                
                float force_at = backwash_force_at + (1.0f - backwash_ratio) * normal_thrust;
                thrust_force[t_x][t_y] = force_at;
                
                float foward_scale = 1;
                float foward_thrust = sin((x_dist - 0.5) * foward_scale) * force_at;
                float vertical_thrust = cos((x_dist - 0.5) * foward_scale) * force_at;
                
                AddRelativeBodyForce(
                    -foward_thrust,0,vertical_thrust,r_x,r_y,0);
            }
        }
        
        if (throttle_percent > 0 || rudder_position != 0.5f) {
            float ship_full_throttle_force = 100.0f;
            
            
            float total_thrust = throttle_percent*ship_full_throttle_force;
            float total_turn_thrust = ship_full_throttle_force;
            float x_thrust = total_thrust * std::cos(rudder_position-0.5f);
            float y_thrust = total_turn_thrust * std::sin(rudder_position-0.5f);
            
            //dBodyAddForceAtRelPos(body_id,
            AddRelativeBodyForce(
                                  -x_thrust,y_thrust,0,
                                  ship_length/-4.0,0,ship_prop_height);
        }
    }
    
    void Player::AddRelativeBodyForce(dReal fx,dReal fy,dReal fz,dReal px,dReal py,dReal pz)
    {
        std::vector<dReal> f;
        f.push_back(fx);
        f.push_back(fy);
        f.push_back(fz);
        f.push_back(px);
        f.push_back(py);
        f.push_back(pz);
        
        dBodyAddRelForceAtRelPos(body_id,
            fx,fy,fz,px,py,pz);
        
        forces.push_back(f);
    }
    
    void Player::AddRelativeBodyForceAbs(dReal fx,dReal fy,dReal fz,dReal px,dReal py,dReal pz)
    {
        std::vector<dReal> f;
        f.push_back(fx);
        f.push_back(fy);
        f.push_back(fz);
        f.push_back(px);
        f.push_back(py);
        f.push_back(pz);
        
        dBodyAddForceAtRelPos(body_id,
            fx,fy,fz,px,py,pz);
        
        forces.push_back(f);
    }
    
    void Player::SetRoll(float x_dist)
    {
        this->x_dist = std::max(std::min(x_dist, 1.0f),0.0f);
    }
    
    void Player::SetPitch(float y_dist)
    {
        this->y_dist = std::max(std::min(y_dist, 1.0f),0.0f);
    }
    
    void Player::SetThrottle(float throttle_percent)
    {
        this->throttle_percent = std::max(std::min(throttle_percent, 1.0f),0.0f);
    }
    
    void Player::SetRudder(float rudder_position)
    {
        this->rudder_position = std::max(std::min(rudder_position, 1.0f),0.0f);
    }
    
    class Main {
    private:
            float camera_rotation;
#if USE_SDL_JOYSTICK
            SDL_Joystick *joystick;
#endif //USE_SDL_JOYSTICK
#if USE_LINUX_JOYSTICK
            Joystick * joystick;
#endif //USE_LINUX_JOYSTICK
            GameArena * game_arena;

    public:

        Main() :
            camera_rotation(0),
#if USE_SDL_JOYSTICK || USE_LINUX_JOYSTICK
            joystick(NULL),
#endif //USE_SDL_JOYSTICK
            game_arena(new GameArena())

        {
#if USE_SDL_JOYSTICK
            int num_joysticks = SDL_NumJoysticks();
            joystick = SDL_JoystickOpen(0);
            int number_of_buttons = SDL_JoystickNumButtons(joystick);
            int x = SDL_JoystickGetAxis(joystick, 0);
            int y = SDL_JoystickGetAxis(joystick, 1);
            int z = SDL_JoystickGetAxis(joystick, 2);
            printf("num_joysticks: %d, number_of_buttons: %d, (%d,%d,%d)\n", num_joysticks, number_of_buttons,x,y,z);
#endif
#if USE_LINUX_JOYSTICK
            std::vector<JoystickDescription> descriptions = Joystick::get_joysticks();
            if (descriptions.size() > 0) {
                INFO("Opening: \"%s\"", descriptions[0].filename.c_str());
                joystick = new Joystick(descriptions[0].filename);
                int number_of_buttons = joystick->get_button_count();
                int number_of_axis = joystick->get_axis_count();
                INFO("Buttons: %d Axis: %d", number_of_buttons, number_of_axis);
                joystick->axis_move.connect(sigc::mem_fun(this, &Main::JoystickAxisCallback));
                joystick->button_move.connect(sigc::mem_fun(this, &Main::JoystickButtonCallback));
            }
#endif //USE_LINUX_JOYSTICK
        }
        
        void JoystickAxisCallback(int index, int value)
        {
            //INFO("Axis %d:%d", index, value);
            Player * first_player = game_arena->players[0];
            switch (index) {
                case ROLL_INDEX:
                {
                    float horizontal_balance = value;
                    horizontal_balance = horizontal_balance / (2 * JOYSTICK_AXIS_RANGE);
                    horizontal_balance += 0.5;
                    //INFO("H:%f",horizontal_balance);
                    first_player->SetRoll(horizontal_balance);
                    break;
                }
                case PITCH_INDEX:
                {
                    float pitch_balance = value;
                    pitch_balance /= (2 * JOYSTICK_AXIS_RANGE);
                    pitch_balance += 0.5;
                    //INFO("P:%f",pitch_balance);
                    first_player->SetPitch(pitch_balance);
                    break;
                }
                case RUDDER_INDEX:
                {
                    float pitch_balance = value;
                    pitch_balance /= (2 * JOYSTICK_AXIS_RANGE);
                    pitch_balance += 0.5;
                    //INFO("R:%f",pitch_balance);
                    first_player->SetRudder(pitch_balance);
                    break;
                }
                case THROTTLE_INDEX:
                {
                    float throttle = value;
                    throttle = (JOYSTICK_AXIS_RANGE + throttle) / (JOYSTICK_AXIS_RANGE * 2);
                    throttle = 1 - throttle;
                    INFO("Throttle %f", throttle);
                    first_player->SetThrust(throttle);
                    break;
                }
            }
        }
        
        
        
        void JoystickButtonCallback(int index, bool value)
        {
            INFO("Button %d:%d", index, value);
            Player * first_player = game_arena->players[0];
            switch (index) {
                case FIRE_THRUSTERS_INDEX:
                    first_player->Jump();
                    break;
            }
        }

        void Init()
        {
            glClearColor (0.0f, 0.0f, 0.0f, 0.0f);
            
            float lightPosition[] = { 1,1,0,0 };

            glLightfv( GL_LIGHT0, GL_POSITION, lightPosition);
            glEnable (GL_LIGHTING);
            glEnable (GL_LIGHT0);
            glColorMaterial ( GL_FRONT, GL_AMBIENT_AND_DIFFUSE );
            glEnable ( GL_COLOR_MATERIAL );
            glEnable(GL_DEPTH_TEST);
            
            glCullFace(GL_FRONT);
            
            glEnable (GL_BLEND);
            glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
        
        void DrawGlutString( const char * s )
        {
            for (size_t i = 0; i < strlen(s); ++i) {
                glutBitmapCharacter(glutBitmapHelvetica18,s[i]);
            }
        }

        void Draw()
        {
            int width = glutGet (GLUT_WINDOW_WIDTH);
            int height = glutGet (GLUT_WINDOW_HEIGHT);
            
            glViewport (0, 0, width, height);
            glMatrixMode (GL_PROJECTION);
            glLoadIdentity ();
            gluOrtho2D (0, width, 0, height);
            glMatrixMode (GL_MODELVIEW);
            glLoadIdentity ();
            glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            Projection(width,height,0);
            
            game_arena->Draw();
            
            glutSwapBuffers ();
        }
        
        void Keyboard (uint8_t key, int key_x, int key_y)
        {
            Player * first_player = game_arena->players[0];
            switch (key) {
                case '0' ... '9':
                    first_player->SetThrust(((float)(key - '0')) / 10.0f);
                    break;
                case 'w':
                    first_player->SetThrottle(1.0f);
                    break;
                case 's':
                    first_player->SetThrottle(0.0f);
                    first_player->SetRudder(0.5f);
                    break;
                case 'a':
                    first_player->SetRudder(0);
                    break;
                case 'd':
                    first_player->SetRudder(1.0f);
                    break;
                case 'r':
                    first_player->Reset();
                    break;
                case 27:
                    exit(0);
                    break;
            }
        }
        
        void Update(int elapsed_ms)
        {
            game_arena->Update(elapsed_ms/1000.0);
#if USE_SDL_JOYSTICK
            for (int i = 0; i < SDL_JoystickNumAxes(joystick); i++) {
                int n = SDL_JoystickGetAxis(joystick, i);
                if (n != 0) {
                    printf("SDL Joy (%d)\n", n);
                }
            }
            int x = SDL_JoystickGetAxis(joystick, 0);
            int y = SDL_JoystickGetAxis(joystick, 1);
            int z = SDL_JoystickGetAxis(joystick, 2);
            if (x != 0 || y != 0 || z != 0) {
                printf("SDL Joy (%d,%d,%d)\n", x,y,z);
                SDL_JoystickGetButton(joystick,0);
            }
#endif
#if USE_LINUX_JOYSTICK
            joystick->update();
#endif //USE_LINUX_JOYSTICK
        }
        
        void Mouse(int button, int state, int x, int y)
        {
            int width = glutGet (GLUT_WINDOW_WIDTH);
            int height = glutGet (GLUT_WINDOW_HEIGHT);
        }

    private:

        void Projection (int width, int height, int perspective)
        {
            float ratio = (float)width / (float)height;

            glMatrixMode (GL_PROJECTION);
            glLoadIdentity ();
            if(perspective == 1) {
                gluPerspective (100, ratio, 1, 10000);
            } else {
                gluPerspective (60, ratio, 1, 10000);
            }
            glMatrixMode (GL_MODELVIEW);
            glLoadIdentity ();
            
            return;
        }
    };
}


static HoverCrafty::Main * root = NULL;

#define GLUT_ROOT_HELPER(name) static void name## Helper() { root-> name (); }

GLUT_ROOT_HELPER(Draw);
static void KeyboardHelper(uint8_t key, int x, int y) { root-> Keyboard (key,x,y); }
#if USE_GLUT_JOYSTICK
// //static void JoystickHelper(uint buttons, int x, int y, int z) { root-> Joystick (buttons,x,y,z); }
#endif
static void TimerHelper(int ignore) {
    root->Update(TICK_MS);
    glutPostRedisplay ();
    glutTimerFunc(TICK_MS, TimerHelper,0);
}
static void MouseHelper(int a,int b,int c,int d) { root->Mouse(a,b,c,d); }

int main(int argc, char** argv)
{
    glutInit (&argc,argv);
    glutInitDisplayMode (GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize (1024, 1024);
    //glutInitWindowPosition (50, 50);
    glutCreateWindow ("HoverCrafty");
    
    dInitODE2(0);
    dAllocateODEDataForThread(dAllocateMaskAll);
    
#if USE_SDL_JOYSTICK
    if (SDL_Init( SDL_INIT_JOYSTICK ) < 0)
    {
        fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
        exit(1);
    }
#endif //USE_SDL_JOYSTICK

    root = new HoverCrafty::Main();
    root->Init();
    
    glutDisplayFunc(DrawHelper);
    glutKeyboardFunc(KeyboardHelper);
#if USE_GLUT_JOYSTICK
    //glutJoystickFunc(JoystickHelper,30);
#endif
    glutMouseFunc(MouseHelper);
    glutTimerFunc(TICK_MS, TimerHelper, 0);

    glutMainLoop ();
    return 0;
}